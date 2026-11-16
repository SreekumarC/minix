#!/usr/bin/env python3
"""MiniX web server — drive control, live video, ADAS status.

Run:  python3 app.py
Then open http://<jetson-ip>:5000 from the phone or laptop.
"""

import atexit
import threading
import time

from flask import Flask, Response, jsonify, render_template, request

import config
from adas.acc import ACC
from adas.aeb import AEB
from adas.ldw import LDW
from camera import Camera
from oled_display import OLEDStatus
from serial_link import SerialLink
from v2v import TargetReceiver

app = Flask(__name__)

link = SerialLink(config.SERIAL_PORT, config.SERIAL_BAUD)
ldw = LDW(config.LDW_OFFSET_WARN) if config.LDW_ENABLED else None
cam = Camera(config.CAM_DEVICE, config.CAM_WIDTH, config.CAM_HEIGHT,
             config.CAM_FPS, config.JPEG_QUALITY, ldw)
aeb = AEB(config.AEB_WARN_CM, config.AEB_BRAKE_CM)
oled = OLEDStatus()  # fails safe — .ok is False if the display isn't wired/detected
v2v = TargetReceiver(config.V2V_PORT, config.V2V_CMD_PORT, config.V2V_STALE_S)  # fails safe — .ok
acc = ACC(config)

speed = config.DEFAULT_SPEED
aeb_events = 0
lka_events = 0
lka_enabled = True  # web-togglable; default ON

# ACC needs a live driver UI: if no browser has polled /status within
# ACC_UI_HEARTBEAT_S, ACC disengages and stops the car. (ACC itself
# resends F + speed, which would otherwise keep the Arduino watchdog fed
# with nobody watching.)
last_ui_poll = 0.0
acc_lock = threading.Lock()   # guards acc.engage/disengage vs. the loop

# LKA defers to the driver: it only corrects when the driver hasn't
# touched steering (joystick sideways, or left/right buttons) recently.
last_manual_steer_ts = 0.0
LKA_DEFER_SECONDS = 1.0
LKA_GAIN = 120       # steer_norm per unit of LDW offset (offset is -1..1)
LKA_MAX_CORRECTION = 45  # cap, in steer_norm units (-100..100 scale)


# ---------- ADAS decision loop ----------
def acc_step(s, now, state):
    """Run one ACC step and apply it to the Arduino. `state` carries the
    last command sent so we only resend on change or every ACC_CMD_PERIOD_S."""
    with acc_lock:
        if not acc.enabled:
            # still evaluate sensors so the UI shows camera/V2V health
            acc.update(now, s, cam, ldw, v2v.get(), v2v.is_fresh(), speed)
            return
        if cam.detect_mode:
            acc.disengage("object detection mode")
            link.stop()
            return
        if now - last_ui_poll > config.ACC_UI_HEARTBEAT_S:
            acc.disengage("driver UI lost")
            link.stop()
            return
        out = acc.update(now, s, cam, ldw, v2v.get(), v2v.is_fresh(), speed)

    if out["action"] == "disengage":
        link.stop()
        state["sent"] = None
        return
    cmd = ("stop",) if out["action"] == "stop" else ("drive", out["decile"])
    if cmd != state.get("sent") or now - state.get("t", 0) >= config.ACC_CMD_PERIOD_S:
        if cmd[0] == "stop":
            link.stop()
        else:
            link.set_speed(out["decile"] / 9 * 255)  # set_speed re-quantises to the same decile
            link.forward()
        state["sent"], state["t"] = cmd, now


def adas_loop():
    global aeb_events, lka_events
    last_oled_update = 0.0
    acc_tx = {}
    while True:
        s = link.snapshot()
        now = time.time()
        moving_fwd = s["l"] > 0 or s["r"] > 0
        if aeb.update(s["dist"], moving_fwd):
            link.ebrake()
            aeb_events += 1
            with acc_lock:
                acc.disengage("AEB intervention")

        acc_step(s, now, acc_tx)

        # LKA: only correct while driving forward, only on a genuine
        # departure, and only if the driver hasn't steered manually in
        # the last LKA_DEFER_SECONDS. Any manual steer input wins
        # immediately — LKA never fights the driver.
        # Sign: ldw.offset > 0 means the lane midpoint appears to the
        # RIGHT of image center, i.e. the car has drifted toward the
        # LEFT edge of the lane and needs to steer RIGHT to recenter.
        # With this firmware's calibration, positive steer_norm = right,
        # so the correction is +offset * gain (not negated).
        if (lka_enabled and ldw and ldw.lanes_seen and ldw.departing
                and s["state"] == 1
                and time.time() - last_manual_steer_ts > LKA_DEFER_SECONDS):
            correction = max(-LKA_MAX_CORRECTION, min(LKA_MAX_CORRECTION,
                              ldw.offset * LKA_GAIN))
            link.set_steer_angle(correction)
            lka_events += 1

        # OLED at ~4Hz — no need to redraw at the loop's full 20Hz
        now = time.time()
        if oled.ok and now - last_oled_update > 0.25:
            last_oled_update = now
            state_txt = {0: "STOP", 1: "FWD", 2: "REV", 3: "MRM!"}.get(s["state"], "?")
            if acc.enabled:
                state_txt += " ACC-" + ("V2V" if acc.mode == "v2v" else "CAM")
            oled.update(
                state=state_txt,
                dist=s["dist"],
                speed=speed,
                aeb=aeb.status(),
                ldw=ldw.status() if ldw else "off",
                link_ok=s["link_ok"],
                fps=cam.fps_measured,
            )

        time.sleep(0.05)


threading.Thread(target=adas_loop, daemon=True).start()


# ---------- routes ----------
@app.route("/")
def index():
    return render_template("index.html")


@app.route("/video")
def video():
    return Response(cam.mjpeg(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/status")
def status():
    global last_ui_poll
    last_ui_poll = time.time()  # driver UI heartbeat for ACC
    s = link.snapshot()
    return jsonify(
        dist=s["dist"],
        left=s["l"],
        right=s["r"],
        state=s["state"],
        link_ok=s["link_ok"],
        aeb=aeb.status(),
        aeb_dist=aeb.trigger_dist,
        aeb_events=aeb_events,
        ldw=ldw.status() if ldw else "off",
        ldw_offset=round(ldw.offset, 3) if ldw else 0,
        lka=("active" if (lka_enabled and ldw and ldw.lanes_seen and ldw.departing
                           and s["state"] == 1
                           and time.time() - last_manual_steer_ts > LKA_DEFER_SECONDS)
             else "standby"),
        lka_enabled=lka_enabled,
        lka_events=lka_events,
        detect_mode=cam.detect_mode,
        acc=acc.status(),
        v2v=v2v.status(),
        cam_quality={k: round(v, 1) for k, v in (cam.quality or {}).items()},
        fps=round(cam.fps_measured, 1),
        speed=speed,
    )


DRIVE_ACTIONS = {"forward", "reverse", "left", "right", "center", "joystick"}


# While ACC is engaged it owns the throttle, like a real car:
#   - Stop / E-brake / Reverse / joystick pulled back = driver "brake" -> ACC off
#   - Forward is ignored (ACC is already driving)
#   - Steering (buttons or joystick sideways) still works, steer-only
ACC_CANCEL_ACTIONS = {"stop", "ebrake", "reverse", "detect_on"}


@app.route("/cmd", methods=["POST"])
def cmd():
    global speed, last_manual_steer_ts, lka_enabled, last_ui_poll
    data = request.get_json(force=True, silent=True) or {}
    action = data.get("action", "")

    if action == "acc_on":
        s = link.snapshot()
        if cam.detect_mode:
            return jsonify(ok=False, error="turn off object detection first"), 409
        if not s["link_ok"]:
            return jsonify(ok=False, error="no Arduino link"), 409
        if aeb.level == AEB.BRAKE:
            return jsonify(ok=False, error="AEB active"), 409
        if "speed" in data:
            speed = max(0, min(255, int(data["speed"])))
        last_ui_poll = time.time()
        with acc_lock:
            acc.engage(time.time())
        return jsonify(ok=True)
    if action == "acc_off":
        with acc_lock:
            acc.disengage("driver switched off")
        link.stop()
        return jsonify(ok=True)
    if action == "v2v_zero":
        ok, error = v2v.send_cmd("ZERO")
        return jsonify(ok=ok, error=error)

    if acc.enabled:
        if action in ACC_CANCEL_ACTIONS or (
                action == "joystick" and float(data.get("throttle", 0)) < -10):
            with acc_lock:
                acc.disengage(f"driver {action}")
            # fall through: the stop/reverse/joystick itself is executed below
        elif action == "forward":
            return jsonify(ok=True, blocked="acc")
        elif action == "joystick":
            steer_val = float(data.get("steer", 0))
            if abs(steer_val) > 3:
                last_manual_steer_ts = time.time()
            link.set_steer_angle(steer_val)   # steer only — ACC keeps the throttle
            return jsonify(ok=True, acc="steer_only")
        elif action == "speed":
            # slider = ACC set speed; ACC applies it, don't send it directly
            if "speed" in data:
                speed = max(0, min(255, int(data["speed"])))
            return jsonify(ok=True, acc="set_speed")

    # Object detection mode: "when YOLO runs, nothing else runs." Block
    # every drive-affecting action while it's on — stop/ebrake still
    # work, since halting the car is never something we want to block.
    if cam.detect_mode and action in DRIVE_ACTIONS:
        return jsonify(ok=True, blocked="detect_mode")

    if "speed" in data:
        speed = max(0, min(255, int(data["speed"])))
        link.set_speed(speed)

    # AEB overrides forward requests while it is intervening
    if action == "forward" and aeb.level == AEB.BRAKE:
        link.ebrake()
        return jsonify(ok=True, blocked="aeb")

    if action == "joystick" and aeb.level == AEB.BRAKE and float(data.get("throttle", 0)) > 0:
        link.ebrake()
        return jsonify(ok=True, blocked="aeb")

    if action == "forward":
        link.forward()
    elif action == "reverse":
        link.reverse()
    elif action == "left":
        last_manual_steer_ts = time.time()
        link.steer_left()
    elif action == "right":
        last_manual_steer_ts = time.time()
        link.steer_right()
    elif action == "center":
        last_manual_steer_ts = time.time()
        link.steer_center()
    elif action == "stop":
        link.stop()
    elif action == "ebrake":
        link.ebrake()
    elif action == "speed":
        pass
    elif action == "joystick":
        steer_val = data.get("steer", 0)
        if abs(float(steer_val)) > 3:  # matches the Arduino's own deadzone
            last_manual_steer_ts = time.time()
        link.joystick(steer_val, data.get("throttle", 0))
    elif action == "lka_on":
        lka_enabled = True
    elif action == "lka_off":
        lka_enabled = False
    elif action == "detect_on":
        link.stop()  # "nothing else runs" starts with the car actually stopping
        ok, error = cam.set_detect_mode(True)
        if not ok:
            return jsonify(ok=False, error=f"detection unavailable: {error}"), 500
    elif action == "detect_off":
        cam.set_detect_mode(False)
        # Deliberately does NOT auto-resume driving — car stays stopped
        # until the driver presses Forward again, same policy as MRM.
    else:
        return jsonify(ok=False, error="unknown action"), 400

    return jsonify(ok=True)


DEBUG_HTML = """<!doctype html>
<html><head><title>MiniX serial debug</title>
<style>
  body { font-family: monospace; background: #111; color: #ddd; margin: 0; padding: 1rem; }
  h1 { font-size: 1rem; color: #9f9; }
  #log { background: #000; border: 1px solid #333; height: 60vh; overflow-y: auto;
         padding: 0.5rem; white-space: pre-wrap; font-size: 0.85rem; }
  .RX { color: #6cf; } .TX { color: #fc6; } .ERR { color: #f66; font-weight: bold; }
  button { font-family: monospace; font-size: 1rem; margin: 2px; padding: 6px 12px;
           background: #222; color: #ddd; border: 1px solid #555; cursor: pointer; }
  button:active { background: #444; }
  #status { margin: 0.5rem 0; }
</style></head>
<body>
  <h1>MiniX raw serial debug</h1>
  <div id="status">link: <span id="linkOk">?</span></div>
  <div>
    <button onclick="send('F')">F fwd</button>
    <button onclick="send('B')">B rev</button>
    <button onclick="send('S')">S stop</button>
    <button onclick="send('E')">E ebrake</button>
    <button onclick="send('L')">L left</button>
    <button onclick="send('R')">R right</button>
    <button onclick="send('C')">C center</button>
    <button onclick="send('5')">5 speed</button>
    <button onclick="send('9')">9 speed</button>
  </div>
  <div id="log"></div>
<script>
function send(c) {
  fetch('/debug/send', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({char:c})})
    .then(r => r.json()).then(j => { if (!j.ok) alert('send failed: ' + j.error); });
}
function poll() {
  fetch('/debug/serial').then(r => r.json()).then(j => {
    const el = document.getElementById('log');
    const atBottom = el.scrollHeight - el.scrollTop - el.clientHeight < 30;
    el.innerHTML = j.log.map(e =>
      `<span class="${e.dir}">[${e.dir}] ${e.raw}</span>`).join('\\n');
    if (atBottom) el.scrollTop = el.scrollHeight;
  });
  fetch('/status').then(r => r.json()).then(j => {
    document.getElementById('linkOk').textContent = j.link_ok ? 'OK' : 'DOWN';
  }).catch(() => {});
}
setInterval(poll, 400);
poll();
</script>
</body></html>"""


@app.route("/debug")
def debug_page():
    return DEBUG_HTML


@app.route("/debug/serial")
def debug_serial():
    return jsonify(log=link.log_snapshot())


@app.route("/debug/send", methods=["POST"])
def debug_send():
    data = request.get_json(force=True, silent=True) or {}
    c = data.get("char", "")
    if not c or len(c) != 1:
        return jsonify(ok=False, error="send exactly one character"), 400
    link.raw_send(c)
    return jsonify(ok=True, sent=c)


@atexit.register
def _shutdown():
    link.close()
    cam.close()
    oled.close()


if __name__ == "__main__":
    print(f"camera opened: {cam.ok}")
    print(f"serving on http://0.0.0.0:{config.PORT}")
    app.run(host=config.HOST, port=config.PORT,
            threaded=True, debug=False)
