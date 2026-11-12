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
from adas.aeb import AEB
from adas.ldw import LDW
from camera import Camera
from oled_display import OLEDStatus
from serial_link import SerialLink

app = Flask(__name__)

link = SerialLink(config.SERIAL_PORT, config.SERIAL_BAUD)
ldw = LDW(config.LDW_OFFSET_WARN) if config.LDW_ENABLED else None
cam = Camera(config.CAM_DEVICE, config.CAM_WIDTH, config.CAM_HEIGHT,
             config.CAM_FPS, config.JPEG_QUALITY, ldw)
aeb = AEB(config.AEB_WARN_CM, config.AEB_BRAKE_CM)
oled = OLEDStatus()  # fails safe — .ok is False if the display isn't wired/detected

speed = config.DEFAULT_SPEED
aeb_events = 0


# ---------- ADAS decision loop ----------
def adas_loop():
    global aeb_events
    last_oled_update = 0.0
    while True:
        s = link.snapshot()
        moving_fwd = s["l"] > 0 or s["r"] > 0
        if aeb.update(s["dist"], moving_fwd):
            link.ebrake()
            aeb_events += 1

        # OLED at ~4Hz — no need to redraw at the loop's full 20Hz
        now = time.time()
        if oled.ok and now - last_oled_update > 0.25:
            last_oled_update = now
            state_txt = {0: "STOP", 1: "FWD", 2: "REV", 3: "MRM!"}.get(s["state"], "?")
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
        fps=round(cam.fps_measured, 1),
        speed=speed,
    )


@app.route("/cmd", methods=["POST"])
def cmd():
    global speed
    data = request.get_json(force=True, silent=True) or {}
    action = data.get("action", "")

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
        link.steer_left()
    elif action == "right":
        link.steer_right()
    elif action == "center":
        link.steer_center()
    elif action == "stop":
        link.stop()
    elif action == "ebrake":
        link.ebrake()
    elif action == "speed":
        pass
    elif action == "joystick":
        link.joystick(data.get("steer", 0), data.get("throttle", 0))
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
