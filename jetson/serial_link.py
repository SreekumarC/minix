"""Serial link to the Arduino UNO drive controller.

Telemetry is consumed on a background thread so nothing in the web
server ever blocks on the port. Commands are written under a lock.

Command protocol (matches minix_drive.ino):
    'F'/'B'/'S'/'E'   drive forward / reverse / stop / ebrake
    'L'/'R'/'C'       steer left / right / center
    '0'-'9'           set base speed to that decile of full scale

Telemetry protocol (unchanged from the previous firmware):
    "U <dist_cm>"        latest ultrasonic reading
    "T <l> <r> <state>"  signed per-side commanded PWM, state code
"""

import collections
import threading
import time

import serial


class SerialLink:
    def __init__(self, port, baud=9600):
        self.port = port
        self.baud = baud
        self.ser = None
        self._lock = threading.Lock()
        self._state = {"dist": 999, "dist_ts": None, "l": 0, "r": 0, "state": 1}
        self._alive = True
        self._last_rx = time.time()

        # Rolling raw log for the /debug page — every line actually read
        # from the port, every char actually written, and any write
        # errors that would otherwise be silently swallowed below.
        self._log = collections.deque(maxlen=300)

        self._try_open()
        threading.Thread(target=self._reader, daemon=True).start()

    def _try_open(self):
        """Attempt to (re)open the port. Never raises — logs instead, so
        a disconnected/not-yet-plugged-in Arduino can't crash the whole
        Flask app on startup. snapshot()'s link_ok will correctly read
        False until this succeeds."""
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
            time.sleep(2.0)  # UNO reboots when the port opens
            self.ser.reset_input_buffer()
            self._log_event("INFO", f"serial port {self.port} opened")
        except Exception as e:
            self.ser = None
            self._log_event("ERR", f"could not open {self.port}: {e}")

    def _log_event(self, direction, text):
        self._log.append({"t": time.time(), "dir": direction, "raw": text})

    # ---------- telemetry ----------
    def _reader(self):
        last_retry = 0.0
        while self._alive:
            if self.ser is None:
                now = time.time()
                if now - last_retry > 2.0:  # retry every 2s, don't spin
                    last_retry = now
                    self._try_open()
                time.sleep(0.2)
                continue

            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except Exception as e:
                self._log_event("ERR", f"read failed: {e}")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None  # triggers a retry above
                time.sleep(0.1)
                continue
            if not line:
                continue

            self._log_event("RX", line)

            p = line.split()
            try:
                if p[0] == "U" and len(p) == 2:
                    with self._lock:
                        self._state["dist"] = int(p[1])
                        self._state["dist_ts"] = time.time()  # lets ACC derive range rate
                        self._last_rx = time.time()
                elif p[0] == "T" and len(p) == 4:
                    with self._lock:
                        self._state["l"] = int(p[1])
                        self._state["r"] = int(p[2])
                        self._state["state"] = int(p[3])
                        self._last_rx = time.time()
            except ValueError:
                continue

    def snapshot(self):
        with self._lock:
            s = dict(self._state)
        s["link_ok"] = (time.time() - self._last_rx) < 1.0
        return s

    def log_snapshot(self, limit=150):
        return list(self._log)[-limit:]

    # ---------- commands ----------
    def _send_char(self, c):
        try:
            with self._lock:
                self.ser.write(c.encode())
            self._log_event("TX", c)
        except Exception as e:
            self._log_event("ERR", f"write failed ({c!r}): {e}")

    def raw_send(self, c):
        """Public entry point for the /debug page — sends exactly one
        character with no other logic (no AEB check, no speed lookup)."""
        self._send_char(c)

    def forward(self):
        self._send_char("F")

    def reverse(self):
        self._send_char("B")

    def stop(self):
        self._send_char("S")

    def ebrake(self):
        self._send_char("E")

    def steer_left(self):
        self._send_char("L")

    def steer_right(self):
        self._send_char("R")

    def steer_center(self):
        self._send_char("C")

    def set_speed(self, pwm):
        """pwm: 0-255, quantized to the 0-9 decile the Arduino expects."""
        pwm = max(0, min(255, int(pwm)))
        decile = round(pwm / 255 * 9)
        self._send_char(str(decile))

    def joystick(self, steer_norm, throttle_norm):
        """steer_norm/throttle_norm: -100..100. Updates steering and
        speed together in one atomic command — see handleJoystick() in
        the sketch. Unlike the single-char commands this needs a
        newline-terminated line, so it bypasses _send_char."""
        steer_norm = max(-100, min(100, int(steer_norm)))
        throttle_norm = max(-100, min(100, int(throttle_norm)))
        line = f"J {steer_norm} {throttle_norm}\n"
        try:
            with self._lock:
                self.ser.write(line.encode())
            self._log_event("TX", line.strip())
        except Exception as e:
            self._log_event("ERR", f"write failed ({line!r}): {e}")

    def set_steer_angle(self, steer_norm):
        """steer_norm: -100..100. Steering ONLY — never touches
        throttle/driveState on the Arduino. This is what LKA uses so a
        lane-centering correction can never start, stop, or change the
        car's speed; see handleSteerOnly() in the sketch."""
        steer_norm = max(-100, min(100, int(steer_norm)))
        line = f"A {steer_norm}\n"
        try:
            with self._lock:
                self.ser.write(line.encode())
            self._log_event("TX", line.strip())
        except Exception as e:
            self._log_event("ERR", f"write failed ({line!r}): {e}")

    def close(self):
        self._alive = False
        self.stop()
        time.sleep(0.2)
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
