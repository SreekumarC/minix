"""Adaptive Cruise Control decision layer.

Pure logic, no I/O — app.py's adas_loop feeds it sensor snapshots and
applies the result to the serial link. That keeps it testable off the car.

Sensor modes
------------
CAMERA  (default)  gap + closing rate from the HC-SR04 ultrasonic; the
                   camera must be healthy (fresh frames, fps, exposure,
                   contrast, sharpness) and able to read the road (lanes
                   found in a good share of recent frames). Speed is held
                   by a PI loop on gap error + D on range rate.
V2V     (fallback) camera can't see the road/vehicles properly -> lead
                   speed and acceleration come from the target vehicle
                   over Wi-Fi (v2v.py). Ultrasonic still gives the gap
                   when it has an echo; the lead's speed is used as
                   feed-forward so MiniX matches it instead of chasing
                   a differentiated ultrasonic signal.
FAULT              camera bad AND no fresh V2V, or the Arduino link is
                   down -> ACC disengages and the car is stopped.

Switching uses hysteresis: camera must be bad for ACC_TO_V2V_S before
going to V2V, and good for ACC_TO_CAMERA_S before coming back, so a
single dark frame doesn't make the mode flicker.

Speed output is a PWM request (0..set speed). The Arduino only accepts
speed deciles ('0'-'9'), so the request is quantised there; below
ACC_PWM_MIN_MOVE the motors stall, so ACC commands a stop instead.
"""

import collections

OFF, CAMERA, V2V, FAULT = "off", "camera", "v2v", "fault"


class ACC:
    def __init__(self, cfg):
        self.c = cfg
        self.enabled = False
        self.mode = OFF
        self.reason = ""           # last mode change / disengage reason (shown in UI)
        self.pwm = 0.0             # current (rate-limited) PWM request
        self.decile = 0
        self.gap = None            # cm, None = no echo
        self.lead = False
        self.range_rate = 0.0      # cm/s, + = opening, - = closing
        self.cam_ok = False
        self.cam_reason = ""
        self._integ = 0.0
        self._using_v2v = False
        self._bad_since = None
        self._good_since = None
        self._last_t = None
        self._last_dist = None
        self._last_dist_ts = None
        self._lanes = collections.deque()   # (t, lanes_seen)

    # ---------- engagement ----------
    def engage(self, now):
        self.enabled = True
        self.mode = CAMERA
        self.reason = "engaged"
        self.pwm = 0.0
        self._integ = 0.0
        self._using_v2v = False
        self._bad_since = self._good_since = None
        self._last_t = now
        self._lanes.clear()

    def disengage(self, reason):
        if self.enabled:
            self.reason = reason
        self.enabled = False
        self.mode = OFF
        self.pwm = 0.0
        self.decile = 0

    # ---------- camera health ----------
    def camera_health(self, now, cam, ldw):
        """Return (ok, reason). cam exposes .ok, .fps_measured,
        .last_frame_ts and .quality = {bright, contrast, sharp}."""
        c = self.c
        if ldw is not None:
            self._lanes.append((now, bool(ldw.lanes_seen)))
        while self._lanes and now - self._lanes[0][0] > c.ACC_CAM_LANE_WINDOW_S:
            self._lanes.popleft()

        if cam is None or not cam.ok:
            return False, "camera not open"
        if now - getattr(cam, "last_frame_ts", 0.0) > 0.5:
            return False, "no frames"
        if cam.fps_measured < c.ACC_CAM_MIN_FPS:
            return False, f"low fps ({cam.fps_measured:.0f})"
        q = getattr(cam, "quality", None) or {}
        if q:
            if q["bright"] < c.ACC_CAM_MIN_BRIGHT:
                return False, "too dark"
            if q["bright"] > c.ACC_CAM_MAX_BRIGHT:
                return False, "overexposed"
            if q["contrast"] < c.ACC_CAM_MIN_CONTRAST:
                return False, "low contrast"
            if q["sharp"] < c.ACC_CAM_MIN_SHARPNESS:
                return False, "blurred"
        if c.ACC_CAM_REQUIRE_LANES:
            if ldw is None:
                return False, "LDW off"
            # need a full window of history before judging the road
            if self._lanes and now - self._lanes[0][0] >= 0.8 * c.ACC_CAM_LANE_WINDOW_S:
                ratio = sum(1 for _, s in self._lanes if s) / len(self._lanes)
                if ratio < c.ACC_CAM_LANE_RATIO:
                    return False, f"road not readable ({ratio:.0%} lanes)"
        return True, "ok"

    # ---------- main step ----------
    def update(self, now, snap, cam, ldw, lead_pkt, v2v_fresh, set_pwm):
        """One control step. Returns dict(action, pwm, decile, reason)
        where action is 'drive' | 'stop' | 'disengage'."""
        c = self.c
        dt = min(0.2, max(0.0, now - (self._last_t or now)))
        self._last_t = now

        # --- ultrasonic gap + range rate (only on new readings) ---
        dist = snap.get("dist", 999)
        dts = snap.get("dist_ts")
        if dts is not None and dts != self._last_dist_ts:
            if dist < 999 and self._last_dist is not None and self._last_dist < 999:
                ddt = dts - self._last_dist_ts
                if 0.02 < ddt < 1.0:
                    raw = (dist - self._last_dist) / ddt
                    raw = max(-300.0, min(300.0, raw))
                    self.range_rate = 0.6 * self.range_rate + 0.4 * raw
            elif dist >= 999:
                self.range_rate = 0.0
            self._last_dist, self._last_dist_ts = dist, dts
        self.gap = dist if dist < 999 else None
        self.lead = self.gap is not None and self.gap < c.ACC_DETECT_CM

        # --- mode selection ---
        self.cam_ok, self.cam_reason = self.camera_health(now, cam, ldw)
        if not self.enabled:
            return {"action": "stop", "pwm": 0, "decile": 0, "reason": self.reason}
        if not snap.get("link_ok", False):
            self.disengage("Arduino link lost")
            self.mode = FAULT
            return {"action": "disengage", "pwm": 0, "decile": 0, "reason": self.reason}

        if not self._using_v2v:
            if self.cam_ok:
                self._bad_since = None
            else:
                self._bad_since = self._bad_since or now
                if now - self._bad_since >= c.ACC_TO_V2V_S:
                    self._using_v2v = True
                    self._good_since = None
                    self.reason = f"camera: {self.cam_reason} -> V2V"
        else:
            if self.cam_ok:
                self._good_since = self._good_since or now
                if now - self._good_since >= c.ACC_TO_CAMERA_S:
                    self._using_v2v = False
                    self._bad_since = None
                    self.reason = "camera recovered -> camera"
            else:
                self._good_since = None

        if self._using_v2v and not (v2v_fresh and lead_pkt):
            if self.cam_ok:      # V2V gone but camera usable again: take it now
                self._using_v2v = False
                self.reason = "V2V lost, camera ok -> camera"
            else:
                self.disengage(f"camera: {self.cam_reason}; no V2V data")
                self.mode = FAULT
                return {"action": "disengage", "pwm": 0, "decile": 0, "reason": self.reason}

        self.mode = V2V if self._using_v2v else CAMERA

        # --- control law ---
        target = float(set_pwm)
        if self.lead:
            err = self.gap - c.ACC_GAP_CM          # + = too far
            if self.mode == V2V:
                ff = (lead_pkt["v_mps"] * c.ACC_PWM_PER_MPS
                      + c.ACC_KA * lead_pkt["ax_mps2"])
                self._integ = self.pwm             # keep bumpless for a switch back
            else:
                self._integ += c.ACC_KI * err * dt
                self._integ = max(0.0, min(float(set_pwm), self._integ))
                ff = self._integ
            target = ff + c.ACC_KP * err + c.ACC_KD * self.range_rate

            if self.gap < c.ACC_STOP_GAP_CM and self.range_rate <= 5:
                target = 0.0
            if (self.mode == V2V and lead_pkt["v_mps"] < 0.05
                    and self.gap < c.ACC_GAP_CM + 20):
                target = 0.0                       # lead has stopped: stop behind it
        else:
            # free road (or lead beyond ultrasonic range). In V2V mode the
            # lead is known to exist, so don't exceed it by much while blind.
            if self.mode == V2V and lead_pkt and lead_pkt["v_mps"] > 0.05:
                target = min(target, (lead_pkt["v_mps"] + c.ACC_V2V_BLIND_MARGIN_MPS)
                             * c.ACC_PWM_PER_MPS)
            self._integ = self.pwm

        target = max(0.0, min(float(set_pwm), target))

        # rate limit: gentle accel, firmer decel (no jerks)
        up, down = c.ACC_RAMP_UP_PWM_S * dt, c.ACC_RAMP_DOWN_PWM_S * dt
        if self.pwm < c.ACC_PWM_MIN_MOVE <= target:
            self.pwm = float(c.ACC_PWM_MIN_MOVE)   # skip the dead band below stall
        if target > self.pwm:
            self.pwm = min(target, self.pwm + up)
        else:
            self.pwm = max(target, self.pwm - down)
        if target == 0.0 and self.pwm < c.ACC_PWM_MIN_MOVE:
            self.pwm = 0.0

        if self.pwm < c.ACC_PWM_MIN_MOVE:
            # below stall: hold the request (so ramp-up continues) but command stop
            self.decile = 0
            action = "stop"
        else:
            self.decile = max(1, min(9, round(self.pwm / 255 * 9)))
            action = "drive"
        return {"action": action, "pwm": self.pwm, "decile": self.decile,
                "reason": self.reason}

    def status(self):
        return {
            "enabled": self.enabled,
            "mode": self.mode,
            "reason": self.reason,
            "gap_cm": self.gap,
            "lead": self.lead,
            "range_rate": round(self.range_rate, 1),
            "pwm": int(self.pwm),
            "decile": self.decile,
            "cam_ok": self.cam_ok,
            "cam_reason": self.cam_reason,
        }
