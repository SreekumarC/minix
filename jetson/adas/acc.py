"""Adaptive Cruise Control decision layer.

Pure logic, no I/O — app.py's adas_loop feeds it sensor snapshots and
applies the result to the serial link. That keeps it testable off the car.

Sensor modes
------------
CAMERA  (default)  camera + ultrasonic. The camera must be healthy (fresh
                   frames, fps, exposure, contrast, sharpness) and able to
                   read the scene: lanes found, or the learned target car
                   seen (adas/target_detect.py), in a good share of recent
                   frames. Gap = ultrasonic, or the camera's distance
                   estimate from the target's box width when the
                   ultrasonic has no echo (the nearer of the two when both
                   exist). Speed is held by a PI loop on gap error + D on
                   range rate.
V2V     (fallback) camera can't see the road/vehicles properly -> lead
                   speed and acceleration come from the target vehicle
                   over Wi-Fi (v2v.py). Ultrasonic still gives the gap
                   when it has an echo; the lead's speed is used as
                   feed-forward so MiniX matches it instead of chasing
                   a differentiated ultrasonic signal.
IMU (optional, imu.py) adds two things in any mode when it's present:
  - curve speed cap: yaw rate above ACC_CURVE_YAW_DPS limits the request
    to ACC_CURVE_SPEED_FRAC of set speed (the ultrasonic looks straight
    ahead, so in a bend it can lose the lead or see a wall)
  - V2V speed matching: lead speed (V2V) minus ego speed (IMU, valid only
    shortly after a stop) corrects the open-loop PWM_PER_MPS feed-forward
Without an IMU both are simply skipped.
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
        self._last_dist_ts = None
        self._lanes = collections.deque()   # (t, lanes_seen or target_seen)
        self.gap_src = None        # 'ultrasonic' | 'camera' | None
        self.imu = None            # last IMU snapshot used (status only)
        self.curve = False
        self.cam_soft = False
        self._v2v_ok_t = None
        self._held = False         # stopped behind the lead; wait for the gap to reopen
        self.target_seen = False
        self._prev_gap = None
        self._prev_gap_t = None

    # ---------- engagement ----------
    def engage(self, now):
        self.enabled = True
        self.mode = CAMERA
        self.reason = "engaged"
        self.pwm = 0.0
        self._integ = 0.0
        self._held = False
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
    def camera_health(self, now, cam, ldw, target=None):
        """Return (ok, reason). cam exposes .ok, .fps_measured,
.last_frame_ts and .image_quality = {bright, contrast, sharp}."""
        c = self.c
        self.cam_soft = False      # True = picture fine, only the scene isn't recognised
        if ldw is not None or target is not None:
            scene = bool(ldw is not None and ldw.lanes_seen) or \
                    bool(target is not None and target.seen)
            self._lanes.append((now, scene))
        while self._lanes and now - self._lanes[0][0] > c.ACC_CAM_LANE_WINDOW_S:
            self._lanes.popleft()

        if cam is None or not cam.ok:
            return False, "camera not open"
        if now - getattr(cam, "last_frame_ts", 0.0) > 0.5:
            return False, "no frames"
        if cam.fps_measured < c.ACC_CAM_MIN_FPS:
            return False, f"low fps ({cam.fps_measured:.0f})"
        q = getattr(cam, "image_quality", None) or {}
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
            if ldw is None and target is None:
                return False, "LDW off"
            # need a full window of history before judging the scene
            if self._lanes and now - self._lanes[0][0] >= 0.8 * c.ACC_CAM_LANE_WINDOW_S:
                ratio = sum(1 for _, s in self._lanes if s) / len(self._lanes)
                if ratio < c.ACC_CAM_LANE_RATIO:
                    self.cam_soft = True
                    return False, f"can't read road/vehicle ({ratio:.0%})"
        return True, "ok"

    # ---------- main step ----------
    def update(self, now, snap, cam, ldw, lead_pkt, v2v_fresh, set_pwm, target=None, imu=None):
        """One control step. Returns dict(action, pwm, decile, reason)
        where action is 'drive' | 'stop' | 'disengage'."""
        c = self.c
        dt = min(0.2, max(0.0, now - (self._last_t or now)))
        self._last_t = now

        # --- mode selection inputs ---
        self.cam_ok, self.cam_reason = self.camera_health(now, cam, ldw, target)
        self.target_seen = bool(target is not None and target.seen)

        # --- gap: ultrasonic and/or camera estimate of the learned target ---
        dist = snap.get("dist", 999)
        dts = snap.get("dist_ts")
        us_gap = dist if dist < 999 else None
        cam_gap = None
        if (self.cam_ok and not self._using_v2v and self.target_seen
                and target.est_cm and abs(target.offset) < 0.5):
            cam_gap = float(target.est_cm)
        if us_gap is not None and cam_gap is not None:
            gap, src = min(us_gap, cam_gap), ("ultrasonic" if us_gap <= cam_gap else "camera")
        elif us_gap is not None:
            gap, src = us_gap, "ultrasonic"
        elif cam_gap is not None:
            gap, src = cam_gap, "camera"
        else:
            gap, src = None, None

        # range rate: ultrasonic only on new readings, camera every step
        new_sample = (src == "camera") or (src == "ultrasonic" and dts != self._last_dist_ts)
        if gap is None or src != self.gap_src:
            self.range_rate = 0.0 if gap is None else self.range_rate
            self._prev_gap, self._prev_gap_t = gap, now
        elif new_sample:
            ddt = now - (self._prev_gap_t or now)
            if 0.02 < ddt < 1.0:
                raw = max(-300.0, min(300.0, (gap - self._prev_gap) / ddt))
                a = 0.4 if src == "ultrasonic" else 0.15     # camera estimate is noisier
                self.range_rate = (1 - a) * self.range_rate + a * raw
            self._prev_gap, self._prev_gap_t = gap, now
        self._last_dist_ts = dts
        self.gap, self.gap_src = (int(gap) if gap is not None else None), src
        self.lead = self.gap is not None and self.gap < c.ACC_DETECT_CM
        if not self.enabled:
            return {"action": "stop", "pwm": 0, "decile": 0, "reason": self.reason}
        if not snap.get("link_ok", False):
            self.disengage("Arduino link lost")
            self.mode = FAULT
            return {"action": "disengage", "pwm": 0, "decile": 0, "reason": self.reason}

        # V2V availability, bridging short Wi-Fi dropouts with the last packet
        if v2v_fresh and lead_pkt:
            self._v2v_ok_t = now
        v2v_ok = bool(lead_pkt) and self._v2v_ok_t is not None and \
            now - self._v2v_ok_t < c.ACC_V2V_HOLD_S

        if not self._using_v2v:
            if self.cam_ok:
                self._bad_since = None
            else:
                self._bad_since = self._bad_since or now
                if now - self._bad_since >= c.ACC_TO_V2V_S:
                    if v2v_ok:
                        self._using_v2v = True
                        self._good_since = None
                        self.reason = f"camera: {self.cam_reason} -> V2V"
                    elif self.cam_soft:
                        # picture is fine, there's just no lane/target in view and
                        # no V2V: keep going on camera + ultrasonic (cruise until
                        # the ultrasonic finds a lead) instead of shutting ACC off
                        self.reason = "no target in view, no V2V -> ultrasonic"
        else:
            if self.cam_ok:
                self._good_since = self._good_since or now
                if now - self._good_since >= c.ACC_TO_CAMERA_S:
                    self._using_v2v = False
                    self._bad_since = None
                    self.reason = "camera recovered -> camera"
            else:
                self._good_since = None

        if self._using_v2v and not v2v_ok:
            if self.cam_ok or self.cam_soft:   # V2V gone, picture usable: camera + ultrasonic
                self._using_v2v = False
                self._bad_since = None
                self.reason = "V2V lost -> camera + ultrasonic"
            else:
                self.disengage(f"camera: {self.cam_reason}; no V2V data")
                self.mode = FAULT
                return {"action": "disengage", "pwm": 0, "decile": 0, "reason": self.reason}

        if (not self._using_v2v and not self.cam_ok and not self.cam_soft
                and self._bad_since and now - self._bad_since >= c.ACC_TO_V2V_S):
            self.disengage(f"camera: {self.cam_reason}; no V2V data")
            self.mode = FAULT
            return {"action": "disengage", "pwm": 0, "decile": 0, "reason": self.reason}

        self.mode = V2V if self._using_v2v else CAMERA

        # --- control law ---
        target = float(set_pwm)
        if self.lead:
            err = self.gap - c.ACC_GAP_CM          # + = too far
            # integrator on gap error in BOTH modes: it finds the PWM that holds
            # the gap even when the lead's reported speed is off (the target
            # has no wheel encoders, so its IMU-only speed can read ~0 while
            # it drives). In V2V the lead's speed is a floor under that, plus
            # its acceleration as feed-forward.
            self._integ += c.ACC_KI * err * dt
            self._integ = max(0.0, min(float(set_pwm), self._integ))
            ff = self._integ
            if self.mode == V2V:
                ff = max(ff, lead_pkt["v_mps"] * c.ACC_PWM_PER_MPS) \
                    + c.ACC_KA * lead_pkt["ax_mps2"]
            target = ff + c.ACC_KP * err + c.ACC_KD * self.range_rate

            # stop on the gap predicted ACC_STOP_LOOKAHEAD_S ahead, so motor
            # lag doesn't carry MiniX below the stop gap (or into AEB range)
            predicted = self.gap + min(0.0, self.range_rate) * c.ACC_STOP_LOOKAHEAD_S
            if predicted < c.ACC_STOP_GAP_CM and self.range_rate <= 5:
                self._held = True
            elif self.gap >= c.ACC_RESTART_GAP_CM:
                self._held = False
            if self._held:
                # creeping at the stop line just ends in a coast past it; stay
                # stopped until the gap has reopened to ACC_RESTART_GAP_CM
                target = 0.0
                self._integ = min(self._integ, float(c.ACC_PWM_MIN_MOVE))
            if (self.mode == V2V and lead_pkt["v_mps"] < 0.05
                    and self.gap < c.ACC_GAP_CM + 20 and self.range_rate <= 5):
                # lead reports stopped AND the gap isn't opening: stop behind
                # it (if the gap opens, the lead is moving whatever it reports)
                target = 0.0
        else:
            self._held = False
            # free road (or lead beyond ultrasonic range). In V2V mode the
            # lead is known to exist, so don't exceed it by much while blind.
            if self.mode == V2V and lead_pkt and lead_pkt["v_mps"] > 0.05:
                target = min(target, (lead_pkt["v_mps"] + c.ACC_V2V_BLIND_MARGIN_MPS)
                             * c.ACC_PWM_PER_MPS)
            self._integ = self.pwm

        # --- IMU refinements (skipped when imu is None) ---
        self.imu, self.curve = imu, False
        if imu:
            if (self.mode == V2V and lead_pkt and imu["v_valid"]
                    and imu["v_mps"] > 0.05 and lead_pkt["v_mps"] > 0.05
                    and target > 0.0):
                corr = c.ACC_KV_PWM_PER_MPS * (lead_pkt["v_mps"] - imu["v_mps"])
                target += max(-c.ACC_KV_MAX_PWM, min(c.ACC_KV_MAX_PWM, corr))
            if abs(imu["yaw_dps"]) > c.ACC_CURVE_YAW_DPS:
                self.curve = True
                target = min(target, c.ACC_CURVE_SPEED_FRAC * float(set_pwm))

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
            "gap_src": self.gap_src,
            "target_seen": self.target_seen,
            "cam_ok": self.cam_ok,
            "cam_reason": self.cam_reason,
            "imu": self.imu is not None,
            "curve": self.curve,
        }
