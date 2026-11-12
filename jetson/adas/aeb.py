"""Autonomous emergency braking decision logic.

Runs on the Jetson (the ADAS domain ECU). The Arduino keeps its own
hard failsafe at a shorter distance, so this layer failing does not
mean the vehicle collides — it just means the intervention is later
and less graceful.
"""

import time


class AEB:
    OFF = 0
    WARN = 1
    BRAKE = 2

    def __init__(self, warn_cm, brake_cm, hold_s=2.0):
        self.warn_cm = warn_cm
        self.brake_cm = brake_cm
        self.hold_s = hold_s          # keep the telltale lit this long

        self.level = self.OFF
        self.trigger_dist = None
        self._latched_at = 0.0

    def update(self, dist_cm, moving_forward):
        """Return True on the rising edge of a brake decision."""
        now = time.time()
        fired = False

        if moving_forward and dist_cm < self.brake_cm:
            if self.level != self.BRAKE:
                fired = True
                self.trigger_dist = dist_cm
            self.level = self.BRAKE
            self._latched_at = now
        elif moving_forward and dist_cm < self.warn_cm:
            if now - self._latched_at > self.hold_s:
                self.level = self.WARN
        else:
            if now - self._latched_at > self.hold_s:
                self.level = self.OFF
                self.trigger_dist = None

        return fired

    def status(self):
        return {self.OFF: "clear",
                self.WARN: "caution",
                self.BRAKE: "braking"}[self.level]
