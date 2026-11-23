"""Ego IMU — ISM330DHCX 6-axis (accel + gyro) on the Jetson's 40-pin header.

Wiring (as built): VCC -> pin 17 (3V3), GND -> pin 25, SDA -> pin 27,
SCL -> pin 28  =>  I2C bus 1, address 0x6B (SDO high). Check with
`i2cdetect -y -r 1` (0x25 / 0x40 showing "UU" are kernel devices, ignore).

Mounting: board flat, Z up (reads +1 g at rest), next to the Arduino. The
vehicle-forward direction was measured with a nose-up tilt test:
forward = IMU_FWD_DEG (-83 deg) from board +X towards +Y, i.e. roughly
board -Y with a 7 deg skew. Everything below is rotated into vehicle axes:
    a_long  m/s^2  + = accelerating forward
    a_lat   m/s^2  + = towards the left
    yaw_dps deg/s  + = turning left (counter-clockwise seen from above)

Ego speed: there is no wheel encoder, so v_mps integrates a_long and is
reset to 0 (zero-velocity update) whenever the Arduino reports the motors
stopped. Integration drifts, so v_valid goes False once the car has been
moving longer than IMU_SPEED_MAX_S without a stop; consumers must check it.

Gravity and gyro bias are learned while stationary (app.py calls
set_stationary()), so a slightly tilted mount or a sloped floor is
absorbed automatically.

Fails safe, like SerialLink/Camera/OLEDStatus: if smbus2 is missing, the
bus can't be opened or WHO_AM_I is wrong, .ok stays False, a single
message is logged and every getter returns neutral values.
"""

import logging
import math
import threading
import time

logger = logging.getLogger(__name__)

G = 9.80665
WHO_AM_I, CTRL1_XL, CTRL2_G, CTRL3_C = 0x0F, 0x10, 0x11, 0x12
OUTX_L_G = 0x22            # gyro X/Y/Z then accel X/Y/Z, 12 bytes
ACC_LSB_G = 0.061e-3       # +-2 g
GYR_LSB_DPS = 8.75e-3      # +-250 dps


class IMU:
    def __init__(self, bus=1, address=0x6B, fwd_deg=-83.0, rate_hz=100,
                 speed_max_s=8.0):
        self.ok = False
        self.error = ""
        self._addr = address
        self._dt = 1.0 / rate_hz
        self._speed_max_s = speed_max_s
        f = math.radians(fwd_deg)
        self._fwd = (math.cos(f), math.sin(f))          # board XY of vehicle forward
        self._left = (-math.sin(f), math.cos(f))        # forward rotated +90 deg about Z
        self._lock = threading.Lock()
        self._stationary = True
        self._g_ref = [0.0, 0.0, 1.0]                   # board accel at rest, in g
        self._gbias = [0.0, 0.0, 0.0]                   # dps
        self._calibrated = False
        self._still_n = 0                               # stationary samples seen
        self.a_long = self.a_lat = self.yaw_dps = 0.0
        self.v_mps = 0.0
        self.v_valid = False
        self._moving_since = None
        self._ts = 0.0
        try:
            from smbus2 import SMBus
            self._bus = SMBus(bus)
            who = self._bus.read_byte_data(address, WHO_AM_I)
            if who != 0x6B:
                raise IOError(f"WHO_AM_I 0x{who:02X}, expected 0x6B")
            self._bus.write_byte_data(address, CTRL3_C, 0x44)   # BDU + auto-increment
            self._bus.write_byte_data(address, CTRL1_XL, 0x40)  # 104 Hz, +-2 g
            self._bus.write_byte_data(address, CTRL2_G, 0x40)   # 104 Hz, 250 dps
            time.sleep(0.05)
            self.ok = True
            threading.Thread(target=self._run, daemon=True).start()
            logger.info("IMU: ISM330DHCX on bus %d @0x%02X, forward %.0f deg",
                        bus, address, fwd_deg)
        except Exception as e:  # noqa: BLE001 — fail safe by design
            self.error = str(e)
            print(f"IMU not available ({e}) — ACC runs without it")

    # ---- inputs from app.py ----
    def set_stationary(self, still):
        """True while the Arduino reports both motors at 0 (state STOP)."""
        self._stationary = bool(still)

    # ---- worker ----
    def _read(self):
        d = self._bus.read_i2c_block_data(self._addr, OUTX_L_G, 12)
        v = [int.from_bytes(bytes(d[i:i + 2]), "little", signed=True) for i in range(0, 12, 2)]
        return [x * GYR_LSB_DPS for x in v[:3]], [x * ACC_LSB_G for x in v[3:]]

    def _run(self):
        last = time.time()
        fails = 0
        while True:
            try:
                gyr, acc = self._read()
                fails = 0
            except OSError:
                fails += 1
                if fails == 20:
                    print("IMU: I2C read failures — marking not ok")
                    self.ok = False
                time.sleep(0.05)
                continue
            if not self.ok:
                self.ok = True
            now = time.time()
            dt = min(0.05, now - last)
            last = now
            self._step(now, dt, gyr, acc)
            time.sleep(max(0.0, self._dt - (time.time() - now)))

    def _step(self, now, dt, gyr, acc):
        gnorm = math.sqrt(sum(x * x for x in gyr))
        anorm = math.sqrt(sum(x * x for x in acc))
        really_still = self._stationary and gnorm < 5.0 and abs(anorm - 1.0) < 0.08
        if really_still:
            # learn gravity direction and gyro bias (fast at first, then slow)
            self._still_n += 1
            k = 0.05 if self._still_n < 150 else 0.005
            for i in range(3):
                self._g_ref[i] += k * (acc[i] - self._g_ref[i])
                self._gbias[i] += k * (gyr[i] - self._gbias[i])
            self._calibrated = self._still_n >= 100     # ~1 s still
        da = [acc[i] - self._g_ref[i] for i in range(3)]
        a_long = (da[0] * self._fwd[0] + da[1] * self._fwd[1]) * G
        a_lat = (da[0] * self._left[0] + da[1] * self._left[1]) * G
        yaw = gyr[2] - self._gbias[2]
        with self._lock:
            # light low-pass: motor/road vibration is large on a 1:10 car
            self.a_long += 0.2 * (a_long - self.a_long)
            self.a_lat += 0.2 * (a_lat - self.a_lat)
            self.yaw_dps += 0.3 * (yaw - self.yaw_dps)
            if self._stationary:
                self.v_mps = 0.0              # zero-velocity update
                self._moving_since = None
                self.v_valid = self._calibrated
            else:
                self._moving_since = self._moving_since or now
                self.v_mps = max(-3.0, min(3.0, self.v_mps + self.a_long * dt))
                self.v_valid = (self._calibrated
                                and now - self._moving_since < self._speed_max_s)
            self._ts = now

    # ---- outputs ----
    def get(self):
        """Snapshot in vehicle axes, or None if the IMU isn't usable."""
        if not self.ok or time.time() - self._ts > 0.2:
            return None
        with self._lock:
            return {"a_long": self.a_long, "a_lat": self.a_lat,
                    "yaw_dps": self.yaw_dps, "v_mps": self.v_mps,
                    "v_valid": self.v_valid, "calibrated": self._calibrated}

    def status(self):
        s = self.get()
        if s is None:
            return {"ok": False, "error": self.error}
        return {"ok": True, "calibrated": s["calibrated"],
                "a_long": round(s["a_long"], 2), "a_lat": round(s["a_lat"], 2),
                "yaw_dps": round(s["yaw_dps"], 1),
                "v_mps": round(s["v_mps"], 2) if s["v_valid"] else None}


if __name__ == "__main__":
    # Standalone check:  ~/minix-detect/bin/python imu.py
    # Leave the car still for 2 s (calibration), then push / turn it by hand.
    imu = IMU()
    if not imu.ok:
        raise SystemExit(1)
    t0 = time.time()
    while True:
        imu.set_stationary(time.time() - t0 < 2.0)
        print(imu.status())
        time.sleep(0.2)
