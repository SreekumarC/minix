"""V2V link: receives ACC-target telemetry over Wi-Fi (UDP).

The target vehicle (mecanum RC car driven by its own stock remote) runs
firmware/ACC_Target_Telemetry/ACC_Target_Telemetry.ino on an ESP32 and
broadcasts a 56-byte packet at 50 Hz: its speed, acceleration, yaw rate,
heading, distance and dead-reckoned x/y. See docs/v2v_protocol.md.

Fail-safe like SerialLink/Camera/OLEDStatus: if the UDP port can't be
bound, .ok is False and get() returns None — ACC simply never enters
V2V mode, nothing else is affected.

Standalone check on the Jetson (stop the minix service first, or it
holds the port):
    python3 v2v.py                 # print live target data
    python3 v2v.py --zero <ip>     # reset target odometry
    python3 v2v.py --cal  <ip>     # re-calibrate target IMU (target still)
"""

import argparse
import logging
import socket
import struct
import threading
import time

logger = logging.getLogger(__name__)

FMT = "<HBBII11f"            # must match TelemetryPkt in the .ino (56 bytes)
SIZE = struct.calcsize(FMT)
MAGIC = 0xAC01
FIELDS = ("magic", "version", "flags", "seq", "t_ms",
          "v_mps", "ax_mps2", "ay_mps2", "yaw_rate_dps", "heading_deg",
          "dist_m", "x_m", "y_m", "vL_mps", "vR_mps", "vbat_v")


class TargetReceiver:
    def __init__(self, port=5005, cmd_port=5006, stale_s=0.3):
        self.port = port
        self.cmd_port = cmd_port
        self.stale_s = stale_s
        self.ok = False
        self.error = None
        self.target_ip = None
        self.received = 0
        self.dropped = 0
        self._latest = None
        self._rx_time = 0.0
        self._last_seq = None
        self._lock = threading.Lock()
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(("0.0.0.0", port))
            self.sock.settimeout(0.5)
            self.ok = True
            threading.Thread(target=self._run, daemon=True).start()
        except OSError as e:
            self.error = str(e)
            logger.warning("V2V receiver disabled — cannot bind UDP %d: %s", port, e)

    def _run(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError as e:
                logger.warning("V2V receive error: %s", e)
                time.sleep(0.5)
                continue
            if len(data) != SIZE:
                continue
            d = dict(zip(FIELDS, struct.unpack(FMT, data)))
            if d["magic"] != MAGIC:
                continue
            d["imu_ok"] = bool(d["flags"] & 1)
            d["moving"] = bool(d["flags"] & 2)
            d["imu_only"] = bool(d["flags"] & 8)   # no wheel encoders: speed drifts more
            with self._lock:
                # seq resets to 0 when the target reboots — don't count that as drops
                if (self._last_seq is not None and d["seq"] > self._last_seq + 1
                        and d["seq"] - self._last_seq < 1000):
                    self.dropped += d["seq"] - self._last_seq - 1
                self._last_seq = d["seq"]
                self._latest = d
                self._rx_time = time.monotonic()
                self.target_ip = addr[0]
                self.received += 1

    def age(self):
        """Seconds since the last packet, or None if nothing received yet."""
        with self._lock:
            return None if self._latest is None else time.monotonic() - self._rx_time

    def is_fresh(self):
        a = self.age()
        return a is not None and a < self.stale_s

    def get(self):
        with self._lock:
            return dict(self._latest) if self._latest else None

    def send_cmd(self, cmd, ip=None):
        """'ZERO' or 'CAL'. Returns (ok, error)."""
        ip = ip or self.target_ip
        if not ip:
            return False, "no target seen yet"
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.sendto(cmd.encode(), (ip, self.cmd_port))
            s.close()
            return True, None
        except OSError as e:
            return False, str(e)

    def status(self):
        d = self.get()
        a = self.age()
        return {
            "ok": self.ok,
            "fresh": self.is_fresh(),
            "age_ms": None if a is None else int(a * 1000),
            "target_ip": self.target_ip,
            "drops": self.dropped,
            "v_mps": round(d["v_mps"], 2) if d else None,
            "ax_mps2": round(d["ax_mps2"], 2) if d else None,
            "dist_m": round(d["dist_m"], 2) if d else None,
            "vbat_v": round(d["vbat_v"], 2) if d else None,
            "imu_ok": d["imu_ok"] if d else None,
        }


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--zero", metavar="TARGET_IP")
    ap.add_argument("--cal", metavar="TARGET_IP")
    a = ap.parse_args()
    rx = TargetReceiver()
    if a.zero or a.cal:
        print(rx.send_cmd("ZERO" if a.zero else "CAL", a.zero or a.cal))
    elif not rx.ok:
        print("cannot bind:", rx.error)
    else:
        print(f"listening on UDP {rx.port} ...")
        while True:
            time.sleep(0.2)
            d = rx.get()
            if not d:
                continue
            state = "OK   " if rx.is_fresh() else "STALE"
            print(f"{state} {rx.target_ip} #{d['seq']:6d} v={d['v_mps']:5.2f} m/s "
                  f"a={d['ax_mps2']:+5.2f} d={d['dist_m']:6.2f} m x={d['x_m']:5.2f} "
                  f"y={d['y_m']:5.2f} h={d['heading_deg']:6.1f} bat={d['vbat_v']:4.2f} V "
                  f"drops={rx.dropped}", flush=True)
