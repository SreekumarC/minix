# MiniX

1:10-scale SDV demonstrator — Jetson Orin Nano Super (perception, web UI,
ADAS decisions) + Arduino UNO (real-time drive/steer + ultrasonic).

## Structure

```
minix/
├── firmware/
│   └── minix_drive/
│       └── minix_drive.ino      # Arduino sketch: drive, steering, ultrasonic, telemetry
├── jetson/
│   ├── app.py                   # Flask routes: /, /video, /status, /cmd
│   ├── config.py                # ports, pins-as-seen-from-Jetson, tunables
│   ├── serial_link.py           # Arduino command/telemetry link
│   ├── camera.py                # GStreamer/OpenCV capture + MJPEG stream
│   ├── adas/
│   │   ├── aeb.py               # Jetson-side AEB decision layer
│   │   └── ldw.py               # lane departure warning
│   ├── templates/
│   │   └── index.html           # browser UI: video + drive buttons + status
│   ├── static/                  # css/js for the UI, if any
│   ├── requirements.txt
│   └── minix.service            # systemd unit (see scripts/install.sh)
├── scripts/
│   └── install.sh               # venv + deps + systemd install, one shot
└── docs/
    └── serial_protocol.md       # the Arduino<->Jetson contract — read this
                                  # before changing either side
```

## Why this split

- **firmware/** vs **jetson/** mirrors the actual hardware boundary — one
  is flashed with the Arduino IDE/CLI, the other runs as a Python
  service. Keeping them as siblings (not nested) makes that obvious and
  keeps `.ino` toolchain quirks (e.g. Arduino requiring the sketch file
  share its folder's name) out of the Python package.
- **docs/serial_protocol.md** exists because the two sides have drifted
  out of sync before (differential-drive protocol → letter protocol,
  baud rate mismatch). Treat it as the contract; update it in the same
  commit as any change to `handleCommand()`/`sendTelemetry()` in the
  sketch or to `serial_link.py`.
- **adas/** is its own package because AEB and LDW are independent
  decision layers that both read `camera`/`serial_link` state but don't
  depend on each other — new ADAS functions (LKA, object-detection
  braking) should be added as siblings here, not folded into `app.py`.

## First-time setup (Jetson)

```bash
./scripts/install.sh
sudo systemctl start minix     # or just reboot — it's enabled at boot
journalctl -u minix -f         # watch logs
```

## Flashing the Arduino

Open `firmware/minix_drive/minix_drive.ino` in the Arduino IDE (or
`arduino-cli compile --upload -p /dev/ttyUSB0 -b arduino:avr:uno
firmware/minix_drive`) and upload. The Jetson service can be running or
stopped either way — `serial_link.py` reconnects automatically on next
start, and the UNO resets cleanly when the port reopens.

## Day-to-day workflow

- Drive/steering/telemetry logic → edit the `.ino`, re-flash, then check
  `docs/serial_protocol.md` is still accurate.
- Web UI, AEB/LDW tuning, camera → edit under `jetson/`, then
  `sudo systemctl restart minix` (or just re-run `python3 app.py`
  directly while iterating, service stopped).
- New ADAS function → new file under `jetson/adas/`, wired into the
  decision loop in `app.py`.
