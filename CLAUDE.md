# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

MiniX is a 1:10-scale self-driving-car demonstrator: a Jetson Orin Nano
Super (perception, web UI, ADAS decisions) paired with an Arduino UNO
(real-time drive/steer + ultrasonic). It is not a general app — it's
split across two physically separate boards that talk over a serial
link, and that hardware boundary drives almost every structural
decision in the repo.

## Repo layout

```
minix/
├── firmware/minix_drive/minix_drive.ino   # Arduino sketch (drive/steer/ultrasonic/telemetry)
├── jetson/
│   ├── app.py                             # Flask routes + the ADAS decision loop thread
│   ├── config.py                          # ports, pins-as-seen-from-Jetson, tunables
│   ├── serial_link.py                     # Arduino command/telemetry link
│   ├── camera.py                          # GStreamer/OpenCV capture + MJPEG + detection mode
│   ├── oled_display.py                    # optional SSD1306 status display on the Jetson's I2C
│   ├── adas/
│   │   ├── aeb.py                         # autonomous emergency braking decision layer
│   │   └── ldw.py                         # lane departure warning (classical CV, no NN)
│   ├── templates/index.html               # the live UI (Flask looks here, not jetson/index.html)
│   ├── static/{app.js,style.css}
│   └── requirements.txt
├── scripts/install.sh                     # venv + deps + systemd install, one shot
├── systemd/minix.service                  # (see "stale/duplicate files" below)
└── docs/serial_protocol.md                # the Arduino<->Jetson contract — read before touching either side
```

## Stale/duplicate files — don't edit the wrong one

Several files exist in two copies at different points in the project's
history. Always check which one is actually wired in before editing:

- **`jetson/app.py`** is current. **`jetson/app__.py`** is an older
  snapshot (no LKA, no detect-mode blocking) — leftover, not imported
  by anything. Don't "fix" it; prefer deleting it if you're touching
  this area.
- **`jetson/templates/index.html`** is what Flask actually serves
  (`render_template` resolves from `templates/`). **`jetson/index.html`**
  is a stale copy missing the Object Detection toggle button — editing
  it has no effect on the running UI.
- **`jetson/camera.py`** is what `app.py` imports (`from camera import
  Camera`). **`jetson/adas/camera.py`** is a byte-identical orphan
  despite living next to `aeb.py`/`ldw.py` — it is not imported
  anywhere. Edit `jetson/camera.py`.
- **`jetson/minix.service`** (copied by `scripts/install.sh`) and
  **`systemd/minix.service`** (top-level) have diverged: the paths in
  `jetson/minix.service` point at a different venv than
  `jetson/app.py`'s own sibling `venv/`. If you change the systemd
  unit, check `install.sh` to see which file path it actually copies
  and update that one, then reconcile the other.

## The serial protocol is the real API boundary

`docs/serial_protocol.md` is the contract between the two boards. The
two sides have drifted out of sync before (differential-drive protocol
→ letter protocol; 9600 vs 115200 baud) and it cost real debug time.
Whenever you touch `handleCommand()` / `handleJoystick()` /
`handleSteerOnly()` / `sendTelemetry()` in the `.ino`, or the
corresponding methods in `jetson/serial_link.py`, update
`docs/serial_protocol.md` in the same change.

Note the doc is currently incomplete relative to the firmware: the
`.ino` and `serial_link.py` both implement `'J <steer> <throttle>\n'`
(joystick, combined steer+speed) and `'A <steer>\n'` (steer-only, used
by LKA) in addition to the single-char commands the doc lists — check
the `.ino`'s header comment and `serial_link.py`'s docstring for the
full, current protocol rather than trusting the doc alone.

Single-character commands (`F`/`B`/`S`/`E`/`L`/`R`/`C`/`0`-`9`) need no
newline; `J`/`A` are newline-terminated lines. Telemetry lines (`U
<dist_cm>`, `T <l> <r> <state>`) are pushed ~10 Hz from the Arduino and
consumed on a background thread in `SerialLink._reader()` — nothing in
the Flask app ever blocks on the port.

## Jetson-side architecture

- **`app.py`** owns a single background thread (`adas_loop`) that
  polls `link.snapshot()` at 20 Hz and makes the AEB/LKA decisions;
  Flask routes themselves never touch ADAS state directly except to
  read it for `/status`. If you add a new ADAS behavior, wire it into
  this loop, not into a route handler.
- **`adas/aeb.py`** and **`adas/ldw.py`** are independent decision
  layers — both read camera/serial state but don't depend on each
  other. New ADAS functions (LKA-style corrections, object-detection
  braking) should be added as siblings here, following the same
  pattern: a small class with `.update()`/`.process()` and `.status()`,
  constructed once in `app.py` and polled from `adas_loop`.
- **LKA** (lane-keeping) lives inline in `app.py`'s `adas_loop`/`cmd()`
  rather than as its own `adas/` module — it's a thin consumer of
  `ldw`'s output. It only fires when driving forward, lanes are
  visible and departing, and the driver hasn't steered manually within
  `LKA_DEFER_SECONDS`; manual steering always wins immediately, and LKA
  uses `serial_link.set_steer_angle()` (the `A` command) specifically
  because that command *cannot* change throttle/drive state.
- **Object detection mode** is a mutually-exclusive alternate
  processing path on `camera.py`'s *same* capture loop and *same*
  `/video` stream as LDW — not a second stream/route. The YOLO/
  `ultralytics` import is lazy, deferred to `set_detect_mode(True)`,
  so a missing/incompatible detection venv only breaks detection mode,
  never normal driving. While detect mode is on, `app.py`'s `/cmd`
  route blocks every action in `DRIVE_ACTIONS` except stop/ebrake.
- **Fail-safe-by-construction** is the house style: `SerialLink`,
  `Camera`, and `OLEDStatus` all swallow their own connection/hardware
  errors internally and expose an `.ok`/`link_ok` flag instead of
  raising, so a disconnected Arduino, missing camera, or unwired OLED
  degrades that one feature without taking down the Flask process.
  Match this pattern for new hardware-adjacent code.

## Arduino-side architecture (`firmware/minix_drive/minix_drive.ino`)

- No `Servo.h`: attaching a Servo object on an Uno claims Timer1 and
  silently kills `analogWrite()` PWM on D9/D10, which this firmware
  needs for the second motor driver. Steering is pulsed manually in
  `updateServo()` instead, with interrupts disabled for the ~1-2ms
  pulse window to avoid UART-RX-induced jitter.
- **MRM (Minimum Risk Maneuver) watchdog** (`checkWatchdog()`): if no
  drive command arrives for `WATCHDOG_TIMEOUT_MS` (500ms) the Arduino
  force-stops itself, independent of Jetson health. This is the one
  safety behavior that doesn't rely on the Jetson being alive.
  Steer-only (`A`) commands deliberately do *not* reset this watchdog.
- There's also a local obstacle cutoff (`MIN_SAFE_DISTANCE_CM`,
  default 15cm) independent of the Jetson's AEB thresholds in
  `config.py` — two separate safety layers at two different distances.
- `STEER_LEFT`/`STEER_RIGHT`/`STEER_CENTER` are physical-linkage
  calibration values, not logical constants — re-measure before
  changing them (see the calibration-history comments in the file for
  why that matters: a past miscalibration caused continuous servo
  stall/battery drain).

## Running / iterating

```bash
./scripts/install.sh                    # first-time Jetson setup: venv, deps, systemd unit
sudo systemctl start minix              # or reboot — enabled at boot
journalctl -u minix -f                  # watch logs
```

While iterating, stop the service and run directly so you can see
stdout/exceptions immediately:

```bash
sudo systemctl stop minix
cd jetson && python3 app.py             # http://<jetson-ip>:5000
```

Flashing the Arduino:

```bash
arduino-cli compile --upload -p /dev/ttyUSB0 -b arduino:avr:uno firmware/minix_drive
```

The Jetson service can be running or stopped during a flash either
way — `serial_link.py` reconnects automatically, and the UNO resets
cleanly when the port reopens.

There is no test suite, linter, or CI in this repo. Verification is
manual/on-hardware (drive the car, watch `/status` and `/debug`).

## Debugging

`/debug` (defined inline in `app.py` as `DEBUG_HTML`, not a template)
is a raw serial console: send single characters directly to the
Arduino and watch the RX/TX/ERR log that `SerialLink` keeps in a
rolling `deque` (`log_snapshot()` / `/debug/serial`). Use this to
isolate whether a problem is in the serial link itself vs. the
Flask/ADAS logic above it.

## Config

Almost every tunable (serial port, camera device/resolution, default
speed, AEB warn/brake distances in cm, LDW offset threshold) lives in
`jetson/config.py` — check there before hunting through the rest of
the codebase for a magic number. Physical/firmware calibration values
(steering angles, motor pins, watchdog timeout) live in the `.ino`
instead, since they're meaningless without the Arduino side.

## Hardware & safety

Arduino is on /dev/ttyUSB0 (CH340; driver was compiled from source for the L4T kernel — don’t touch it).
Python env for detection is ~/minix-detect, pinned: torch 2.8.0 (jetson-ai-lab wheel), numpy 1.26.4, opencv-python 4.10.0, TensorRT 10.3 from JetPack symlinked in. Never upgrade or pip install into it without asking.
Camera (Brio 100, /dev/video0) is mounted upside down — frames need 180° rotation. Only one process can hold /dev/video0; stop the minix service before running detection standalone.
Before any motor or flashing test: ask me to confirm wheels are off the ground. Never flash the Arduino or restart the service without asking.
When the vehicle misbehaves physically (steering, tracking, motors dead), suspect wiring, ground, or linkage before code — that’s been the root cause most times on this build.
The servo uses Timer1, so pins 9/10 lose PWM — check timer conflicts before reassigning pins.


