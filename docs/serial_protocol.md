# MiniX serial protocol

Single source of truth for the Arduino <-> Jetson contract. If you change
one side, update this file and grep both `firmware/` and `jetson/` for
the affected command/telemetry line before you call it done — the two
sides have gone out of sync twice already (differential-drive protocol
vs letter protocol; 9600 vs 115200 baud) and both times cost a debug
session.

- **Port**: `/dev/ttyUSB0` (Arduino UNO, CH340-type)
- **Baud**: `9600` — must match on both ends (`Serial.begin()` in the
  sketch, `config.SERIAL_BAUD` on the Jetson)

## Commands (Jetson -> Arduino, one raw character, no newline needed)

| Char  | Meaning                                            |
|-------|-----------------------------------------------------|
| `F`   | drive forward at current speed                      |
| `B`   | drive backward at current speed                      |
| `S`   | stop (motors off, steering holds position)           |
| `E`   | ebrake — same as `S`, sent from the Jetson AEB path  |
| `L`   | steer left (Arduino also trims the inner wheel)      |
| `R`   | steer right (same trim)                              |
| `C`   | center steering                                      |
| `0`-`9` | set base speed to that decile of full scale (0-255) |

## Telemetry (Arduino -> Jetson, ~10 Hz, newline-terminated)

| Line                | Meaning                                                        |
|----------------------|------------------------------------------------------------------|
| `U <dist_cm>`        | latest ultrasonic reading; `999` = no echo / out of range        |
| `T <l> <r> <state>`  | signed commanded PWM per side (+forward / -reverse); `state`: 0=stopped, 1=forward, 2=reverse |

## Where each side lives

- Arduino: `firmware/minix_drive/minix_drive.ino` — the `handleCommand()`
  switch and `sendTelemetry()` are the two functions that define this
  contract on the firmware side.
- Jetson: `jetson/serial_link.py` — `_reader()` parses telemetry,
  `forward()`/`reverse()`/`stop()`/`ebrake()`/`steer_left()`/`steer_right()`/
  `steer_center()`/`set_speed()` send commands. `jetson/app.py`'s `cmd()`
  route is the only caller of those methods.
