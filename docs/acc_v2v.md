# ACC with camera / ultrasonic and V2V fallback

MiniX (ego) follows a target vehicle (mecanum RC car, driven by its own
stock remote). ACC is switched on/off from the web UI.

```
target RC car ──ESP32 (Hall ×2 + MPU-6050)──UDP 5005, 50 Hz──▶ Jetson v2v.py ─┐
                                                                               ├─▶ adas/acc.py ─▶ serial F / S / 0-9 ─▶ Arduino
MiniX HC-SR04 ──Arduino "U <cm>"──▶ serial_link.py ────────────────────────────┤
MiniX Brio 100 ──camera.py (quality metrics) + ldw.py (lanes seen) ────────────┘
```

## Files

| File | Role |
|------|------|
| `jetson/adas/acc.py` | ACC decision layer: mode selection, gap control, rate limits. Pure logic. |
| `jetson/adas/target_detect.py` | Camera detection of the target car (classical CV, runs every frame next to LDW). Learn/forget from the UI; model saved to `jetson/target_model.json` (not in git). |
| `jetson/v2v.py` | UDP receiver for target telemetry. Fail-safe: `.ok` False if the port can't be bound. Also a CLI (`python3 v2v.py`). |
| `jetson/app.py` | `acc_step()` in `adas_loop`, `acc_on`/`acc_off`/`v2v_zero` actions in `/cmd`, `acc`/`v2v`/`cam_quality` in `/status`. |
| `jetson/camera.py` | Adds `last_frame_ts` and `image_quality` (brightness, contrast, sharpness) per frame. |
| `jetson/serial_link.py` | Adds `dist_ts` (time of each `U` line) so ACC can compute range rate. No protocol change. |
| `jetson/config.py` | All `ACC_*` and `V2V_*` tunables. |
| `firmware/ACC_Target_Telemetry/ACC_Target_Telemetry.ino` | ESP32 sketch for the target vehicle. |
| `docs/wiring/ACC_Target_Wiring_RevC.*` | Target wiring as built (ESP32 DevKit 30-pin, LM2596), pins in board order. Rev B = earlier logical version. |

## Modes

| Mode | When | Gap from | Lead speed from |
|------|------|----------|-----------------|
| **CAMERA** (default) | camera healthy **and** road or target car readable | ultrasonic; camera estimate when no echo (nearer of the two when both) | range rate + integrator |
| **V2V** | camera bad for `ACC_TO_V2V_S` (0.5 s) and target packets fresh | ultrasonic (if echo) | target telemetry (speed + accel feed-forward) |
| **FAULT** | camera bad and no fresh V2V, or Arduino link down | — | ACC disengages, car stops |

Back from V2V to CAMERA after the camera has been healthy for
`ACC_TO_CAMERA_S` (2 s). If V2V drops while the camera is usable, ACC
goes straight back to CAMERA.

**Camera healthy** = camera open, frame < 0.5 s old, fps ≥ `ACC_CAM_MIN_FPS`,
mean brightness within `ACC_CAM_MIN/MAX_BRIGHT`, contrast ≥
`ACC_CAM_MIN_CONTRAST`, sharpness (Laplacian variance) ≥
`ACC_CAM_MIN_SHARPNESS`, and (if `ACC_CAM_REQUIRE_LANES`) LDW found lanes
**or saw the learned target car** in ≥ `ACC_CAM_LANE_RATIO` of the last
`ACC_CAM_LANE_WINDOW_S` seconds.
Live values are in `/status` → `cam_quality`, so thresholds can be tuned
at the venue. With no lane markings, learning the target car is enough for camera mode.
To skip the scene check entirely (bench tests), set
`ACC_CAM_REQUIRE_LANES = False`.

### Target car detection (`adas/target_detect.py`)

The black mecanum car on the light floor is found as a compact dark
region in the lower part of the image, filtered by size, shape and
solidity; regions cut off by one side of the frame (desks, walls) are
ignored. It works roughly before learning; **Learn target car** (web UI)
makes it specific: put the car straight ahead of MiniX at 40–150 cm and
press it. That stores the car's grey-level histogram, shape and darkness,
and — using the ultrasonic reading at that moment — a width × distance
constant so the camera can estimate distance from the box width alone.
Re-learn when the lighting or venue changes. The video shows a green
**TARGET** box with score and estimated distance.

Object detection (YOLO) is still a separate mode that blocks driving,
so it is not used for lead-vehicle confirmation here; turning it on
disengages ACC.

## Control

- Desired gap `ACC_GAP_CM` (80 cm). PWM request =
  feed-forward + `ACC_KP`·gap error + `ACC_KD`·range rate.
  Feed-forward is an integrator in CAMERA mode, and
  `v_lead·ACC_PWM_PER_MPS + ACC_KA·a_lead` in V2V mode.
- Stops if the gap predicted `ACC_STOP_LOOKAHEAD_S` ahead is below
  `ACC_STOP_GAP_CM` (60 cm), or (V2V) the lead has stopped; after a stop it
  waits until the gap reopens to `ACC_RESTART_GAP_CM` (75 cm), so it doesn't
  creep at the stop line behind a lead slower than MiniX's minimum speed. Ramp limits give smooth starts and firmer stops.
- No lead within `ACC_DETECT_CM` → cruise at the speed slider value. In
  V2V mode with the lead beyond ultrasonic range, speed is capped at the
  lead's speed + `ACC_V2V_BLIND_MARGIN_MPS`.
- Output is quantised to the Arduino's `0`-`9` speed deciles (~28 PWM a
  step) and resent with `F` every `ACC_CMD_PERIOD_S`. Below
  `ACC_PWM_MIN_MOVE` it sends `S`. The decile step is the main limit on
  smoothness; a fine-grained speed command in the firmware would be the
  next improvement (needs a protocol change + reflash).

**Calibrate `ACC_PWM_PER_MPS`** before relying on V2V mode: drive at a
fixed slider value, time it over a known distance, PWM ÷ m/s.

## Ego IMU (`jetson/imu.py`, optional)

ISM330DHCX on the Jetson 40-pin header: VCC pin 17 (3V3), GND pin 25,
SDA pin 27, SCL pin 28 -> I2C bus 1, address 0x6B. Mounted flat (Z up) next
to the Arduino. Vehicle forward was measured with a nose-up tilt test
(change X +0.019 g, Y -0.152 g) -> `IMU_FWD_DEG = -83` (board -Y, 7 deg
skew); `imu.py` rotates into vehicle axes (a_long, a_lat, yaw + = left).
If the board is ever remounted, repeat the tilt test and update that value.

- Gravity and gyro bias are learned whenever the Arduino reports the
  motors stopped (~1 s of standstill after boot before `calibrated`).
- Ego speed = integrated a_long, reset to 0 at every stop; trusted only
  `IMU_SPEED_MAX_S` after a stop (no wheel encoder, so it drifts).
- ACC uses it for a curve speed cap (`ACC_CURVE_YAW_DPS`,
  `ACC_CURVE_SPEED_FRAC`) and, in V2V mode, speed matching
  (`ACC_KV_PWM_PER_MPS` x (lead speed - ego speed)).
- Missing/unwired IMU: `.ok` False, UI shows "IMU not found", ACC behaves
  exactly as before.

Standalone check (service stopped): `~/minix-detect/bin/python jetson/imu.py`.

## Driver interaction (web UI)

- **ACC: On** engages; the speed slider is the set speed.
- Stop, E-brake, Reverse, or pulling the joystick back → ACC off.
- Forward is ignored while ACC drives. Steering (buttons or joystick
  sideways) stays manual and is sent steer-only (`A`), so it never
  touches the throttle. LKA works as before.
- AEB still runs independently; an AEB brake also switches ACC off.
- If no browser polls `/status` for `ACC_UI_HEARTBEAT_S` (1 s), ACC
  disengages and stops — ACC resends `F` itself, so without this the
  Arduino watchdog would stay fed with nobody watching.
- **Zero V2V** resets the target's distance/x/y/heading (UDP `ZERO`).
- **Learn target car / Forget**: see Target car detection above.

## V2V packet (target → Jetson, UDP 5005, 50 Hz, 56 bytes, little-endian)

Python: `struct.unpack('<HBBII11f', data)`

| Field | Type | Notes |
|-------|------|-------|
| magic | u16 | `0xAC01` |
| version | u8 | 1 |
| flags | u8 | bit0 IMU ok, bit1 moving, bit2 OLED ok |
| seq | u32 | resets on target reboot |
| t_ms | u32 | target `millis()` |
| v_mps | f32 | fused speed (≥ 0; single Hall has no direction) |
| ax_mps2, ay_mps2 | f32 | filtered, bias removed |
| yaw_rate_dps, heading_deg | f32 | CCW + |
| dist_m, x_m, y_m | f32 | since last `ZERO` |
| vL_mps, vR_mps | f32 | raw wheel speeds |
| vbat_v | f32 | target battery |

Commands (Jetson → target, UDP 5006, text): `ZERO`, `CAL` (only when the
target is still). If you change the struct, change it in the `.ino`, in
`jetson/v2v.py` (`FMT`/`FIELDS`) and here together.

## Bring-up

1. Flash `firmware/ACC_Target_Telemetry` to the target ESP32 (Arduino IDE,
   board *ESP32 Dev Module*, library *Adafruit SSD1306*). Set `WIFI_SSID` /
   `WIFI_PASS` to the network the Jetson is on (e.g. the phone hotspot),
   `WHEEL_DIAMETER_M` and `MAGNETS_PER_REV`. Keep the target still for ~1 s
   at power-on (IMU calibration).
2. On the Jetson, with the minix service stopped: `cd jetson && python3 v2v.py`
   — packets should show `OK`. Some hotspots block broadcast; if so set
   `EGO_IP` in the sketch to the Jetson's IP.
3. Start the app, open the UI: the **Target** telltale shows live speed;
   the **ACC** telltale shows `cam ok` or why the camera is rejected.
4. Wheels off the ground first, then a slow floor test with the target
   pushed by hand.
