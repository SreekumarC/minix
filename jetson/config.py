"""MiniX tuning constants. Everything you might change at the demo venue."""

# --- serial link to Arduino UNO ---
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 9600

# --- camera ---
CAM_DEVICE = "/dev/video0"
CAM_WIDTH = 640
CAM_HEIGHT = 360
CAM_FPS = 30
JPEG_QUALITY = 70

# --- drive ---
DEFAULT_SPEED = 150      # PWM, 0-255. Below ~60 the motors stall.
TURN_RATIO = 0.55        # inner wheel fraction during a turn
TRIM_L = 1.10            # per-side trim for straight-line drift
TRIM_R = 1.00

# --- AEB thresholds (cm) ---
AEB_WARN_CM = 100         # amber telltale
AEB_BRAKE_CM = 50        # Jetson commands emergency brake
# Arduino has its own hard failsafe at 15 cm, independent of this.

# --- LDW ---
LDW_ENABLED = True
LDW_OFFSET_WARN = 0.18   # lane-centre offset as fraction of frame width

# --- ACC (adaptive cruise control) — see adas/acc.py ---
# Gap / speed (the Arduino speed command is a 0-9 decile, ~28 PWM per step)
ACC_GAP_CM = 80            # desired following gap; keep it above AEB_BRAKE_CM
ACC_STOP_GAP_CM = 60       # closer than this and not opening -> ACC stops the car
ACC_STOP_LOOKAHEAD_S = 0.6 # stop early on the gap predicted this far ahead (motor lag)
ACC_RESTART_GAP_CM = 75    # after a stop, move again only once the gap reopens to this
ACC_DETECT_CM = 250        # ultrasonic beyond this = no lead in range -> cruise
ACC_KP = 0.8               # PWM per cm of gap error
ACC_KI = 0.4               # PWM/s per cm of gap error (camera mode integrator)
ACC_KD = 0.6               # PWM per cm/s of range rate (+ opening, - closing)
ACC_KA = 20.0              # PWM per m/s^2 of lead accel (V2V feed-forward)
ACC_PWM_PER_MPS = 150.0    # CALIBRATE: PWM that gives MiniX ~1 m/s on the floor
ACC_PWM_MIN_MOVE = 70      # below this the motors stall -> command stop instead
ACC_RAMP_UP_PWM_S = 120    # smooth acceleration
ACC_RAMP_DOWN_PWM_S = 400  # firmer deceleration
ACC_V2V_BLIND_MARGIN_MPS = 0.15  # V2V mode, lead beyond ultrasonic: max lead speed + this
ACC_CMD_PERIOD_S = 0.1     # resend speed+F (also feeds the Arduino MRM watchdog)
ACC_UI_HEARTBEAT_S = 1.0   # ACC disengages + stops if no browser polls /status this long
ACC_KV_PWM_PER_MPS = 60.0  # V2V: PWM per m/s of (lead speed - IMU ego speed)
ACC_KV_MAX_PWM = 40.0      # ... correction clamped to +- this
ACC_CURVE_YAW_DPS = 60.0   # IMU yaw rate above this = in a bend
ACC_CURVE_SPEED_FRAC = 0.6 # ... cap ACC request to this share of set speed

# Camera health — any failure for ACC_TO_V2V_S switches ACC to V2V mode
ACC_CAM_MIN_FPS = 10
ACC_CAM_MIN_BRIGHT = 35    # mean grey level 0-255
ACC_CAM_MAX_BRIGHT = 220
ACC_CAM_MIN_CONTRAST = 18  # grey std-dev
ACC_CAM_MIN_SHARPNESS = 40 # variance of Laplacian (blur / dirty lens)
ACC_CAM_REQUIRE_LANES = True   # camera must also read the scene: LDW lanes OR the
                               # learned target car (adas/target_detect.py)
ACC_CAM_LANE_RATIO = 0.4       # share of frames with lanes/target in the window
ACC_CAM_LANE_WINDOW_S = 2.0
ACC_TO_V2V_S = 0.5         # camera bad this long -> V2V
ACC_TO_CAMERA_S = 2.0      # camera good this long -> back to camera

# --- V2V link from the ACC target (firmware/ACC_Target_Telemetry) ---
V2V_PORT = 5005            # target -> Jetson telemetry (UDP)
V2V_CMD_PORT = 5006        # Jetson -> target commands (ZERO / CAL)
V2V_STALE_S = 0.3          # no packet this long = V2V unavailable

# --- server ---
HOST = "0.0.0.0"
PORT = 5000

CAM_ROTATE_180 = True

# --- Ego IMU: ISM330DHCX on the 40-pin header (jetson/imu.py) ---
IMU_ENABLED = True
IMU_I2C_BUS = 1            # pins 27 (SDA) / 28 (SCL)
IMU_I2C_ADDR = 0x6B
IMU_FWD_DEG = -83.0        # vehicle forward, deg from board +X toward +Y (tilt test 2026-11-23)
IMU_SPEED_MAX_S = 4.0      # integrated ego speed trusted this long after a stop (drifts ~0.1 m/s/s)
