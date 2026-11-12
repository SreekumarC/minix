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

# --- server ---
HOST = "0.0.0.0"
PORT = 5000

CAM_ROTATE_180 = True
