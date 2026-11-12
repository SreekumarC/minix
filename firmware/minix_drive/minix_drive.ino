/*
 * MiniX Drive Controller — two-driver version
 * ------------------------------------------------------------
 * Hardware (with the second BTS7960 added):
 *   - LEFT motor on its own BTS7960
 *       L_RPWM -> D5
 *       L_LPWM -> D6
 *   - RIGHT motor on its own BTS7960
 *       R_RPWM -> D9
 *       R_LPWM -> D10
 *       (both boards' R_EN/L_EN tied to 5V, or drive HIGH from
 *        spare digital pins in setup() if you want software enable)
 *   - MG958 steering servo -> D11
 *   - HC-SR04 ultrasonic -> TRIG D2, ECHO D3
 *
 * IMPORTANT — why this isn't using Servo.h:
 *   The Arduino Servo library on an Uno permanently claims Timer1
 *   the moment any Servo object is attach()'d, which silently
 *   disables analogWrite() (hardware PWM) on D9 and D10 for as long
 *   as the servo is attached — regardless of which pin the servo
 *   itself uses. That's exactly why D9/D10 weren't driving motors
 *   earlier: nothing wrong with the pins or the driver, Timer1 was
 *   just locked up by the servo. Now that the second BTS7960 needs
 *   D9/D10 for its PWM, the fix is to stop using Servo.h and pulse
 *   the steering servo manually instead (updateServo() below),
 *   which frees Timer1 and restores hardware PWM on D9/D10.
 *
 * Serial protocol (from Jetson Flask app, 9600 baud):
 *   'F'  drive forward at current speed
 *   'B'  drive backward at current speed
 *   'S'  stop (motors off, servo stays where it is)
 *   'E'  ebrake — same immediate stop as 'S', sent by the Jetson AEB path
 *   'L'  steer left  (also trims the inner-wheel speed for a tighter turn)
 *   'R'  steer right (same)
 *   'C'  center steering (full speed on both motors)
 *   '0'-'9'  set base speed to that decile of full scale
 *   'J <steer_norm> <throttle_norm>\n'  combined smooth control — both
 *        values -100..100, updates steering angle and speed together
 *        in one call (see handleJoystick() below)
 *   'A <steer_norm>\n'  steer-only — updates steering angle WITHOUT
 *        touching driveState/throttle. Used by LKA so lane-centering
 *        corrections never fight the Forward/Reverse commands driving
 *        the car (see handleSteerOnly() below).
 *
 * CALIBRATION (Aug 7): STEER_LEFT/STEER_RIGHT below were found swapped
 * during Trial 1 — commanding 'R' was physically turning the car left.
 * Fixed by swapping the two constant values. If you ever re-calibrate
 * the linkage, change the values here, not the L/R command mapping.
 *
 * Throttle and steering are independent — the last 'F'/'B'/'S' state
 * and the last steering angle are both held continuously, so e.g.
 * sending 'L' then 'F' turns while driving forward.
 *
 * Outbound telemetry (Arduino -> Jetson, ~10 Hz, one line each):
 *   "U <dist_cm>"        ultrasonic reading (999 = no echo / out of range)
 *   "T <l> <r> <state>"  signed per-side commanded PWM (+F/-B) and a
 *                         state code: 0=stopped (commanded), 1=forward,
 *                         2=reverse, 3=stopped by the MRM watchdog
 *
 * Basic MRM (Minimum Risk Maneuver): if no command (single-char or 'J')
 * arrives for WATCHDOG_TIMEOUT_MS while driving, the Arduino forces a
 * stop on its own — see checkWatchdog() below. This runs independent
 * of the Jetson's health, so it still protects against a Jetson crash/
 * hang or a dropped serial link, not just a released button.
 * ------------------------------------------------------------
 */

// ---- Pin map ----
//const int L_RPWM_PIN = 3;
//const int L_LPWM_PIN = 11;
//const int R_RPWM_PIN = 6;
//const int R_LPWM_PIN = 5;
//const int SERVO_PIN  = 7;
//const int TRIG_PIN   = 2;
//const int ECHO_PIN   = 3;

const int L_RPWM_PIN = 5;
const int L_LPWM_PIN = 6;
const int R_RPWM_PIN = 9;
const int R_LPWM_PIN = 10;
const int SERVO_PIN  = 11;
const int TRIG_PIN   = 2;
const int ECHO_PIN   = 3;


// ---- Steering geometry (tune these for your linkage) ----
const int STEER_CENTER = 95;
// SAFETY (Aug 11): STEER_LEFT/RIGHT were set to 165/25 — a 140-degree
// total swing. Real steering linkages rarely have that much travel;
// the servo was almost certainly stalling against a mechanical hard
// stop trying to reach an unreachable angle, which draws continuous
// high current (stalled motor) and looks/sounds like it's "rotating
// continuously." That's also the battery drain.
// Pulled back to a conservative ±20 degrees as a SAFE STARTING POINT.
// This is a guess, not a measurement — verify it: disconnect the servo
// horn from the linkage, power the Arduino alone, and send 'L'/'C'/'R'
// (Serial Monitor works for this). Confirm the servo can reach both
// STEER_LEFT and STEER_RIGHT WITHOUT buzzing/straining before ever
// reattaching the horn. Only then increase the spread gradually,
// re-testing unloaded at each step, until you find the real safe range.
const int STEER_LEFT   = 115;
const int STEER_RIGHT  = 75;
const int STEER_SPREAD = 20;   // |STEER_LEFT - CENTER| == |STEER_RIGHT - CENTER|

// ---- Inner-wheel trim while turning ----
// Scales continuously with how sharp the turn is (0 at center, full
// effect at full lock) instead of snapping to one fixed value — this
// is what makes combined steer+speed changes feel smooth rather than
// jerky. TURN_TRIM_MIN is the trim factor AT FULL LOCK (0.0-1.0).
const float TURN_TRIM_MIN = 0.55;

// ---- Manual (Timer1-free) servo pulse generator ----
unsigned long lastPulseMicros = 0;
const unsigned long PULSE_PERIOD_US = 20000; // 50 Hz refresh
int servoPulseWidthUs = 1500;                // updated whenever angle changes

void setServoAngle(int angleDeg) {
  angleDeg = constrain(angleDeg, 0, 180);
  servoPulseWidthUs = map(angleDeg, 0, 180, 1000, 2000);
}

void updateServo() {
  // Call every loop(); sends a pulse at most once per 20ms period.
  unsigned long now = micros();
  if (now - lastPulseMicros >= PULSE_PERIOD_US) {
    lastPulseMicros = now;
    // The UART RX interrupt fires on every incoming byte — real or
    // noise — regardless of what's running. If it preempts mid-pulse,
    // it stretches the HIGH duration by however long the ISR takes,
    // which the servo reads as a slightly wrong angle and corrects for
    // next cycle. That's the actual jitter mechanism (confirmed: it's
    // NOT a blocking-read stall, setTimeout() had no effect). Disabling
    // interrupts for just this ~1-2ms window makes the pulse timing
    // atomic. A byte arriving during this window can be lost, but at
    // 9600 baud (~1ms/byte) that's a rare coincidence, and a lost byte
    // just means the next repeated command (browser resends every
    // 150ms) gets through instead — an acceptable tradeoff for a
    // steady, non-jittering servo.
    noInterrupts();
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(servoPulseWidthUs);
    digitalWrite(SERVO_PIN, LOW);
    interrupts();
  }
}

// ---- Drive state ----
int steerAngle = STEER_CENTER;
char driveState = 'S';   // 'F', 'B', or 'S'
int speedPWM = 180;      // 0-255, default cruising speed (~70%)

// ---- Command-link watchdog (basic MRM) ----
// If no command arrives for WATCHDOG_TIMEOUT_MS while the car is
// driving, something upstream has failed — Jetson crash/hang, serial
// disconnect, WiFi drop taking the browser with it — and the Arduino
// has no way to know why. Rather than keep executing the last command
// forever, force a stop. This is deliberately the one safety behavior
// that does NOT depend on the Jetson being alive to trigger it.
unsigned long lastCommandMs = 0;
const unsigned long WATCHDOG_TIMEOUT_MS = 500;
bool mrmActive = false;

void checkWatchdog() {
  if (driveState == 'S') { mrmActive = false; return; }
  if (millis() - lastCommandMs > WATCHDOG_TIMEOUT_MS) {
    driveState = 'S';
    mrmActive = true;   // latched for telemetry until the next real command
  }
}

// Simple obstacle cutoff so a stray 'F' command doesn't drive into
// something right in front of the ultrasonic sensor. Set to 0 to disable.
const int MIN_SAFE_DISTANCE_CM = 15;

// pulseIn() blocks for up to 30ms when there's no echo — calling it every
// loop iteration was stretching loop() unpredictably and disturbing the
// manual servo pulse timing. Throttle it instead; the cached reading is
// reused between checks.
unsigned long lastUltrasonicMs = 0;
const unsigned long ULTRASONIC_PERIOD_MS = 60;

// ---- Telemetry (sent back to the Jetson for its own AEB loop / status page) ----
// "U <dist_cm>"        — latest ultrasonic reading (999 = no echo / out of range)
// "T <l> <r> <state>"  — signed commanded PWM per side (+forward/-reverse), state code
//                         state: 0 = stopped, 1 = forward, 2 = reverse
long lastDistanceCM = 999;
int lastLeftSigned = 0;
int lastRightSigned = 0;
int lastStateCode = 0;
unsigned long lastTelemetryMs = 0;
const unsigned long TELEMETRY_PERIOD_MS = 100;

void sendTelemetry() {
  unsigned long now = millis();
  if (now - lastTelemetryMs < TELEMETRY_PERIOD_MS) return;
  lastTelemetryMs = now;
  Serial.print("U ");
  Serial.println(lastDistanceCM);
  Serial.print("T ");
  Serial.print(lastLeftSigned);
  Serial.print(' ');
  Serial.print(lastRightSigned);
  Serial.print(' ');
  Serial.println(lastStateCode);
}

// ---- OLED status moved to the Jetson (jetson/oled_display.py) — the
// Jetson has the richer status (AEB/LDW/FPS/link health) and it keeps
// this loop's timing budget untouched. ----

long readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout (~5m)
  if (duration == 0) return -1; // no echo / out of range
  return duration * 0.0343 / 2;
}

void driveMotor(int rpwmPin, int lpwmPin, char state, int pwmVal) {
  if (state == 'F') {
    analogWrite(rpwmPin, pwmVal);
    analogWrite(lpwmPin, 0);
  } else if (state == 'B') {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, pwmVal);
  } else {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
  }
}

void applyDrive() {
  if (MIN_SAFE_DISTANCE_CM > 0) {
    unsigned long now = millis();
    if (now - lastUltrasonicMs >= ULTRASONIC_PERIOD_MS) {
      lastUltrasonicMs = now;
      long dist = readDistanceCM();
      lastDistanceCM = (dist > 0) ? dist : 999;
    }
  }

  bool blocked = (driveState == 'F' && lastDistanceCM < MIN_SAFE_DISTANCE_CM);
  char effectiveState = blocked ? 'S' : driveState;

  int leftPWM = speedPWM;
  int rightPWM = speedPWM;

  // Trim the inner wheel while steering, scaled to how sharp the turn
  // is (proportional, not a flat cut) so speed and steering blend
  // smoothly instead of snapping. Side selection matches the
  // calibrated STEER_LEFT(125)/STEER_RIGHT(55) values above: angle
  // below center = physically turning right = right wheel is inner.
  float turnFactor = constrain(abs(steerAngle - STEER_CENTER) / (float)STEER_SPREAD, 0.0, 1.0);
  float trim = 1.0 - turnFactor * (1.0 - TURN_TRIM_MIN);

  if (steerAngle < STEER_CENTER) {          // turning right -> right is inner wheel
    rightPWM = (int)(speedPWM * trim);
  } else if (steerAngle > STEER_CENTER) {   // turning left -> left is inner wheel
    leftPWM = (int)(speedPWM * trim);
  }

  driveMotor(L_RPWM_PIN, L_LPWM_PIN, effectiveState, leftPWM);
  driveMotor(R_RPWM_PIN, R_LPWM_PIN, effectiveState, rightPWM);

  int signMul = (effectiveState == 'F') ? 1 : (effectiveState == 'B') ? -1 : 0;
  lastLeftSigned = signMul * leftPWM;
  lastRightSigned = signMul * rightPWM;
  if (effectiveState == 'F') {
    lastStateCode = 1;
  } else if (effectiveState == 'B') {
    lastStateCode = 2;
  } else {
    lastStateCode = mrmActive ? 3 : 0;   // 3 = stopped by the watchdog, not by command
  }
}

void handleCommand(char c) {
  lastCommandMs = millis();
  mrmActive = false;

  switch (c) {
    case 'F': driveState = 'F'; break;
    case 'B': driveState = 'B'; break;
    case 'S': driveState = 'S'; break;
    case 'E': driveState = 'S'; break;  // ebrake: same immediate stop, kept as a distinct command for the Jetson side

    case 'L': steerAngle = STEER_LEFT; break;
    case 'R': steerAngle = STEER_RIGHT; break;
    case 'C': steerAngle = STEER_CENTER; break;

    default:
      if (c >= '0' && c <= '9') {
        int decile = c - '0';
        speedPWM = map(decile, 0, 9, 0, 255);
      }
      break;
  }
  setServoAngle(steerAngle);
  applyDrive();
}

// Combined smooth control: "J <steer_norm> <throttle_norm>\n"
//   steer_norm    -100 (full left) .. 0 (center) .. +100 (full right)
//   throttle_norm -100 (full reverse) .. 0 (stop) .. +100 (full forward)
// Both steering angle and speed are updated together in one call, so a
// joystick UI can send continuously-varying values instead of snapping
// between the discrete L/C/R and 0-9 commands above.
void handleJoystick() {
  lastCommandMs = millis();
  mrmActive = false;

  String rest = Serial.readStringUntil('\n');
  int sp = rest.indexOf(' ');
  if (sp < 0) return;

  int steerNorm = constrain(rest.substring(0, sp).toInt(), -100, 100);
  int throttleNorm = constrain(rest.substring(sp + 1).toInt(), -100, 100);

  steerAngle = map(steerNorm, -100, 100, STEER_LEFT, STEER_RIGHT);
  setServoAngle(steerAngle);

  const int DEADZONE = 3;
  if (throttleNorm > DEADZONE) {
    driveState = 'F';
    speedPWM = map(throttleNorm, 0, 100, 0, 255);
  } else if (throttleNorm < -DEADZONE) {
    driveState = 'B';
    speedPWM = map(-throttleNorm, 0, 100, 0, 255);
  } else {
    driveState = 'S';
    speedPWM = 0;
  }

  applyDrive();
}

// Steer-only: "A <steer_norm>\n" — used by LKA. Updates steerAngle and
// re-runs applyDrive() to recompute PWM for the (unchanged) driveState,
// but never sets driveState/speedPWM itself — a lane-centering nudge
// must never be able to start, stop, or change the car's throttle.
void handleSteerOnly() {
  lastCommandMs = millis();  // proves the Jetson is alive, same as any other command
  mrmActive = false;

  String rest = Serial.readStringUntil('\n');
  int steerNorm = constrain(rest.toInt(), -100, 100);
  steerAngle = map(steerNorm, -100, 100, STEER_LEFT, STEER_RIGHT);
  setServoAngle(steerAngle);
  applyDrive();
}

// Steering-only command for LKA (Lane Keeping Assist): "K <steer_norm>\n"
//   steer_norm -100 (full left) .. 0 (center) .. +100 (full right)
// Deliberately does NOT touch driveState/speedPWM (LKA must never alter
// throttle) and does NOT reset lastCommandMs/mrmActive (a steering-only
// nudge must never be able to mask the MRM watchdog — that still
// requires a genuine F/B/S/E/J command to stay satisfied).
void setup() {
  Serial.begin(9600);
  // Default Serial timeout is 1000ms, which readStringUntil() in
  // handleJoystick()/handleSteerOnly() would block for if noise on the
  // line produces a byte matching 'J'/'A' with no real newline behind
  // it. During that stall, updateServo() doesn't run and the servo
  // stops getting refreshed — capping this to 20ms bounds the damage
  // to something imperceptible instead of up to a full second.
  Serial.setTimeout(20);

  pinMode(L_RPWM_PIN, OUTPUT);
  pinMode(L_LPWM_PIN, OUTPUT);
  pinMode(R_RPWM_PIN, OUTPUT);
  pinMode(R_LPWM_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  setServoAngle(STEER_CENTER);

  analogWrite(L_RPWM_PIN, 0);
  analogWrite(L_LPWM_PIN, 0);
  analogWrite(R_RPWM_PIN, 0);
  analogWrite(R_LPWM_PIN, 0);

  Serial.println("MiniX drive ready: F/B/S drive, L/R/C steer, 0-9 speed");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'J') {
      handleJoystick();
    } else if (c == 'A') {
      handleSteerOnly();
    } else if (c != '\n' && c != '\r') {
      handleCommand(c);
    }
  }

  updateServo();
  checkWatchdog();

  // Re-check the safety cutoff periodically even with no new serial
  // command, so a forward command doesn't keep driving into an
  // obstacle that appears after the last button press.
  applyDrive();

  sendTelemetry();
}
