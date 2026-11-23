/*
  ACC_Target_Telemetry.ino  —  Rev C (wiring diagram Rev C, 18-Nov-2026)
  -----------------------------------------------------------------------------
  Measurement & telemetry unit for the ACC TARGET vehicle (mecanum RC car).
  The car is driven by its stock remote. This ESP32 only MEASURES and SENDS:

    - wheel speed / distance   : 2x A3144 Hall sensors (IO34 left, IO35 right) — optional,
                                 HAS_HALL_SENSORS=false => speed from the IMU alone
    - acceleration / yaw rate  : MPU-6050 on I2C (IO21 SDA, IO22 SCL)
    - fused speed, heading, x/y position (dead reckoning)
    - battery voltage          : divider 100k/33k on IO36

  Telemetry -> Ego (MiniX Jetson) as a 56-byte binary UDP packet at 50 Hz.
  Ego -> target commands (UDP text on CMD_PORT):  "ZERO"  reset distance/x/y/heading
                                                  "CAL"   re-calibrate IMU (car must be still)

  Board   : "ESP32 Dev Module" (ESP32-WROOM-32), Arduino-ESP32 core 2.x or 3.x
  Library : Adafruit SSD1306 + Adafruit GFX (Library Manager). MPU-6050 is read
            directly through registers, so no IMU library is needed.
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ============================ USER CONFIGURATION ============================
// Wi-Fi: join the same network as the MiniX Jetson (e.g. a phone hotspot).
// List one or more networks; the first that answers is used. If it is lost
// for WIFI_TRY_MS, the next one in the list is tried, and so on round-robin.
// The Jetson must be on the same hotspot the target ends up on.
struct WifiNet { const char *ssid; const char *pass; };
const WifiNet   WIFI_NETS[] = {
  { "YOUR_SSID_1", "YOUR_PASSWORD_1" },
  { "YOUR_SSID_2", "YOUR_PASSWORD_2" },
};
const uint32_t  WIFI_TRY_MS = 12000;   // give each network this long to connect

// 255.255.255.255 = broadcast (no IP setup needed). Broadcast on Wi-Fi is sent
// at the lowest rate with no retries, so it loses a few % of packets; for a
// steadier link put the Jetson's IP here instead, e.g. IPAddress(10,156,22,46).
IPAddress       EGO_IP(255, 255, 255, 255);
const uint16_t  EGO_PORT = 5005;           // Jetson listens here
const uint16_t  CMD_PORT = 5006;           // ESP32 listens here for ZERO / CAL

// What is actually fitted. Set true once the parts are wired.
const bool      HAS_HALL_SENSORS = false;  // A3144 wheel encoders on IO34/IO35
const bool      HAS_VBAT_DIVIDER = false;  // 100k/33k divider on IO36 (VP)

// Wheel encoder — MEASURE THESE on your car.
const float     WHEEL_DIAMETER_M = 0.065f; // outer diameter over the mecanum rollers
const uint8_t   MAGNETS_PER_REV  = 4;      // magnets glued on each hub
const float     DIST_PER_PULSE_M = (float)M_PI * WHEEL_DIAMETER_M / MAGNETS_PER_REV;

// IMU mounting: X must point FORWARD. Flip signs here if your board is rotated.
const float     IMU_X_SIGN  = +1.0f;       // forward accel
const float     IMU_Y_SIGN  = +1.0f;       // lateral accel (left +)
const float     IMU_GZ_SIGN = +1.0f;       // yaw rate (counter-clockwise +)

// Battery divider: V_bat = V_pin * (R1+R2)/R2. Trim VBAT_CAL against a multimeter.
const float     VBAT_DIVIDER = (100.0f + 33.0f) / 33.0f;
const float     VBAT_CAL     = 1.00f;

// Timing
const uint32_t  IMU_PERIOD_US   = 10000;   // 100 Hz sensor fusion
const uint32_t  TX_PERIOD_MS    = 20;      // 50 Hz telemetry
const uint32_t  OLED_PERIOD_MS  = 200;     // 5 Hz display
const uint32_t  VBAT_PERIOD_MS  = 500;

// Encoder filtering
const uint32_t  MIN_PULSE_US    = 1500;    // debounce: ignores pulses faster than this
const uint32_t  STOP_TIMEOUT_US = 400000;  // no pulse for 0.4 s => wheel stopped

// Fusion tuning
const float     ACC_LPF_HZ      = 10.0f;   // low-pass on accel (motor vibration)
const float     K_WHEEL         = 0.10f;   // pull of wheel speed on fused speed per 10 ms step
const float     STILL_ACC_MPS2  = 0.25f;   // |a| below this + wheels stopped => standstill
// IMU-only speed (HAS_HALL_SENSORS = false): standstill is detected from the
// absence of motor vibration. Tune STILL_VIB_MPS2 from the "vib" value on
// Serial: parked ~0.0x, driving clearly higher.
const float     STILL_VIB_MPS2  = 0.30f;   // vibration below this => maybe parked
const uint32_t  STILL_HOLD_MS   = 300;     // ... for this long => parked (speed reset to 0)
const float     V_LEAK_PER_S    = 0.05f;   // IMU-only: bleed 5 %/s of speed (bounds drift)
// ============================================================================

// Pins (wiring diagram Rev B)
const int PIN_SDA    = 21;
const int PIN_SCL    = 22;
const int PIN_HALL_L = 34;   // input-only, external 10k pull-up to 3V3
const int PIN_HALL_R = 35;   // input-only, external 10k pull-up to 3V3
const int PIN_VBAT   = 36;   // ADC1_CH0 (works while Wi-Fi is on)
const int PIN_MPUINT = 39;   // wired, not used by this sketch (polled at 100 Hz)

// MPU-6050 registers
uint8_t       MPU_ADDR       = 0x68;   // 0x69 if AD0 is high; probed at boot
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_SMPLRT_DIV = 0x19;
const uint8_t REG_CONFIG     = 0x1A;
const uint8_t REG_GYRO_CFG   = 0x1B;
const uint8_t REG_ACCEL_CFG  = 0x1C;
const uint8_t REG_ACCEL_XOUT = 0x3B;
const uint8_t REG_WHO_AM_I   = 0x75;
const float   ACC_LSB_PER_G  = 8192.0f;    // +/-4 g
const float   GYR_LSB_PER_DPS = 65.5f;     // +/-500 dps
const float   G_MPS2         = 9.80665f;

// OLED
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOk = false;

// Network
WiFiUDP udp;
bool    udpStarted = false;

// ------------------------------- Encoders ----------------------------------
struct Encoder {
  volatile uint32_t count  = 0;
  volatile uint32_t lastUs = 0;
  volatile uint32_t periodUs = 0;
};
Encoder encL, encR;
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onPulse(Encoder &e) {
  uint32_t now = micros();
  uint32_t dt  = now - e.lastUs;
  if (e.lastUs != 0 && dt < MIN_PULSE_US) return;   // debounce
  portENTER_CRITICAL_ISR(&encMux);
  e.periodUs = (e.lastUs == 0) ? 0 : dt;
  e.lastUs   = now;
  e.count++;
  portEXIT_CRITICAL_ISR(&encMux);
}
void IRAM_ATTR isrHallL() { onPulse(encL); }
void IRAM_ATTR isrHallR() { onPulse(encR); }

// Wheel speed from pulse period (good resolution at low speed). Decays toward
// zero when pulses stop, using the time since the last pulse.
float wheelSpeed(Encoder &e, uint32_t &countOut) {
  portENTER_CRITICAL(&encMux);
  uint32_t last = e.lastUs, period = e.periodUs;
  countOut = e.count;
  portEXIT_CRITICAL(&encMux);

  uint32_t since = micros() - last;
  if (last == 0 || period == 0 || since > STOP_TIMEOUT_US) return 0.0f;
  uint32_t effective = (since > period) ? since : period;
  return DIST_PER_PULSE_M / (effective * 1e-6f);
}

// --------------------------------- IMU -------------------------------------
bool   imuOk = false;
float  biasAx = 0, biasAy = 0, biasGz = 0;   // raw-count biases

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return false;
  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();
  ax = (int16_t)(b[0] << 8 | b[1]);
  ay = (int16_t)(b[2] << 8 | b[3]);
  az = (int16_t)(b[4] << 8 | b[5]);
  // b[6..7] temperature, b[8..11] gyro X/Y (unused)
  gz = (int16_t)(b[12] << 8 | b[13]);
  return true;
}

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// List every I2C device on the bus — the first thing to look at when the
// OLED (0x3C) or MPU-6050 (0x68/0x69) isn't found.
void i2cScan() {
  Serial.print("[I2C] devices found:");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cPresent(a)) { Serial.printf(" 0x%02X", a); n++; }
  }
  Serial.println(n ? "" : " none (check SDA=D21, SCL=D22, 3V3, GND)");
}

bool mpuInit() {
  if (i2cPresent(0x68))      MPU_ADDR = 0x68;
  else if (i2cPresent(0x69)) MPU_ADDR = 0x69;
  else return false;                               // not found on the bus
  Serial.printf("[IMU] MPU found at 0x%02X\n", MPU_ADDR);
  mpuWrite(REG_PWR_MGMT_1, 0x01);  // wake, PLL with X-gyro clock
  delay(50);
  mpuWrite(REG_SMPLRT_DIV, 0x09);  // 1 kHz / (1+9) = 100 Hz
  mpuWrite(REG_CONFIG,     0x03);  // DLPF ~44 Hz
  mpuWrite(REG_GYRO_CFG,   0x08);  // +/-500 dps
  mpuWrite(REG_ACCEL_CFG,  0x08);  // +/-4 g

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_WHO_AM_I);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 1);
  uint8_t who = Wire.available() ? Wire.read() : 0;
  Serial.printf("[IMU] WHO_AM_I = 0x%02X (0x68 genuine; clones may differ)\n", who);
  return true;
}

void oledMsg(const char *l1, const char *l2 = "") {
  if (!oledOk) return;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);  oled.print("ACC TARGET");
  oled.setCursor(0, 24); oled.print(l1);
  oled.setCursor(0, 40); oled.print(l2);
  oled.display();
}

// Average ~1 s of samples. The car MUST be still and level.
void calibrateImu() {
  if (!imuOk) return;
  oledMsg("Calibrating IMU", "keep car still...");
  Serial.println("[IMU] calibrating — keep the car still");
  const int N = 400;
  double sx = 0, sy = 0, sg = 0;
  int good = 0;
  for (int i = 0; i < N; i++) {
    int16_t ax, ay, az, gz;
    if (mpuReadRaw(ax, ay, az, gz)) { sx += ax; sy += ay; sg += gz; good++; }
    delay(3);
  }
  if (good > 0) {
    biasAx = sx / good;  biasAy = sy / good;  biasGz = sg / good;
  }
  Serial.printf("[IMU] bias ax=%.1f ay=%.1f gz=%.1f (raw counts, %d samples)\n",
                biasAx, biasAy, biasGz, good);
}

// ------------------------------ State --------------------------------------
float vFused = 0, axF = 0, ayF = 0, yawRateDps = 0;
float headingRad = 0, distM = 0, posX = 0, posY = 0;
float vL = 0, vR = 0, vbat = 0;
bool  moving = false;
float vib = 0;          // motor-vibration level, m/s^2
uint32_t seq = 0;

void resetOdometry() {
  distM = posX = posY = headingRad = 0;
  Serial.println("[CMD] odometry zeroed");
}

// --------------------------- Telemetry packet -------------------------------
// Little-endian, packed, 56 bytes. Python: struct.unpack('<HBBII11f', data)
struct __attribute__((packed)) TelemetryPkt {
  uint16_t magic;        // 0xAC01
  uint8_t  version;      // 1
  uint8_t  flags;        // bit0 IMU ok, bit1 moving, bit2 OLED ok, bit3 speed IMU-only
  uint32_t seq;
  uint32_t t_ms;         // target's millis() at sample time
  float    v_mps;        // fused longitudinal speed (>= 0, see note on direction)
  float    ax_mps2;      // longitudinal accel, filtered, gravity/bias removed
  float    ay_mps2;      // lateral accel, filtered
  float    yaw_rate_dps;
  float    heading_deg;  // since last ZERO, CCW +
  float    dist_m;       // travelled distance since last ZERO
  float    x_m;          // dead-reckoned position since last ZERO (x = initial forward)
  float    y_m;
  float    vL_mps;       // raw left wheel speed
  float    vR_mps;       // raw right wheel speed
  float    vbat_v;
};
static_assert(sizeof(TelemetryPkt) == 56, "packet size changed");

void sendTelemetry() {
  if (WiFi.status() != WL_CONNECTED || !udpStarted) return;
  TelemetryPkt p;
  p.magic = 0xAC01;  p.version = 1;
  p.flags = (imuOk ? 1 : 0) | (moving ? 2 : 0) | (oledOk ? 4 : 0) | (HAS_HALL_SENSORS ? 0 : 8);
  p.seq = seq++;     p.t_ms = millis();
  p.v_mps = vFused;  p.ax_mps2 = axF;  p.ay_mps2 = ayF;
  p.yaw_rate_dps = yawRateDps;
  p.heading_deg  = headingRad * 180.0f / (float)M_PI;
  p.dist_m = distM;  p.x_m = posX;   p.y_m = posY;
  p.vL_mps = vL;     p.vR_mps = vR;  p.vbat_v = vbat;
  udp.beginPacket(EGO_IP, EGO_PORT);
  udp.write((const uint8_t *)&p, sizeof(p));
  udp.endPacket();
}

void pollCommands() {
  if (!udpStarted) return;
  int n = udp.parsePacket();
  if (n <= 0) return;
  char buf[16] = {0};
  udp.read(buf, sizeof(buf) - 1);
  if (strncmp(buf, "ZERO", 4) == 0) resetOdometry();
  else if (strncmp(buf, "CAL", 3) == 0) {
    if (!moving) { calibrateImu(); vFused = 0; }
    else Serial.println("[CMD] CAL refused: car is moving");
  }
}

// ------------------------------- Wi-Fi --------------------------------------
const char *wlName(wl_status_t st) {
  switch (st) {
    case WL_IDLE_STATUS:     return "idle / connecting";
    case WL_NO_SSID_AVAIL:   return "SSID not found";
    case WL_CONNECT_FAILED:  return "connect failed (wrong password?)";
    case WL_CONNECTION_LOST: return "connection lost";
    case WL_DISCONNECTED:    return "disconnected";
    default:                 return "other";
  }
}

const int WIFI_N = sizeof(WIFI_NETS) / sizeof(WIFI_NETS[0]);
int      wifiIdx = 0;
uint32_t wifiTryStart = 0;

void wifiBegin(int idx) {
  wifiIdx = idx % WIFI_N;
  Serial.printf("[WiFi] trying \"%s\" (%d/%d)\n", WIFI_NETS[wifiIdx].ssid, wifiIdx + 1, WIFI_N);
  WiFi.disconnect(false, false);   // stop any attempt in progress first —
  delay(100);                      // avoids "sta is connecting, cannot set config"
  WiFi.begin(WIFI_NETS[wifiIdx].ssid, WIFI_NETS[wifiIdx].pass);
  wifiTryStart = millis();
}

// The driver retries the current network by itself (setAutoReconnect). If it
// hasn't connected within WIFI_TRY_MS, move on to the next network in the list.
void wifiService() {
  static uint32_t lastPrint = 0;
  static bool wasConnected = false;
  wl_status_t st = WiFi.status();
  bool connected = (st == WL_CONNECTED);
  if (connected && !wasConnected) {
    Serial.printf("[WiFi] connected to \"%s\", IP %s, RSSI %d dBm\n", WIFI_NETS[wifiIdx].ssid,
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    udp.stop();
    udpStarted = udp.begin(CMD_PORT);
  }
  if (!connected && wasConnected) {
    Serial.println("[WiFi] connection lost");
    wifiTryStart = millis();         // give the same network a fresh window first
  }
  if (!connected && millis() - lastPrint > 5000) {
    lastPrint = millis();
    Serial.printf("[WiFi] not connected: %s (SSID \"%s\")\n", wlName(st), WIFI_NETS[wifiIdx].ssid);
  }
  if (!connected && millis() - wifiTryStart > WIFI_TRY_MS) {
    udpStarted = false;
    wifiBegin(wifiIdx + 1);          // round-robin (with one network: plain retry)
  }
  wasConnected = connected;
}

// ------------------------------- Display ------------------------------------
void updateOled() {
  if (!oledOk) return;
  char s[24];
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("ACC TGT ");
  if (WiFi.status() == WL_CONNECTED) { oled.print("WiFi "); oled.print(wifiIdx + 1); }
  else oled.print("WiFi --");
  if (!imuOk) { oled.setCursor(104, 0); oled.print("!IMU"); }

  oled.setTextSize(2);
  oled.setCursor(0, 12);
  snprintf(s, sizeof(s), "%4.2f m/s", vFused);
  oled.print(s);

  oled.setTextSize(1);
  snprintf(s, sizeof(s), "a %+5.2f  d %6.2fm", axF, distM);
  oled.setCursor(0, 32); oled.print(s);
  snprintf(s, sizeof(s), "x %5.2f y %5.2f", posX, posY);
  oled.setCursor(0, 42); oled.print(s);
  if (HAS_VBAT_DIVIDER)
    snprintf(s, sizeof(s), "%4.1fV h%4.0f #%lu", vbat, headingRad * 57.2958f, (unsigned long)(seq % 100000));
  else
    snprintf(s, sizeof(s), "IMU h%4.0f #%lu", headingRad * 57.2958f, (unsigned long)(seq % 100000));
  oled.setCursor(0, 54); oled.print(s);
  oled.display();
}

// ------------------------------- Setup --------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== ACC Target Telemetry (Rev C) ===");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (oledOk) { oled.setTextColor(SSD1306_WHITE); oledMsg("Booting..."); }
  else Serial.println("[OLED] not found at 0x3C — continuing without display");

  pinMode(PIN_HALL_L, INPUT);   // external pull-ups (IO34/35 have none)
  pinMode(PIN_HALL_R, INPUT);
  pinMode(PIN_MPUINT, INPUT);
  if (HAS_HALL_SENSORS) {       // unconnected IO34/35 float and would fire spuriously
    attachInterrupt(digitalPinToInterrupt(PIN_HALL_L), isrHallL, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_HALL_R), isrHallR, FALLING);
  } else {
    Serial.println("[ENC] no Hall sensors fitted — speed from IMU only");
  }

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);

  i2cScan();
  imuOk = mpuInit();
  if (imuOk) calibrateImu();
  else { Serial.println("[IMU] MPU-6050 not found — speed from wheels only"); oledMsg("IMU NOT FOUND", "wheels only"); delay(1000); }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // lower, steadier latency
  WiFi.setAutoReconnect(true);
  wifiBegin(0);

  Serial.printf("Distance per pulse: %.4f m\n", DIST_PER_PULSE_M);
}

// -------------------------------- Loop --------------------------------------
void loop() {
  static uint32_t nextImuUs = micros();
  static uint32_t lastTx = 0, lastOled = 0, lastVbat = 0;

  wifiService();
  pollCommands();

  // ---- 100 Hz sensor fusion ----
  if ((int32_t)(micros() - nextImuUs) >= 0) {
    nextImuUs += IMU_PERIOD_US;
    // If we fell far behind (e.g. Wi-Fi reconnect), resync instead of bursting
    if ((int32_t)(micros() - nextImuUs) > (int32_t)(3 * IMU_PERIOD_US)) nextImuUs = micros() + IMU_PERIOD_US;
    const float dt = IMU_PERIOD_US * 1e-6f;

    // Wheel speeds
    uint32_t cL, cR;
    vL = wheelSpeed(encL, cL);
    vR = wheelSpeed(encR, cR);
    float vWheel = 0.5f * (vL + vR);
    // If one encoder is missing/faulty, fall back to the other
    if (vL == 0 && vR > 0) vWheel = vR;
    if (vR == 0 && vL > 0) vWheel = vL;

    // IMU
    float axRaw = 0, ayRaw = 0, gzDps = 0;
    if (imuOk) {
      int16_t ax, ay, az, gz;
      if (mpuReadRaw(ax, ay, az, gz)) {
        axRaw = IMU_X_SIGN  * (ax - biasAx) / ACC_LSB_PER_G * G_MPS2;
        ayRaw = IMU_Y_SIGN  * (ay - biasAy) / ACC_LSB_PER_G * G_MPS2;
        gzDps = IMU_GZ_SIGN * (gz - biasGz) / GYR_LSB_PER_DPS;
      }
    }
    const float rc = 1.0f / (2.0f * (float)M_PI * ACC_LPF_HZ);
    const float alpha = dt / (rc + dt);
    axF += alpha * (axRaw - axF);
    ayF += alpha * (ayRaw - ayF);
    yawRateDps = gzDps;

    // Vibration level: high-frequency part of the accel (motors running)
    vib += 0.05f * ((fabsf(axRaw - axF) + fabsf(ayRaw - ayF)) - vib);

    // Standstill detection (zero-velocity update + gyro bias tracking)
    bool still;
    if (HAS_HALL_SENSORS) {
      bool wheelsStopped = (vL == 0 && vR == 0);
      still = wheelsStopped && fabsf(axF) < STILL_ACC_MPS2 && fabsf(gzDps) < 2.0f;
    } else {
      // no wheel data: parked = quiet (no motor vibration), level accel, not turning
      static uint32_t quietSinceMs = 0;
      // (no |ax| test here: a small accel offset after a tilt/temperature
      //  change would otherwise keep the car "moving" forever while parked)
      bool quiet = imuOk && vib < STILL_VIB_MPS2 && fabsf(gzDps) < 2.0f;
      if (!quiet) quietSinceMs = 0;
      else if (!quietSinceMs) quietSinceMs = millis() | 1;
      still = !imuOk || (quietSinceMs && millis() - quietSinceMs >= STILL_HOLD_MS);
    }
    moving = !still;

    if (still) {
      vFused = 0;
      if (imuOk) {   // slowly track gyro drift while parked
        int16_t ax, ay, az, gz;
        if (mpuReadRaw(ax, ay, az, gz)) {
          biasGz += 0.002f * (gz - biasGz);
          biasAx += 0.002f * (ax - biasAx);   // re-level: absorbs tilt / temp drift
          biasAy += 0.002f * (ay - biasAy);
        }
      }
      yawRateDps = 0;
    } else {
      // Complementary filter: IMU gives fast response, wheels remove drift
      vFused += (imuOk ? axF : 0.0f) * dt;
      if (HAS_HALL_SENSORS) vFused += K_WHEEL * (vWheel - vFused);
      else                  vFused -= V_LEAK_PER_S * vFused * dt;
      if (vFused < 0) vFused = 0;   // single-channel Hall has no direction info
    }

    // Dead reckoning
    headingRad += yawRateDps * (float)M_PI / 180.0f * dt;
    distM += vFused * dt;
    posX  += vFused * cosf(headingRad) * dt;
    posY  += vFused * sinf(headingRad) * dt;
  }

  uint32_t nowMs = millis();

  if (nowMs - lastTx >= TX_PERIOD_MS) {
    lastTx = nowMs;
    sendTelemetry();
  }

  if (nowMs - lastVbat >= VBAT_PERIOD_MS) {
    lastVbat = nowMs;
    if (HAS_VBAT_DIVIDER) {
      float vPin = analogReadMilliVolts(PIN_VBAT) / 1000.0f;
      vbat = vPin * VBAT_DIVIDER * VBAT_CAL;
    } else {
      vbat = 0;                 // VP floating — don't show a made-up voltage
    }
  }

  static uint32_t lastDbg = 0;
  if (nowMs - lastDbg >= 1000) {   // 1 Hz Serial line for bench checks / tuning
    lastDbg = nowMs;
    Serial.printf("v %.2f m/s  ax %+.2f  ay %+.2f  yaw %+.1f dps  vib %.2f  %s\n",
                  vFused, axF, ayF, yawRateDps, vib, moving ? "MOVING" : "parked");
  }

  if (nowMs - lastOled >= OLED_PERIOD_MS) {
    lastOled = nowMs;
    updateOled();   // ~25 ms at 400 kHz; runs between fusion steps
  }
}
