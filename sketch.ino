/*
  Wearable Fall Detector - Refined Wokwi Validation Firmware
  Board: ESP32-S3 DevKitC-1

  Current simulation:
  - Automatic fall detection: acceleration -> gyro rotation -> posture
  - 10 s cancel period
  - GSM alert mocked with Serial + blue LED
  - Wi-Fi failover uses Wokwi-GUEST
  - Green button ONLY: hold >= 3 s to show a demo Wi-Fi configuration portal
  - Deep sleep, MPU6050 interrupt wake-up, and real SIM800L are future hardware work

  IMPORTANT:
  Detection thresholds below are simulation/development values only.
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// -------------------- Pin map --------------------
constexpr uint8_t I2C_SDA_PIN = 8;
constexpr uint8_t I2C_SCL_PIN = 9;
constexpr uint8_t MPU_INT_PIN = 4;       // Reserved for future interrupt wake-up
constexpr uint8_t CONFIG_BUTTON_PIN = 1; // Green
constexpr uint8_t CANCEL_BUTTON_PIN = 14;
constexpr uint8_t RED_LED_PIN = 13;
constexpr uint8_t GSM_LED_PIN = 21;      // GSM mock indicator
constexpr uint8_t BUZZER_PIN = 12;
constexpr uint8_t SYSTEM_SWITCH_PIN = 10;

// -------------------- Development thresholds --------------------
constexpr float G = 9.80665f;
constexpr float IMPACT_THRESHOLD_G = 1.50f;
constexpr float GYRO_THRESHOLD_DPS = 150.0f;
constexpr float HORIZONTAL_Z_LIMIT_G = 0.65f;
constexpr float POSTURE_MIN_G = 0.70f;
constexpr float POSTURE_MAX_G = 1.30f;

// -------------------- Timing --------------------
constexpr unsigned long SENSOR_INTERVAL_MS = 50;
constexpr unsigned long GYRO_SETTLE_MS = 50;
constexpr unsigned long GYRO_TIMEOUT_MS = 1500;
constexpr unsigned long POSTURE_SETTLE_MS = 400;
constexpr unsigned long POSTURE_TIMEOUT_MS = 3500;
constexpr uint8_t REQUIRED_HORIZONTAL_SAMPLES = 5;

constexpr unsigned long GRACE_PERIOD_MS = 10000;
constexpr unsigned long LED_BLINK_MS = 250;
constexpr unsigned long GSM_TIMEOUT_MS = 5000;
constexpr unsigned long WIFI_TIMEOUT_MS = 12000;
constexpr unsigned long RESULT_TIME_MS = 5000;
constexpr unsigned long BUTTON_DEBOUNCE_MS = 30;
constexpr unsigned long CONFIG_HOLD_MS = 3000;
constexpr unsigned long CONFIG_DEMO_TIME_MS = 7000;

// false = demonstrate GSM failure -> Wi-Fi fallback
constexpr bool SIMULATE_GSM_SUCCESS = false;

constexpr char WIFI_SSID[] = "Wokwi-GUEST";
constexpr char WIFI_PASSWORD[] = "";

// -------------------- State machine --------------------
enum class State : uint8_t {
  SYSTEM_OFF,
  MONITORING,
  GYRO_VERIFY,
  POSTURE_CHECK,
  PRE_ALERT,
  GSM_ALERT,
  WIFI_FAILOVER,
  CONFIG_DEMO,
  RESULT
};

struct MotionSample {
  float ax = 0, ay = 0, az = 0;
  float totalG = 0, zG = 0;
  float gyroDps = 0;
};

struct Button {
  explicit Button(uint8_t p) : pin(p) {}

  uint8_t pin;
  bool stable = HIGH;
  bool raw = HIGH;
  bool pressedEvent = false;
  unsigned long changedAt = 0;

  void begin() {
    pinMode(pin, INPUT_PULLUP);
    stable = raw = digitalRead(pin);
  }

  void update(unsigned long now) {
    bool newRaw = digitalRead(pin);

    if (newRaw != raw) {
      raw = newRaw;
      changedAt = now;
    }

    if (now - changedAt >= BUTTON_DEBOUNCE_MS && stable != raw) {
      stable = raw;
      if (stable == LOW) pressedEvent = true;
    }
  }

  bool isPressed() const {
    return stable == LOW;
  }

  bool consumePress() {
    if (!pressedEvent) return false;
    pressedEvent = false;
    return true;
  }
};

// -------------------- Globals --------------------
Adafruit_MPU6050 mpu;
Button configButton(CONFIG_BUTTON_PIN);
Button cancelButton(CANCEL_BUTTON_PIN);

State state = State::SYSTEM_OFF;
MotionSample sample;

unsigned long stateStartedAt = 0;
unsigned long lastSensorAt = 0;
unsigned long lastLedAt = 0;
unsigned long configHoldStartedAt = 0;

uint8_t horizontalSamples = 0;
uint8_t gsmStep = 0;

bool redLedOn = false;
bool resultSuccess = false;
bool configHoldActive = false;
bool configTriggered = false;

// -------------------- Function declarations --------------------
const char *stateName(State s);
void enterState(State next);
void updateState(unsigned long now);

bool readMotion(MotionSample &m);
bool horizontalPosture(const MotionSample &m);

void handleConfigButton(unsigned long now);
void updateMonitoring(unsigned long now);
void updateGyro(unsigned long now);
void updatePosture(unsigned long now);
void updatePreAlert(unsigned long now);
void updateGsm(unsigned long now);
void updateWifi(unsigned long now);
void updateConfigDemo(unsigned long now);
void updateResult(unsigned long now);

void confirmFall();
void finishAlert(bool success, const char *message);
void outputsOff();
void wifiOff();

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GSM_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MPU_INT_PIN, INPUT);
  pinMode(SYSTEM_SWITCH_PIN, INPUT_PULLUP);

  configButton.begin();
  cancelButton.begin();
  outputsOff();

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("[FATAL] MPU6050 not found.");
    digitalWrite(RED_LED_PIN, HIGH);
    while (true) {
      tone(BUZZER_PIN, 2000);
      delay(150);
      noTone(BUZZER_PIN);
      delay(850);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu.setAccelerometerStandby(false, false, false);
  mpu.setGyroStandby(true, true, true);

  Serial.println("\n============================================");
  Serial.println(" Wearable Fall Detector - Refined Wokwi");
  Serial.println("============================================");
  Serial.println("[INFO] Green button: HOLD 3 s -> demo Wi-Fi portal.");
  Serial.println("[INFO] Red button: cancel during 10 s grace period.");
  Serial.println("[INFO] Fall logic: acceleration -> rotation -> posture.");

  enterState(digitalRead(SYSTEM_SWITCH_PIN) == LOW
                 ? State::MONITORING
                 : State::SYSTEM_OFF);
}

// -------------------- Main loop --------------------
void loop() {
  const unsigned long now = millis();

  configButton.update(now);
  cancelButton.update(now);

  if (digitalRead(SYSTEM_SWITCH_PIN) == HIGH) {
    if (state != State::SYSTEM_OFF) enterState(State::SYSTEM_OFF);
    return;
  }

  if (state == State::SYSTEM_OFF) {
    Serial.println("[POWER] Master switch ON.");
    enterState(State::MONITORING);
  }

  handleConfigButton(now);
  updateState(now);
}

// -------------------- Green button: long press only --------------------
void handleConfigButton(unsigned long now) {
  if (!configButton.isPressed()) {
    configHoldActive = false;
    configTriggered = false;
    return;
  }

  if (!configHoldActive) {
    configHoldActive = true;
    configHoldStartedAt = now;
  }

  if (state == State::MONITORING &&
      !configTriggered &&
      now - configHoldStartedAt >= CONFIG_HOLD_MS) {
    configTriggered = true;
    enterState(State::CONFIG_DEMO);
  }
}

// -------------------- State machine --------------------
void updateState(unsigned long now) {
  switch (state) {
    case State::SYSTEM_OFF:    break;
    case State::MONITORING:    updateMonitoring(now); break;
    case State::GYRO_VERIFY:   updateGyro(now); break;
    case State::POSTURE_CHECK: updatePosture(now); break;
    case State::PRE_ALERT:     updatePreAlert(now); break;
    case State::GSM_ALERT:     updateGsm(now); break;
    case State::WIFI_FAILOVER: updateWifi(now); break;
    case State::CONFIG_DEMO:   updateConfigDemo(now); break;
    case State::RESULT:        updateResult(now); break;
  }
}

void enterState(State next) {
  state = next;
  stateStartedAt = millis();
  Serial.printf("[STATE] -> %s\n", stateName(next));

  switch (next) {
    case State::SYSTEM_OFF:
      outputsOff();
      wifiOff();
      mpu.setGyroStandby(true, true, true);
      Serial.println("[POWER] SYSTEM OFF");
      break;

    case State::MONITORING:
      outputsOff();
      wifiOff();
      horizontalSamples = 0;
      gsmStep = 0;
      redLedOn = false;
      mpu.setAccelerometerStandby(false, false, false);
      mpu.setGyroStandby(true, true, true);
      Serial.println("[POWER] Accelerometer ON | Gyroscope OFF");
      break;

    case State::GYRO_VERIFY:
      horizontalSamples = 0;
      mpu.setGyroStandby(false, false, false);
      Serial.println("[POWER] Gyroscope ON for fall verification.");
      break;

    case State::POSTURE_CHECK:
      horizontalSamples = 0;
      Serial.println("[VERIFY] Checking post-fall posture.");
      break;

    case State::PRE_ALERT:
      lastLedAt = stateStartedAt;
      redLedOn = true;
      digitalWrite(RED_LED_PIN, HIGH);
      tone(BUZZER_PIN, 2000);
      break;

    case State::GSM_ALERT:
      noTone(BUZZER_PIN);
      digitalWrite(RED_LED_PIN, HIGH);
      digitalWrite(GSM_LED_PIN, HIGH);
      gsmStep = 0;
      Serial.println("[GSM MOCK] Cellular alert started.");
      break;

    case State::WIFI_FAILOVER:
      digitalWrite(GSM_LED_PIN, LOW);
      Serial.println("[WIFI] Connecting to Wokwi-GUEST...");
      WiFi.mode(WIFI_STA);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD, 6);
      break;

    case State::CONFIG_DEMO:
      outputsOff();
      wifiOff();
      mpu.setGyroStandby(true, true, true);

      Serial.println();
      Serial.println("========================================");
      Serial.println("      WI-FI CONFIGURATION PORTAL");
      Serial.println("========================================");
      Serial.println(" Device: Safety-Wearable");
      Serial.println();
      Serial.println(" Available Networks:");
      Serial.println("   1. Home_WiFi");
      Serial.println("   2. Caregiver_WiFi");
      Serial.println("   3. Wokwi-GUEST");
      Serial.println();
      Serial.println(" SSID:     [ Select Network ]");
      Serial.println(" Password: [ ************** ]");
      Serial.println();
      Serial.println("             [ SAVE ]");
      Serial.println("========================================");
      Serial.println("[DEMO] Wi-Fi setup portal displayed.");
      break;

    case State::RESULT:
      noTone(BUZZER_PIN);
      digitalWrite(GSM_LED_PIN, LOW);
      lastLedAt = stateStartedAt;
      redLedOn = true;
      digitalWrite(RED_LED_PIN, HIGH);
      wifiOff();
      break;
  }
}

const char *stateName(State s) {
  switch (s) {
    case State::SYSTEM_OFF:    return "SYSTEM_OFF";
    case State::MONITORING:    return "MONITORING";
    case State::GYRO_VERIFY:   return "GYRO_VERIFY";
    case State::POSTURE_CHECK: return "POSTURE_CHECK";
    case State::PRE_ALERT:     return "PRE_ALERT";
    case State::GSM_ALERT:     return "GSM_ALERT";
    case State::WIFI_FAILOVER: return "WIFI_FAILOVER";
    case State::CONFIG_DEMO:   return "CONFIG_DEMO";
    case State::RESULT:        return "RESULT";
  }
  return "UNKNOWN";
}

// -------------------- Fall detection --------------------
void updateMonitoring(unsigned long now) {
  if (now - lastSensorAt < SENSOR_INTERVAL_MS) return;
  lastSensorAt = now;

  if (!readMotion(sample)) return;

  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 1000) {
    lastPrint = now;
    Serial.printf("[MONITOR] Total=%.2f g | Gyro=OFF\n", sample.totalG);
  }

  if (sample.totalG > IMPACT_THRESHOLD_G) {
    Serial.printf("[TRIGGER] Acceleration %.2f g > %.2f g\n",
                  sample.totalG, IMPACT_THRESHOLD_G);
    enterState(State::GYRO_VERIFY);
  }
}

void updateGyro(unsigned long now) {
  const unsigned long elapsed = now - stateStartedAt;

  if (elapsed < GYRO_SETTLE_MS) return;

  if (elapsed >= GYRO_TIMEOUT_MS) {
    Serial.println("[REJECT] No strong rotation detected.");
    enterState(State::MONITORING);
    return;
  }

  if (now - lastSensorAt < SENSOR_INTERVAL_MS) return;
  lastSensorAt = now;

  if (!readMotion(sample)) return;

  Serial.printf("[GYRO] %.1f deg/s\n", sample.gyroDps);

  if (sample.gyroDps > GYRO_THRESHOLD_DPS) {
    Serial.println("[GYRO] Rapid rotation confirmed.");
    enterState(State::POSTURE_CHECK);
  }
}

void updatePosture(unsigned long now) {
  const unsigned long elapsed = now - stateStartedAt;

  if (elapsed >= POSTURE_TIMEOUT_MS) {
    Serial.println("[REJECT] Horizontal posture not confirmed.");
    enterState(State::MONITORING);
    return;
  }

  if (elapsed < POSTURE_SETTLE_MS ||
      now - lastSensorAt < SENSOR_INTERVAL_MS) return;

  lastSensorAt = now;
  if (!readMotion(sample)) return;

  if (horizontalPosture(sample)) {
    if (horizontalSamples < REQUIRED_HORIZONTAL_SAMPLES) horizontalSamples++;
    Serial.printf("[POSTURE] %u/%u | Z=%.2f g | Total=%.2f g\n",
                  horizontalSamples, REQUIRED_HORIZONTAL_SAMPLES,
                  sample.zG, sample.totalG);
  } else {
    horizontalSamples = 0;
  }

  if (horizontalSamples >= REQUIRED_HORIZONTAL_SAMPLES) confirmFall();
}

void confirmFall() {
  Serial.println("[CONFIRMED FALL] Acceleration + rotation + posture.");
  enterState(State::PRE_ALERT);
}

// -------------------- Alert sequence --------------------
void updatePreAlert(unsigned long now) {
  if (cancelButton.consumePress()) {
    Serial.println("[CANCELLED] Emergency alert cancelled.");
    enterState(State::MONITORING);
    return;
  }

  if (now - lastLedAt >= LED_BLINK_MS) {
    lastLedAt = now;
    redLedOn = !redLedOn;
    digitalWrite(RED_LED_PIN, redLedOn);
  }

  const unsigned long elapsed = now - stateStartedAt;

  static unsigned long lastCountdown = 0;
  if (now - lastCountdown >= 1000) {
    lastCountdown = now;
    unsigned long remaining =
        elapsed >= GRACE_PERIOD_MS ? 0 : (GRACE_PERIOD_MS - elapsed + 999) / 1000;
    Serial.printf("[GRACE] %lu second(s) remaining.\n", remaining);
  }

  if (elapsed >= GRACE_PERIOD_MS) enterState(State::GSM_ALERT);
}

void updateGsm(unsigned long now) {
  const unsigned long elapsed = now - stateStartedAt;

  if (gsmStep == 0 && elapsed >= 500) {
    Serial.println("[GSM MOCK] AT");
    gsmStep = 1;
  }
  if (gsmStep == 1 && elapsed >= 1200) {
    Serial.println("[GSM MOCK] Sending emergency SMS...");
    gsmStep = 2;
  }
  if (gsmStep == 2 && elapsed >= 2200) {
    Serial.println("[GSM MOCK] Calling caregiver...");
    gsmStep = 3;
  }

  if (SIMULATE_GSM_SUCCESS && elapsed >= 3200) {
    finishAlert(true, "Simulated GSM SMS/call completed.");
  } else if (!SIMULATE_GSM_SUCCESS && elapsed >= GSM_TIMEOUT_MS) {
    Serial.println("[GSM MOCK] GSM failed -> Wi-Fi fallback.");
    enterState(State::WIFI_FAILOVER);
  }
}

void updateWifi(unsigned long now) {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WIFI] Connected. IP: %s\n",
                  WiFi.localIP().toString().c_str());
    Serial.println("[EMAIL MOCK] Emergency notification sent.");
    finishAlert(true, "Wi-Fi failover completed.");
  } else if (now - stateStartedAt >= WIFI_TIMEOUT_MS) {
    finishAlert(false, "GSM and Wi-Fi both failed.");
  }
}

void updateResult(unsigned long now) {
  if (!resultSuccess && now - lastLedAt >= LED_BLINK_MS) {
    lastLedAt = now;
    redLedOn = !redLedOn;
    digitalWrite(RED_LED_PIN, redLedOn);
  }

  if (now - stateStartedAt >= RESULT_TIME_MS) {
    Serial.println("[RESET] Returning to monitoring.");
    enterState(State::MONITORING);
  }
}

void finishAlert(bool success, const char *message) {
  resultSuccess = success;
  Serial.printf("[%s] %s\n", success ? "SUCCESS" : "CRITICAL", message);
  enterState(State::RESULT);
}

// -------------------- Demo Wi-Fi portal --------------------
void updateConfigDemo(unsigned long now) {
  if (now - stateStartedAt >= CONFIG_DEMO_TIME_MS) {
    Serial.println("[DEMO] Configuration saved.");
    Serial.println("[DEMO] Returning to fall monitoring.\n");
    enterState(State::MONITORING);
  }
}

// -------------------- Sensor helpers --------------------
bool readMotion(MotionSample &m) {
  sensors_event_t a, gEvent, temp;

  if (!mpu.getEvent(&a, &gEvent, &temp)) {
    Serial.println("[ERROR] MPU6050 read failed.");
    return false;
  }

  m.ax = a.acceleration.x;
  m.ay = a.acceleration.y;
  m.az = a.acceleration.z;

  const float gx = gEvent.gyro.x * 180.0f / PI;
  const float gy = gEvent.gyro.y * 180.0f / PI;
  const float gz = gEvent.gyro.z * 180.0f / PI;

  m.gyroDps = sqrtf(gx * gx + gy * gy + gz * gz);

  const float accelMagnitude =
      sqrtf(m.ax * m.ax + m.ay * m.ay + m.az * m.az);

  m.totalG = accelMagnitude / G;
  m.zG = m.az / G;

  return true;
}

bool horizontalPosture(const MotionSample &m) {
  return fabsf(m.zG) < HORIZONTAL_Z_LIMIT_G &&
         m.totalG >= POSTURE_MIN_G &&
         m.totalG <= POSTURE_MAX_G;
}

// -------------------- Output/network helpers --------------------
void outputsOff() {
  noTone(BUZZER_PIN);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GSM_LED_PIN, LOW);
}

void wifiOff() {
  if (WiFi.getMode() != WIFI_OFF) {
    WiFi.disconnect(false, false);
    WiFi.mode(WIFI_OFF);
  }
}
