// Motor control pins
const int rightForward = 2;  // GPIO2
const int rightBackward = 4;  // GPIO4
const int leftForward = 18;  // GPIO18 (corrected from GPIO5 in original comment)
const int leftBackward = 5;  // GPIO5 (corrected from GPI18 in original comment)
const int rightPWM = 15;     // GPIO15
const int leftPWM = 19;      // GPIO19

// I2C and Sensor related includes and defines
#include <Wire.h>
#include <VL53L0X.h> // Using standard VL53L0X library
#include <stdlib.h>
#include <string.h>
#include <math.h>

// MPU-6050 (IMU) definitions
const uint8_t MPU_ADDR = 0x68; // AD0 to GND keeps default address 0x68
const int MPU_INT_PIN = 23;    // Data ready interrupt pin from MPU-6050

// MPU-6050 registers
const uint8_t MPU_REG_PWR_MGMT_1 = 0x6B;
const uint8_t MPU_REG_SMPLRT_DIV = 0x19;
const uint8_t MPU_REG_CONFIG = 0x1A;
const uint8_t MPU_REG_GYRO_CONFIG = 0x1B;
const uint8_t MPU_REG_ACCEL_CONFIG = 0x1C;
const uint8_t MPU_REG_INT_PIN_CFG = 0x37;
const uint8_t MPU_REG_INT_ENABLE = 0x38;
const uint8_t MPU_REG_INT_STATUS = 0x3A;
const uint8_t MPU_REG_GYRO_XOUT_H = 0x43;
const uint8_t MPU_REG_WHO_AM_I = 0x75;

const float MPU_GYRO_SENS = 131.0f; // LSB per deg/s for ±250°/s range

struct GyroState {
  float headingDeg = 0.0f;          // Integrated heading relative to startup (degrees)
  float rateDps = 0.0f;             // Latest angular velocity around Y axis (deg/s)
  float bias = 0.0f;                // Gyro bias computed during calibration (raw units)
  unsigned long lastUpdateMicros = 0;
  bool initialized = false;
  bool calibrated = false;
};

volatile bool imuDataReady = false;
GyroState gyroState;
bool imuReady = false;
const char* imuModelName = "Unknown IMU";

void IRAM_ATTR onImuDataReady() {
  imuDataReady = true;
}

// Protocol v2: M,<sequence>,<stop|forward|backward|left|right>,<lease_ms>\n
// No legacy command can start a motor. Flash together with the Jev host.
const uint8_t TCAADDR = 0x70;
const uint8_t channels[5] = {7, 6, 5, 4, 3};
VL53L0X sensors[5];
bool sensorReady[5] = {};
int rangeMM[5] = {};
unsigned long rangeAt[5] = {};
bool rangeValid[5] = {};
const char* rangeStatus[5] = {"init_failed", "init_failed", "init_failed", "init_failed", "init_failed"};
const unsigned long SENSOR_MAX_AGE_MS = 250;
const unsigned long MAX_LEASE_MS = 650;
const int CLEARANCE_MM = 300;
const int DRIVE_PWM = 100; // Calibrate on the chassis before free roaming.
const int TURN_PWM = 100;
unsigned long leaseUntil = 0;
unsigned long lastTelemetry = 0;
uint32_t commandId = 0;
const char* motion = "stop";
const char* stopReason = "startup";
char commandBuffer[80];
size_t commandLength = 0;
bool discardCommand = false;

bool initializeIMU();
void calibrateGyro(int samples = 1000);
bool readGyroRaw(int16_t &gx, int16_t &gy, int16_t &gz);
void updateGyro(bool force = false);

void stopMotors() {
  analogWrite(rightPWM, 0); analogWrite(leftPWM, 0);
  digitalWrite(rightForward, LOW); digitalWrite(rightBackward, LOW);
  digitalWrite(leftForward, LOW); digitalWrite(leftBackward, LOW);
}

void halt(const char* reason) {
  stopMotors(); motion = "stop"; stopReason = reason;
}

bool selectChannel(uint8_t channel) {
  Wire.beginTransmission(TCAADDR); Wire.write(1 << channel);
  return Wire.endTransmission() == 0;
}

bool freshIMU() {
  return imuReady && gyroState.calibrated &&
         (unsigned long)(micros() - gyroState.lastUpdateMicros) < 100000;
}

bool clearFor(const char* action) {
  if (!strcmp(action, "stop")) return true;
  if (!freshIMU()) return false;
  // Missing returns are unknown. Only fresh, detected obstacles veto motion.
  // Turns sweep the chassis, so check valid returns in every direction.
  for (int i = 0; i < 5; ++i) {
    if (!rangeValid[i] || millis() - rangeAt[i] > SENSOR_MAX_AGE_MS) continue;
    if (!strcmp(action, "backward") && i >= 1 && i <= 3) continue;
    if ((strcmp(action, "forward") || (i >= 1 && i <= 3)) && rangeMM[i] < CLEARANCE_MM)
      return false;
  }
  return true;
}

void applyMotion() {
  if (!strcmp(motion, "stop")) { stopMotors(); return; }
  bool left = !strcmp(motion, "left");
  bool right = !strcmp(motion, "right");
  bool backward = !strcmp(motion, "backward");
  digitalWrite(leftForward, (left || backward) ? LOW : HIGH);
  digitalWrite(leftBackward, (left || backward) ? HIGH : LOW);
  digitalWrite(rightForward, (right || backward) ? LOW : HIGH);
  digitalWrite(rightBackward, (right || backward) ? HIGH : LOW);
  analogWrite(leftPWM, (left || right) ? TURN_PWM : DRIVE_PWM);
  analogWrite(rightPWM, (left || right) ? TURN_PWM : DRIVE_PWM);
}

void acceptCommand(char* line) {
  unsigned long id, ttl;
  char action[12], extra;
  if (sscanf(line, "M,%lu,%11[^,],%lu%c", &id, action, &ttl, &extra) != 3) {
    halt("bad_command"); return;
  }
  commandId = id;
  if (!strcmp(action, "stop")) { halt("commanded"); return; }
  if (!ttl || ttl > MAX_LEASE_MS) { halt("bad_lease"); return; }
  const char* selected = !strcmp(action, "forward") ? "forward" :
                         !strcmp(action, "backward") ? "backward" :
                         !strcmp(action, "left") ? "left" :
                         !strcmp(action, "right") ? "right" : nullptr;
  if (!selected) { halt("bad_action"); return; }
  if (!clearFor(selected)) { halt("clearance_or_sensor"); return; }
  if (!strcmp(selected, "backward") && ttl > 250) ttl = 250;
  motion = selected; stopReason = "none"; leaseUntil = millis() + ttl;
  applyMotion();
}

void readCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'x') { halt("emergency_stop"); commandLength = 0; discardCommand = false; continue; }
    if (c == '\n') {
      if (!discardCommand && commandLength) {
        commandBuffer[commandLength] = 0; acceptCommand(commandBuffer);
      }
      commandLength = 0; discardCommand = false;
    } else if (c != '\r') {
      if (commandLength < sizeof(commandBuffer) - 1 && !discardCommand)
        commandBuffer[commandLength++] = c;
      else { halt("bad_command"); discardCommand = true; }
    }
  }
}

// One single-shot measurement per loop, through exactly one mux channel.
// Preserve other sensors' last readings while their turn is pending. The library's
// two polling phases are each bounded by the 30 ms timeout; service commands and
// the watchdog between sensors, rather than blocking on an entire five-sensor scan.
void pollRange() {
  static uint8_t i = 0;
  if (sensorReady[i] && selectChannel(channels[i])) {
    uint16_t mm = sensors[i].readRangeSingleMillimeters();
    bool timedOut = sensors[i].timeoutOccurred();
    rangeValid[i] = !timedOut && sensors[i].last_status == 0 && mm > 0 && mm < 8190;
    rangeStatus[i] = timedOut ? "timeout" : sensors[i].last_status != 0 ? "i2c_error" :
                     (mm == 0 || mm >= 8190) ? "out_of_range" : "ok";
    rangeMM[i] = mm;
    rangeAt[i] = millis();
  } else {
    rangeValid[i] = false;
    rangeStatus[i] = sensorReady[i] ? "mux_error" : "init_failed";
  }
  i = (i + 1) % 5;
}

void telemetry() {
  if (millis() - lastTelemetry < 50) return;
  lastTelemetry = millis();
  Serial.printf("{\"protocol\":2,\"uptime_ms\":%lu,\"command_id\":%lu,\"motion\":\"%s\",\"stop_reason\":\"%s\",\"tof_mm\":[",
                millis(), (unsigned long)commandId, motion, stopReason);
  for (int i = 0; i < 5; ++i) {
    if (i) Serial.print(',');
    if (rangeValid[i] && millis() - rangeAt[i] <= SENSOR_MAX_AGE_MS) Serial.print(rangeMM[i]);
    else Serial.print("null");
  }
  Serial.print("],\"tof_status\":[");
  for (int i = 0; i < 5; ++i) {
    if (i) Serial.print(',');
    Serial.printf("\"%s\"", rangeValid[i] && millis() - rangeAt[i] > SENSOR_MAX_AGE_MS ? "stale" : rangeStatus[i]);
  }
  Serial.printf("],\"heading_deg\":%.2f,\"yaw_rate_dps\":%.2f,\"imu_valid\":%s,\"imu_age_ms\":%lu,\"imu_model\":\"%s\"}\n",
    gyroState.headingDeg, gyroState.rateDps, freshIMU() ? "true" : "false",
    (unsigned long)(micros() - gyroState.lastUpdateMicros) / 1000, imuModelName);
}

void setup() {
  Serial.begin(115200);
  pinMode(rightForward, OUTPUT); pinMode(rightBackward, OUTPUT);
  pinMode(leftForward, OUTPUT); pinMode(leftBackward, OUTPUT);
  pinMode(rightPWM, OUTPUT); pinMode(leftPWM, OUTPUT);
  stopMotors();
  Wire.begin(); Wire.setTimeOut(5);
  for (int i = 0; i < 5; ++i) {
    if (!selectChannel(channels[i])) continue;
    sensors[i].setTimeout(200); // Match main's startup calibration allowance.
    sensorReady[i] = sensors[i].init();
    if (sensorReady[i]) {
      sensors[i].setMeasurementTimingBudget(20000);
      // Runtime reads are single-shot; no other sensor ranges concurrently.
      sensors[i].setTimeout(30);
    }
  }
  imuReady = initializeIMU();
  if (imuReady) calibrateGyro();
}

void loop() {
  readCommands();
  if (strcmp(motion, "stop") && (long)(millis() - leaseUntil) >= 0) halt("lease_expired");
  updateGyro();
  pollRange();
  if (strcmp(motion, "stop") && (long)(millis() - leaseUntil) >= 0) halt("lease_expired");
  updateGyro();
  readCommands();
  if (strcmp(motion, "stop") && !clearFor(motion)) halt("clearance_or_sensor");
  telemetry();
  delay(1);
}

bool initializeIMU() {
  Wire.beginTransmission(MPU_ADDR);
  uint8_t status = Wire.endTransmission();
  if (status != 0) {
    Serial.print("MPU: Address 0x68 not acknowledged. Status code: ");
    Serial.println(status);
    return false; // Device not found on bus
  }

  // Verify WHO_AM_I register
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_WHO_AM_I);
  status = Wire.endTransmission(false);
  if (status != 0) {
    Serial.print("MPU: Failed to write WHO_AM_I register. Status: ");
    Serial.println(status);
    return false;
  }
  Wire.requestFrom(MPU_ADDR, (uint8_t)1);
  if (!Wire.available()) {
    Serial.println("MPU: WHO_AM_I read returned no data.");
    return false;
  }
  uint8_t whoami = Wire.read();
  if (whoami == 0x68) {
    imuModelName = "MPU-6050";
  } else if (whoami == 0x70) {
    imuModelName = "MPU-6500";
  } else if (whoami == 0x71) {
    imuModelName = "MPU-9250";
  } else if (whoami == 0x72) {
    imuModelName = "MPU-9255";
  } else if (whoami == 0x98) {
    imuModelName = "MPU-6480"; // Likely factory ID for some clones
  } else {
    Serial.print("IMU: Unexpected WHO_AM_I: 0x");
    Serial.println(whoami, HEX);
    return false;
  }

  // Reset the device
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_PWR_MGMT_1);
  Wire.write(0x80);
  status = Wire.endTransmission();
  if (status != 0) {
    Serial.print("MPU: Failed to issue reset. Status: ");
    Serial.println(status);
    return false;
  }
  delay(100);

  // Wake up and select PLL with X axis gyroscope reference
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_PWR_MGMT_1);
  Wire.write(0x01);
  status = Wire.endTransmission();
  if (status != 0) {
    Serial.print("MPU: Failed to exit sleep. Status: ");
    Serial.println(status);
    return false;
  }

  // Sample rate divider -> 1kHz / (1 + 9) = 100 Hz
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_SMPLRT_DIV);
  Wire.write(9);
  status = Wire.endTransmission();
  if (status != 0) {
    Serial.print("MPU: Failed to set sample rate. Status: ");
    Serial.println(status);
    return false;
  }

  // Configure DLPF (set to 0x03 -> ~44 Hz bandwidth)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_CONFIG);
  Wire.write(0x03);
  status = Wire.endTransmission();
  if (status != 0) {
    Serial.print("MPU: Failed to set DLPF. Status: ");
    Serial.println(status);
    return false;
  }

  // Gyro full scale ±250°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_GYRO_CONFIG);
  Wire.write(0x00);
  status = Wire.endTransmission();
  if (status != 0) {
    Serial.print("MPU: Failed to set gyro range. Status: ");
    Serial.println(status);
    return false;
  }

  // Accelerometer ±2g (default) - keep for completeness
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_ACCEL_CONFIG);
  Wire.write(0x00);
  status = Wire.endTransmission();
  if (status != 0) {
    Serial.print("MPU: Failed to set accel range. Status: ");
    Serial.println(status);
    return false;
  }

  // Configure interrupt pin (active low, open drain, latch until read)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_INT_PIN_CFG);
  Wire.write(0x10);
  status = Wire.endTransmission();
  if (status != 0) {
    Serial.print("MPU: Failed to set INT pin config. Status: ");
    Serial.println(status);
    return false;
  }

  // Enable data ready interrupt
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_INT_ENABLE);
  Wire.write(0x01);
  status = Wire.endTransmission();
  if (status != 0) {
    Serial.print("MPU: Failed to enable interrupt. Status: ");
    Serial.println(status);
    return false;
  }

  // Clear any pending interrupts by reading the status register
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_INT_STATUS);
  if (Wire.endTransmission(false) == 0) {
    Wire.requestFrom(MPU_ADDR, (uint8_t)1);
    if (Wire.available()) Wire.read();
  }

  gyroState.initialized = true;
  gyroState.lastUpdateMicros = micros();
  imuDataReady = false;
  return true;
}

void calibrateGyro(int samples) {
  if (!gyroState.initialized) return;

  long sum = 0;
  int16_t gx, gy, gz;
  const int maxSamples = max(samples, 100);

  // Allow sensor to settle
  delay(50);

  unsigned long started = millis();
  for (int i = 0; i < maxSamples; ++i) {
    if (millis() - started > 5000) return;
    if (!readGyroRaw(gx, gy, gz)) {
      delay(2);
      --i;
      continue;
    }
    sum += gy;
    delay(2);
  }

  gyroState.bias = sum / (float)maxSamples;
  gyroState.calibrated = true;
  gyroState.lastUpdateMicros = micros();
  imuDataReady = false;
}

bool readGyroRaw(int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_GYRO_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  Wire.requestFrom(MPU_ADDR, (uint8_t)6);
  if (Wire.available() < 6) {
    return false;
  }

  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();

  gx = rawX;
  gy = rawY;
  gz = rawZ;
  return true;
}

void updateGyro(bool force) {
  if (!imuReady) return;
  if (!force && micros() - gyroState.lastUpdateMicros < 10000) return;

  int16_t gx, gy, gz;
  if (!readGyroRaw(gx, gy, gz)) {
    return;
  }

  imuDataReady = false;

  unsigned long now = micros();
  if (gyroState.lastUpdateMicros == 0) {
    gyroState.lastUpdateMicros = now;
    return;
  }

  float dt = (now - gyroState.lastUpdateMicros) / 1000000.0f;
  gyroState.lastUpdateMicros = now;

  float gyroY = (gy - gyroState.bias) / MPU_GYRO_SENS;
  gyroState.rateDps = gyroY;
  gyroState.headingDeg += gyroY * dt;
  // Keep a continuous startup-relative heading; never reset at each turn.
}

float getHeadingDegrees() {
  return gyroState.headingDeg;
}

float normalizeAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}
