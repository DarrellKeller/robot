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
#include <PID_v1.h>
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

bool initializeIMU();
void calibrateGyro(int samples = 1000);
bool readGyroRaw(int16_t &gx, int16_t &gy, int16_t &gz);
void updateGyro(bool force = false);
float getHeadingDegrees();
float normalizeAngle(float angle);
float headingError(float target);
bool turnDegrees(float degrees, int fastSpeed = 160, int slowSpeed = 110, float tolerance = 3.0f);
void applyPivot(int direction, int speed);
void zeroHeading();
void pivotWithoutIMU(int direction, unsigned long durationMs, int speed);

#define TCAADDR 0x70 // TCA9548A I2C multiplexer address

// Sensor objects
VL53L0X sensorL90; // Left 90 degrees on TCA Channel 7
VL53L0X sensorL45; // Left 45 degrees on TCA Channel 6
VL53L0X sensorF;   // Front on TCA Channel 5
VL53L0X sensorR45; // Right 45 degrees on TCA Channel 4
VL53L0X sensorR90; // Right 90 degrees on TCA Channel 3

// Moving average window size - Removed as VL53L0X_MA.h is not used
// int window = 3; 

// PID Variables
double Setpoint, Input, Output;
double Kp = 50, Ki = 1, Kd = 20; // PID gains - Increased Kp for more drastic turns, adjusted Kd
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int baseSpeed = 180; // Default base speed (0-255)
const int MIN_DISTANCE = 30; // Minimum distance to obstacle in cm (approx 8 inches) - Increased from 14
bool pid_active = false; // Flag to enable/disable PID control

void setup() {
  Serial.begin(115200); // Increased baud rate
  Wire.begin(); // ESP32 default SDA=21, SCL=22

  // Setup MPU interrupt pin
  pinMode(MPU_INT_PIN, INPUT_PULLUP);
  
  // Set motor pins as outputs
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);
  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(rightPWM, OUTPUT);
  pinMode(leftPWM, OUTPUT);

  // Initialize sensors
  setupSensors();

  imuReady = initializeIMU();
  if (imuReady) {
    calibrateGyro();
    zeroHeading();
    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), onImuDataReady, FALLING);
    Serial.print(imuModelName);
    Serial.println(" initialized and calibrated.");
  } else {
    Serial.println("Failed to initialize MPU-6050!");
  }

  // PID setup
  Setpoint = 0; // Target for steering, 0 means balanced
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255); // PID output will adjust motor speed difference

  // Initialize all motors to stop
  stopMotors();
}

void loop() {
  if (imuReady) {
    updateGyro();
  }
  // Handle manual serial commands
  if (Serial.available() > 0) {
    char command = Serial.read();
    handleSerialCommand(command);
  }

  if (pid_active) {
    // Read sensors
    selectChannel(7);
    // int l90 = sensorL90.movingAverage(sensorL90.readRangeSingleMillimeters() / 10); // Using direct read
    int l90 = sensorL90.readRangeSingleMillimeters() / 10;
    if (sensorL90.timeoutOccurred()) { Serial.println("L90 TIMEOUT"); l90 = 200; } // Handle timeout, assume max distance
    selectChannel(6);
    // int l45 = sensorL45.movingAverage(sensorL45.readRangeSingleMillimeters() / 10);
    int l45 = sensorL45.readRangeSingleMillimeters() / 10;
    if (sensorL45.timeoutOccurred()) { Serial.println("L45 TIMEOUT"); l45 = 200; }
    selectChannel(5);
    // int front = sensorF.movingAverage(sensorF.readRangeSingleMillimeters() / 10);
    int front = sensorF.readRangeSingleMillimeters() / 10;
    if (sensorF.timeoutOccurred()) { Serial.println("F TIMEOUT"); front = 200; }
    selectChannel(4);
    // int r45 = sensorR45.movingAverage(sensorR45.readRangeSingleMillimeters() / 10);
    int r45 = sensorR45.readRangeSingleMillimeters() / 10;
    if (sensorR45.timeoutOccurred()) { Serial.println("R45 TIMEOUT"); r45 = 200; }
    selectChannel(3);
    // int r90 = sensorR90.movingAverage(sensorR90.readRangeSingleMillimeters() / 10);
    int r90 = sensorR90.readRangeSingleMillimeters() / 10;
    if (sensorR90.timeoutOccurred()) { Serial.println("R90 TIMEOUT"); r90 = 200; }

    // Avoid division by zero and unrealistic high values (e.g. > 200cm)
    front = constrain(max(front, 1), 1, 200);
    l45 = constrain(max(l45, 1), 1, 200);
    l90 = constrain(max(l90, 1), 1, 200);
    r45 = constrain(max(r45, 1), 1, 200);
    r90 = constrain(max(r90, 1), 1, 200);

    // Compute proximity and softmax weights
    double proxFront = 1.0 / front;
    double proxL45 = 1.0 / l45;
    double proxL90 = 1.0 / l90;
    double proxR45 = 1.0 / r45;
    double proxR90 = 1.0 / r90;
    double sumProx = proxFront + proxL45 + proxL90 + proxR45 + proxR90;
    
    double wF = proxFront / sumProx;
    double wL45 = proxL45 / sumProx;
    double wL90 = proxL90 / sumProx;
    double wR45 = proxR45 / sumProx;
    double wR90 = proxR90 / sumProx;

    // Calculate steering input for PID
    // Positive steering means turn right, negative means turn left
    double steering = (wR45 + wR90) - (wL45 + wL90); // Match friend's: (LHS) - (RHS) for steering value
                                                     // If L is smaller (more prox), value is negative (turn left)
                                                     // If R is smaller (more prox), value is positive (turn right)
                                                     // My PID output: Positive -> R motor decrease, L motor increase

    Input = steering;
    myPID.Compute(); // Output will be from -255 to 255

    // Determine current speed based on sensors and baseSpeed
    int currentSpeed = baseSpeed;
    bool criticalStop = (front < MIN_DISTANCE / 2); // Very close obstacle directly in front

    if (criticalStop) {
        currentSpeed = 0; // Immediate stop for very close frontal obstacles
    } else if (front < MIN_DISTANCE || l45 < MIN_DISTANCE || r45 < MIN_DISTANCE) {
        currentSpeed = baseSpeed / 2; // Slow down if obstacles are somewhat close, allowing PID to steer
        // Ensure currentSpeed doesn't go below a minimum operational speed if baseSpeed/2 is too low
        currentSpeed = max(currentSpeed, 80); // Assuming 80 is a good minimum maneuvering speed
        if (baseSpeed < 160) currentSpeed = baseSpeed; // Avoid going slower than base if base is already low
    }
    // The old stopForObstacle logic is replaced by the above

    int leftMotorSpeed = currentSpeed;
    int rightMotorSpeed = currentSpeed;

    // Apply PID output to differentiate motor speeds for steering
    // If Output is positive (turn right command from PID based on friend's steering formula),
    // we want to slow down the right motor or speed up the left motor.
    // My convention for moveMotors: higher speed = faster.
    // Output > 0 means steer right (LHS smaller distances / higher prox) -> left motor faster, right motor slower
    // Output < 0 means steer left (RHS smaller distances / higher prox) -> right motor faster, left motor slower
    leftMotorSpeed += Output; 
    rightMotorSpeed -= Output;

    // Constrain motor speeds
    // Using friend's constraints (42-127) if moving, else 0. Max can be 255.
    // Let's use a wider range like 60-200 for ESP32 potentially. For now, use 42-255
    if (currentSpeed == 0) {
      leftMotorSpeed = 0;
      rightMotorSpeed = 0;
    } else {
      leftMotorSpeed = constrain(leftMotorSpeed, 80, 255); // Updated min speed to 80
      rightMotorSpeed = constrain(rightMotorSpeed, 80, 255); // Updated min speed to 80
    }
    
    moveMotors(leftMotorSpeed, rightMotorSpeed, (Output > 0)); // Pass PID output to hint direction if needed

    // Serial output for Python script
    // Format: L90,L45,F,R45,R90,sW_L90,sW_L45,sW_F,sW_R45,sW_R90,SteeringIn,PID_Out,L_Speed,R_Speed,BaseSpeed,CurSpeed,PID_Active_ESP
    Serial.print(l90); Serial.print(","); Serial.print(l45); Serial.print(","); Serial.print(front); Serial.print(","); Serial.print(r45); Serial.print(","); Serial.print(r90); Serial.print(",");
    Serial.print(wL90, 4); Serial.print(","); Serial.print(wL45, 4); Serial.print(","); Serial.print(wF, 4); Serial.print(","); Serial.print(wR45, 4); Serial.print(","); Serial.print(wR90, 4); Serial.print(",");
    Serial.print(Input, 4); Serial.print(","); Serial.print(Output, 4); Serial.print(",");
    Serial.print(leftMotorSpeed); Serial.print(","); Serial.print(rightMotorSpeed); Serial.print(",");
    Serial.print(baseSpeed); Serial.print(","); Serial.print(currentSpeed); Serial.print(","); Serial.println(pid_active ? 1 : 0);

  } // end if(pid_active)
  
  delay(50); // Loop delay
}

void handleSerialCommand(char command) {
  if (imuReady) {
    updateGyro(true);
  }
  switch(command) {
    case 'w':
      pid_active = false; // Disable PID for manual control
      forward();
      break;
    case 's':
      pid_active = false;
      reverse();
      break;
    case 'a':
      pid_active = false;
      left_turn();
      break;
    case 'd':
      pid_active = false;
      right_turn();
      break;
    case 'x':
      pid_active = false;
      stopMotors();
      break;
    case 'p': // Toggle PID
      pid_active = !pid_active;
      if (!pid_active) stopMotors(); // Stop if disabling PID
      Serial.print("PID Active: "); Serial.println(pid_active);
      break;
    case '1': baseSpeed = 255 * 0.1; Serial.print("BaseSpeed: "); Serial.println(baseSpeed); break;
    case '2': baseSpeed = 255 * 0.2; Serial.print("BaseSpeed: "); Serial.println(baseSpeed); break;
    case '3': baseSpeed = 255 * 0.3; Serial.print("BaseSpeed: "); Serial.println(baseSpeed); break;
    case '4': baseSpeed = 255 * 0.4; Serial.print("BaseSpeed: "); Serial.println(baseSpeed); break;  
    case '5': baseSpeed = 255 * 0.5; Serial.print("BaseSpeed: "); Serial.println(baseSpeed); break;
    case '6': baseSpeed = 255 * 0.6; Serial.print("BaseSpeed: "); Serial.println(baseSpeed); break;
    case '7': baseSpeed = 255 * 0.7; Serial.print("BaseSpeed: "); Serial.println(baseSpeed); break;
    case '8': baseSpeed = 255 * 0.8; Serial.print("BaseSpeed: "); Serial.println(baseSpeed); break;
    case '9': baseSpeed = 255 * 0.9; Serial.print("BaseSpeed: "); Serial.println(baseSpeed); break;
    case '0': baseSpeed = 255 * 1.0; Serial.print("BaseSpeed: "); Serial.println(baseSpeed); break;
  }
}

void stopMotors() { // Renamed from stop()
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, LOW);
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, LOW);
  // Ensure PWM is also off
  analogWrite(rightPWM, 0);
  analogWrite(leftPWM, 0);
}

void moveMotors(int leftSpeed, int rightSpeed, bool turningRightHint) {
  // For now, simple forward differential speed.
  // Speeds are magnitudes (0-255)
  // Left Motor
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  analogWrite(leftPWM, constrain(leftSpeed, 0, 255));

  // Right Motor
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);
  analogWrite(rightPWM, constrain(rightSpeed, 0, 255));
}

void pivotWithoutIMU(int direction, unsigned long durationMs, int speed) {
  unsigned long start = millis();
  applyPivot(direction, speed);
  while (millis() - start < durationMs) {
    delay(5);
  }
  stopMotors();
}

// Manual control functions (will set motors directly, overriding PID if pid_active is false)
void forward() {
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  analogWrite(rightPWM, baseSpeed);
  analogWrite(leftPWM, baseSpeed);
}

void reverse() {
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, HIGH);
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, HIGH);
  analogWrite(rightPWM, baseSpeed);
  analogWrite(leftPWM, baseSpeed);
}

void left_turn() {
  if (imuReady) {
    if (!turnDegrees(75.0f)) {
      Serial.println("IMU turn (left) timed out.");
    }
    return;
  }
  pivotWithoutIMU(1, 550, constrain(baseSpeed, 100, 200));
}

void right_turn() {
  if (imuReady) {
    if (!turnDegrees(-75.0f)) {
      Serial.println("IMU turn (right) timed out.");
    }
    return;
  }
  pivotWithoutIMU(-1, 550, constrain(baseSpeed, 100, 200));
}

// Function to select I2C channel on TCA9548A
void selectChannel(uint8_t i2cBus) {
  if (i2cBus > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i2cBus);
  Wire.endTransmission();
}

// Function to setup sensors
void setupSensors() {
  uint8_t channels[] = {7, 6, 5, 4, 3}; // L90, L45, F, R45, R90
  VL53L0X* sensors[] = {&sensorL90, &sensorL45, &sensorF, &sensorR45, &sensorR90};
  const char* sensorNames[] = {"L90_CH7", "L45_CH6", "F_CH5", "R45_CH4", "R90_CH3"};

  for (int i = 0; i < 5; ++i) {
    selectChannel(channels[i]);
    sensors[i]->setTimeout(200); // Standard timeout from Adafruit library is 500ms, 200 is fine for quicker reads
    if (!sensors[i]->init()) {
      Serial.print("Failed to detect and initialize ");
      Serial.print(sensorNames[i]);
      Serial.println(" sensor!");
      // Potentially loop forever or set a flag
    } else {
      Serial.print(sensorNames[i]);
      Serial.println(" initialized.");
       // Configuration for VL53L0X (example settings, can be tuned)
      sensors[i]->setMeasurementTimingBudget(20000); // microseconds (20ms = 20000us). Valid range 20ms to 1000ms.
                                                     // Longer budget for more accuracy, shorter for faster reads.
      // sensors[i]->setSignalRateLimit(0.1); // Example: advanced tuning, default 0.25 Mcps
      // sensors[i]->setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18); // Example: advanced tuning
      // sensors[i]->setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14); // Example: advanced tuning
    }
    // sensors[i]->setWindowSize(window); // Removed, not part of standard VL53L0X library
  }
} 

// -----------------------------
// IMU helper implementations
// -----------------------------

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

  for (int i = 0; i < maxSamples; ++i) {
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
  if (!imuDataReady && !force) return;

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
  gyroState.headingDeg = normalizeAngle(gyroState.headingDeg);
}

float getHeadingDegrees() {
  return gyroState.headingDeg;
}

float normalizeAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

void applyPivot(int direction, int speed) {
  speed = constrain(speed, 0, 255);
  if (direction >= 0) { // Positive -> left turn
    digitalWrite(rightForward, HIGH);
    digitalWrite(rightBackward, LOW);
    digitalWrite(leftForward, LOW);
    digitalWrite(leftBackward, HIGH);
  } else { // Negative -> right turn
    digitalWrite(rightForward, LOW);
    digitalWrite(rightBackward, HIGH);
    digitalWrite(leftForward, HIGH);
    digitalWrite(leftBackward, LOW);
  }
  analogWrite(rightPWM, speed);
  analogWrite(leftPWM, speed);
}

void zeroHeading() {
  gyroState.headingDeg = 0.0f;
  gyroState.lastUpdateMicros = micros();
}

float headingError(float target) {
  float diff = target - gyroState.headingDeg;
  return normalizeAngle(diff);
}

bool turnDegrees(float degrees, int fastSpeed, int slowSpeed, float tolerance) {
  if (!imuReady || !gyroState.calibrated) {
    return false;
  }

  int direction = (degrees >= 0.0f) ? 1 : -1;
  fastSpeed = constrain(fastSpeed, 80, 255);
  slowSpeed = constrain(slowSpeed, 60, fastSpeed - 5);
  float target = degrees;
  float absTarget = fabs(target);

  zeroHeading();
  imuDataReady = false;

  unsigned long startTime = millis();
  unsigned long timeout = max(3000UL, (unsigned long)(fabs(degrees) * 28.0f));

  uint8_t stage = 0; // 0 = soft start, 1 = medium push, 2 = high torque

  while (millis() - startTime < timeout) {
    updateGyro();

    float error = headingError(target);
    float absError = fabs(error);
    float absHeading = fabs(gyroState.headingDeg);

    if (absError <= tolerance) {
      stopMotors();
      return true;
    }

    if (stage == 0 && (absHeading >= absTarget * 0.45f || millis() - startTime > 500)) {
      stage = 1;
    }
    if (stage == 1 && (absHeading >= absTarget * 0.85f || millis() - startTime > 1500)) {
      stage = 2;
    }

    int speed;
    unsigned long pulseOn;
    unsigned long pulsePause;

    if (stage == 0) {
      speed = max(slowSpeed - 5, 95);
      pulseOn = 18;
      pulsePause = 14;
    } else if (stage == 1) {
      speed = max(slowSpeed + 30, 120);
      pulseOn = 20;
      pulsePause = 10;
    } else {
      speed = min(max(fastSpeed + 50, slowSpeed + 50), 240);
      pulseOn = 22;
      pulsePause = 10;
    }

    if (absError < 18.0f) {
      speed = max(speed - 10, 100);
      pulseOn = max(pulseOn - 3, (unsigned long)12);
      pulsePause += 4;
    }
    if (absError < 8.0f) {
      speed = max(speed - 20, 80);
      pulseOn = max(pulseOn - 4, (unsigned long)8);
      pulsePause += 6;
    }

    applyPivot(direction, speed);
    delay(pulseOn);
    stopMotors();
    delay(pulsePause);

    updateGyro(true); // Force read after each pulse
  }

  stopMotors();
  return false;
}