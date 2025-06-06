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
const int MIN_DISTANCE = 50; // Minimum distance to obstacle in cm (approx 8 inches) - Increased from 14
bool pid_active = true; // Flag to enable/disable PID control

void setup() {
  Serial.begin(115200); // Increased baud rate
  Wire.begin(); // ESP32 default SDA=21, SCL=22
  
  // Set motor pins as outputs
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);
  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(rightPWM, OUTPUT);
  pinMode(leftPWM, OUTPUT);

  // Initialize sensors
  setupSensors();

  // PID setup
  Setpoint = 0; // Target for steering, 0 means balanced
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255); // PID output will adjust motor speed difference

  // Initialize all motors to stop
  stopMotors();
}

void loop() {
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
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);
  digitalWrite(leftForward, LOW); // Option 1: Pivot - Left motor backward or stop
  digitalWrite(leftBackward, LOW); // if using LOW, it stops. For tighter turn, could set leftBackward HIGH
  analogWrite(rightPWM, baseSpeed); 
  analogWrite(leftPWM, constrain(baseSpeed / 2, 0, 255)); // Slower on left, or 0 for pivot. Ensure it does not go below 0 if baseSpeed is low.
                                                        // If baseSpeed/2 is too low for movement, it will just pivot on stopped wheel.
                                                        // Consider setting a fixed turning speed or ensuring baseSpeed is high enough.
                                                        // For now, if baseSpeed/2 < 80 and not 0, it might not turn as expected.
                                                        // Let's ensure left still moves, even if slowly, or stops for pivot
  // If pivoting (left motor stopped): analogWrite(leftPWM, 0);
  // If gentle turn: analogWrite(leftPWM, constrain(baseSpeed / 2, 80, 255) if baseSpeed/2 > 0 else 0);
  // For simplicity now, let's try a fixed slower speed or stop for the inner wheel
  analogWrite(leftPWM, 0); // Pivot turn by default
}

void right_turn() {
  digitalWrite(rightForward, LOW); // Option 1: Pivot - Right motor backward or stop
  digitalWrite(rightBackward, LOW); // if using LOW, it stops. For tighter turn, could set rightBackward HIGH
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  analogWrite(rightPWM, 0); // Pivot turn by default
  analogWrite(leftPWM, baseSpeed); 
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