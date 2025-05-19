#include <Wire.h>

//---------------------------------------------------------------------
// Debug Flags (Set to true to enable specific print sections)
//---------------------------------------------------------------------
bool DEBUG_SETUP = true;
bool DEBUG_MPU_RAW = false;      // Prints raw Accel/Gyro values
bool DEBUG_MPU_ANGLES = true;   // Prints calculated Accelerometer and Gyro angles, and filtered angle
bool DEBUG_PID_TERMS = true;    // Prints P, I, D terms and PID output
bool DEBUG_MOTOR_COMMAND = true; // Prints the command sent to motors

//---------------------------------------------------------------------
// Pin Definitions (Your Configuration)
//---------------------------------------------------------------------
// Motor A (e.g., Left Motor)
const int ENA_PIN = 9;  // PWM Speed control for Motor A
const int IN1_PIN = 4;  // Direction control 1 for Motor A
const int IN2_PIN = 7;  // Direction control 2 for Motor A

// Motor B (e.g., Right Motor)
const int ENB_PIN = 3;  // PWM Speed control for Motor B
const int IN3_PIN = 6;  // Direction control 1 for Motor B
const int IN4_PIN = 8;  // Direction control 2 for Motor B

// MPU6050 Pins
// SCL -> A5 (Implicitly used by Wire library)
// SDA -> A4 (Implicitly used by Wire library)
const int MPU_INT_PIN = 2; // Interrupt pin

//---------------------------------------------------------------------
// MPU6050 Settings
//---------------------------------------------------------------------
const int MPU_ADDR = 0x68; // MPU6050 I2C address
float accX_raw, accY_raw, accZ_raw;
float gyroX_raw, gyroY_raw, gyroZ_raw;

float angleAccX, angleAccY;     // Angle from accelerometer
float currentAngleX = 0.0;      // Filtered angle for X-axis (pitch - for balancing)
// float currentAngleY = 0.0;   // Filtered angle for Y-axis (roll - not primary for 2-wheel)

// Store gyro-integrated angle separately for clarity in filter
float gyroAngleX_integrated = 0.0;

// Complementary Filter
float alpha = 0.98; // Weight for gyroscope data (trust gyro more for short term)

// Loop timing
unsigned long prevTimeMPU = 0;
float dtMPU; // Delta time for MPU calculations in seconds

volatile bool mpuInterrupt = false; // Flag set by ISR

// MPU6050 Offsets (Ideally, calibrate these and store them)
float accX_offset = 0.0;
float accY_offset = 0.0;
float accZ_offset = 0.0; // Usually, Z offset isn't as critical for angle if it's near 1g
float gyroX_offset = 0.0;
float gyroY_offset = 0.0;
float gyroZ_offset = 0.0;

//---------------------------------------------------------------------
// PID Controller Settings
//---------------------------------------------------------------------
// --- START WITH THESE LOW AND TUNE CAREFULLY ---
double Kp = 15.0; // START LOW (e.g., 10-30)
double Ki = 0.5;  // START VERY LOW or 0 (e.g., 0-2)
double Kd = 5.0;  // START LOW (e.g., 1-10)

double setpointAngle = 0.0; // Target angle (robot vertical)
double pidError = 0;
double prevError = 0;
double integralError = 0;
double derivativeError = 0; // Not strictly needed to store, but can be useful for debug
double pidOutput = 0;

const int PID_OUT_MIN = -255;
const int PID_OUT_MAX = 255;

unsigned long prevTimePID = 0;
float dtPID; // Delta time for PID calculations

//---------------------------------------------------------------------
// MPU6050 Interrupt Service Routine
//---------------------------------------------------------------------
void mpuDataReadyISR() {
  mpuInterrupt = true;
}

//---------------------------------------------------------------------
// Setup Function
//---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000); // Wait for serial connection (optional)

  if (DEBUG_SETUP) Serial.println("\n--- System Setup Starting ---");

  Wire.begin();

  setupMPU6050();

  if (DEBUG_SETUP) Serial.print("Configuring MPU Interrupt on D");
  if (DEBUG_SETUP) Serial.println(MPU_INT_PIN);
  pinMode(MPU_INT_PIN, INPUT_PULLUP); // Or INPUT if external pull-up used
  attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), mpuDataReadyISR, FALLING);

  if (DEBUG_SETUP) Serial.println("Configuring Motor Pins...");
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  stopMotors(); // Ensure motors are off
  if (DEBUG_SETUP) Serial.println("Motors Stopped Initially.");

  if (DEBUG_SETUP) Serial.println("Waiting for MPU to settle and first data...");
  // Wait for first interrupt or a timeout to ensure dtMPU is valid
  unsigned long setupStartTime = millis();
  while(!mpuInterrupt && (millis() - setupStartTime < 2000)) { delay(1); }

  if (mpuInterrupt) {
    mpuInterrupt = false; // Consume the first interrupt
    readMPU6050Data();    // Initial read to populate values
    calculateAngles();    // Initial angle calculation
    currentAngleX = angleAccX; // Initialize filtered angle with accelerometer angle
    gyroAngleX_integrated = angleAccX; // And the integrated gyro angle too
    if (DEBUG_SETUP) Serial.print("Initial Angle X (from Accel): ");
    if (DEBUG_SETUP) Serial.println(currentAngleX);
  } else {
    if (DEBUG_SETUP) Serial.println("WARNING: MPU Interrupt not received during setup timeout!");
  }
  
  prevTimeMPU = micros(); // Initialize timers after first data
  prevTimePID = micros();

  if (DEBUG_SETUP) Serial.println("--- System Setup Complete ---");
}

//---------------------------------------------------------------------
// Main Loop
//---------------------------------------------------------------------
void loop() {
  if (mpuInterrupt) {
    mpuInterrupt = false; // Reset interrupt flag

    unsigned long currentTimeMPU = micros();
    dtMPU = (currentTimeMPU - prevTimeMPU) / 1000000.0; // dt in seconds
    prevTimeMPU = currentTimeMPU;

    if (dtMPU <= 0) { // Should not happen if micros() is working
        if (DEBUG_MPU_ANGLES) Serial.println("ERR: dtMPU <= 0!");
        return; // Skip this cycle
    }

    readMPU6050Data();
    calculateAngles();

    // --- PID Calculation ---
    // Using dtMPU for PID as well, assuming PID runs right after MPU read
    // If PID runs at a different rate, calculate dtPID separately
    dtPID = dtMPU; 

    pidError = setpointAngle - currentAngleX;

    // Integral Term (with anti-windup based on output saturation)
    if (abs(pidOutput) < PID_OUT_MAX * 0.95) { // Only integrate if not fully saturated
       integralError += pidError * dtPID;
    }
    // Clamp integral to prevent excessive windup if necessary
    // float maxIntegralTermContribution = 100.0; // Max effect I-term can have on output
    // if (Ki != 0) integralError = constrain(integralError, -maxIntegralTermContribution/Ki, maxIntegralTermContribution/Ki);


    derivativeError = (pidError - prevError) / dtPID; // dtPID is same as dtMPU here
    prevError = pidError;

    double pTerm = Kp * pidError;
    double iTerm = Ki * integralError;
    double dTerm = Kd * derivativeError;
    
    pidOutput = pTerm + iTerm + dTerm;
    pidOutput = constrain(pidOutput, PID_OUT_MIN, PID_OUT_MAX);

    if (DEBUG_PID_TERMS) {
      Serial.print("PID: Err="); Serial.print(pidError, 2);
      Serial.print(" P="); Serial.print(pTerm, 2);
      Serial.print(" I="); Serial.print(iTerm, 2);
      Serial.print(" D="); Serial.print(dTerm, 2);
      Serial.print(" Out="); Serial.println(pidOutput, 0);
    }

    // --- Motor Control ---
    // Fall detection: if robot tilts too far, stop motors
    if (abs(currentAngleX) > 40.0) { // Tunable fall angle
      stopMotors();
      integralError = 0; // Reset integral to prevent large jerk on recovery
      if (DEBUG_MOTOR_COMMAND) Serial.println("FALL DETECTED - MOTORS STOPPED");
    } else {
      controlMotors((int)pidOutput);
      if (DEBUG_MOTOR_COMMAND) {
        Serial.print("MotorCmd: "); Serial.println((int)pidOutput);
      }
    }
  } // End of if(mpuInterrupt)
} // End of loop()

//---------------------------------------------------------------------
// MPU6050 Functions
//---------------------------------------------------------------------
void setupMPU6050() {
  if (DEBUG_SETUP) Serial.println("Setting up MPU6050...");
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00); // Wake up MPU6050
  if (Wire.endTransmission(true) == 0) {
    if (DEBUG_SETUP) Serial.println("MPU6050 Wakeup SUCCESS.");
  } else {
    if (DEBUG_SETUP) Serial.println("MPU6050 Wakeup FAILED. Check Connections/Address.");
    while(1); // Halt
  }

  // Configure Gyro Full Scale Range to +/- 250 deg/s (default)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG register
  Wire.write(0x00); // FS_SEL=0 (+/- 250deg/s)
  Wire.endTransmission(true);

  // Configure Accelerometer Full Scale Range to +/- 2g (default)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0x00); // AFS_SEL=0 (+/- 2g)
  Wire.endTransmission(true);
  
  // Configure Interrupts: Data Ready Interrupt Enable
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x38); // INT_ENABLE register
  Wire.write(0x01); // Set bit 0 (DATA_RDY_EN) to 1
  Wire.endTransmission(true);

  // Configure Interrupt Pin Behavior (optional, but good practice)
  // Wire.beginTransmission(MPU_ADDR);
  // Wire.write(0x37); // INT_PIN_CFG register
  // Wire.write(0x00); // Default: INT active HIGH, push-pull, held until cleared
  // // Wire.write(0x10); // INT active LOW (if needed for your ISR trigger)
  // Wire.endTransmission(true);

  if (DEBUG_SETUP) Serial.println("MPU6050 Setup Complete.");
}

void readMPU6050Data() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Start with ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) {
    if (DEBUG_MPU_RAW || DEBUG_MPU_ANGLES) Serial.println("MPU Read: endTransmission failed");
    return; // Error in I2C communication
  }
  
  uint8_t bytesRead = Wire.requestFrom(MPU_ADDR, 14, true);
  if (bytesRead != 14) {
    if (DEBUG_MPU_RAW || DEBUG_MPU_ANGLES) {
      Serial.print("MPU Read: Incorrect byte count: "); Serial.println(bytesRead);
    }
    return; // Error, did not get all 14 bytes
  }

  // Accelerometer data (16384 LSB/g for +/-2g scale)
  accX_raw = (Wire.read() << 8 | Wire.read());
  accY_raw = (Wire.read() << 8 | Wire.read());
  accZ_raw = (Wire.read() << 8 | Wire.read());

  Wire.read(); Wire.read(); // Skip temperature

  // Gyroscope data (131 LSB/deg/s for +/-250deg/s scale)
  gyroX_raw = (Wire.read() << 8 | Wire.read());
  gyroY_raw = (Wire.read() << 8 | Wire.read());
  gyroZ_raw = (Wire.read() << 8 | Wire.read());

  if (DEBUG_MPU_RAW) {
    Serial.print("RAW: Ax="); Serial.print(accX_raw);
    Serial.print(" Ay="); Serial.print(accY_raw);
    Serial.print(" Az="); Serial.print(accZ_raw);
    Serial.print(" | Gx="); Serial.print(gyroX_raw);
    Serial.print(" Gy="); Serial.print(gyroY_raw);
    Serial.print(" Gz="); Serial.println(gyroZ_raw);
  }
}

void calculateAngles() {
  // Apply offsets and convert to physical units
  // Accelerometer: to 'g's
  float accX = (accX_raw - accX_offset) / 16384.0;
  float accY = (accY_raw - accY_offset) / 16384.0;
  float accZ = (accZ_raw - accZ_offset) / 16384.0;
  // Gyroscope: to 'deg/s'
  float gyroX = (gyroX_raw - gyroX_offset) / 131.0;
  // float gyroY = (gyroY_raw - gyroY_offset) / 131.0; // If using roll

  // Calculate angle from accelerometer (Pitch: rotation around X-axis, using AccY and AccZ)
  // This assumes MPU is mounted such that tilting forward/backward primarily changes AccY/AccZ,
  // and this tilt is around the MPU's X-axis.
  // If Z is near 0 (e.g. MPU on its side), this formula is unstable.
  // Make sure your "level" position has accZ close to 1g or -1g.
  angleAccX = atan2(accY, accZ) * RAD_TO_DEG; // atan2(Ay,Az) for pitch if X is fwd/back axis of robot

  // Integrate gyroscope data (gyroX is rate of rotation around X-axis)
  gyroAngleX_integrated += gyroX * dtMPU;

  // Complementary Filter for Pitch (balancing axis)
  currentAngleX = alpha * (gyroAngleX_integrated) + (1.0 - alpha) * angleAccX;
  
  // After filtering, update the gyro integrated angle to match the current filtered angle.
  // This prevents the gyro part from drifting away indefinitely if the filter gives it less weight.
  // It essentially "resets" the gyro integration baseline to the filtered angle.
  gyroAngleX_integrated = currentAngleX;


  if (DEBUG_MPU_ANGLES) {
    Serial.print("ANGLES: AccX="); Serial.print(angleAccX, 2);
    Serial.print(" GyroX_Integ="); Serial.print(gyroAngleX_integrated, 2); // This will now track currentAngleX
    Serial.print(" FiltX="); Serial.print(currentAngleX, 2);
    Serial.print(" dtMPU="); Serial.println(dtMPU, 4);
  }
}

//---------------------------------------------------------------------
// Motor Control Functions
//---------------------------------------------------------------------
void controlMotors(int motorSpeed) {
  // motorSpeed: positive -> move "forward" to correct backward tilt
  //             negative -> move "backward" to correct forward tilt

  bool goForward = motorSpeed > 0;
  int pwmValue = abs(motorSpeed);
  pwmValue = constrain(pwmValue, 0, 255); // Should be already constrained by PID

  if (DEBUG_MOTOR_COMMAND && (millis() % 500 < 20)) { // Print less frequently
     Serial.print("Motor Ctrl: Speed="); Serial.print(motorSpeed);
     Serial.print(" PWM="); Serial.print(pwmValue);
     Serial.print(" Dir="); Serial.println(goForward ? "FWD" : "BWD");
  }

  if (goForward) {
    // Move Robot Forward (adjust INx pins if motors spin wrong way for "forward")
    digitalWrite(IN1_PIN, HIGH); // Motor A Forward
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, HIGH); // Motor B Forward
    digitalWrite(IN4_PIN, LOW);
  } else {
    // Move Robot Backward
    digitalWrite(IN1_PIN, LOW);  // Motor A Backward
    digitalWrite(IN2_PIN, HIGH);
    digitalWrite(IN3_PIN, LOW);  // Motor B Backward
    digitalWrite(IN4_PIN, HIGH);
  }
  analogWrite(ENA_PIN, pwmValue);
  analogWrite(ENB_PIN, pwmValue);
}

void stopMotors() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(ENA_PIN, 0);
  analogWrite(ENB_PIN, 0);
  if (DEBUG_MOTOR_COMMAND) {
    // Serial.println("MOTORS STOPPED (called by function)");
  }
}