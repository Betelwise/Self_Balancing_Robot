
// 22:13:44.775 -> accX_offset: -1294.02
// 22:13:44.779 -> accY_offset: -476.89
// 22:13:44.810 -> accZ_offset: 17059.09
// 22:13:44.810 -> gyroX_offset: 141.05
// 22:13:44.842 -> gyroY_offset: 192.49
// 22:13:44.874 -> gyroZ_offset: 4.43

// --- Global variables for offsets (you already have these) ---
 #include <Wire.h>
bool DEBUG_SETUP = true;
const int MPU_ADDR = 0x68; // MPU6050 I2C address
// float accX_offset = 0.0;
// ...
// float gyroZ_offset = 0.0;
float accX_offset = 0.0;
float accY_offset = 0.0;
float accZ_offset = 0.0; // Usually, Z offset isn't as critical for angle if it's near 1g
float gyroX_offset = 0.0;
float gyroY_offset = 0.0;
float gyroZ_offset = 0.0;

void setup() {
  Serial.begin(9600);
  // ...
  Wire.begin();
  setupMPU6050(); // Basic MPU register setup
  for (int i = 0; i < 10; i++) {
    delay(1000); // Allow MPU to stabilize
    Serial.print("Calibrating MPU6050... ");
    Serial.println(i + 1);
  }
  calibrateMPU6050(); // <<--- ADD THIS

  // ... rest of your setup (interrupts, motors, etc.)
  // ... ensure prevTimeMPU is initialized AFTER calibration if it uses raw reads
}

void loop(){
  delay(100);
}
// --- Call this function in your setup() BEFORE you start using the MPU for balancing ---
void calibrateMPU6050() {
  const int numSamples = 2000; // Number of samples to average
  long accX_sum = 0;
  long accY_sum = 0;
  long accZ_sum = 0;
  long gyroX_sum = 0;
  long gyroY_sum = 0;
  long gyroZ_sum = 0;

  if (DEBUG_SETUP) Serial.println("Starting MPU6050 Calibration. Keep it VERY STILL and LEVEL...");

  // Ensure MPU is awake and configured before calibration
  // (Your existing setupMPU6050() should have done this)

  // Discard first few readings to let sensor stabilize
  for (int i = 0; i < 100; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // Start with ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);
    for(int k=0; k<14; k++) Wire.read(); // Discard data
    delay(2);
  }

  if (DEBUG_SETUP) Serial.println("Collecting samples...");
  for (int i = 0; i < numSamples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // Start with ACCEL_XOUT_H
    Wire.endTransmission(false); // Send a restart message after master ACK
    
    uint8_t bytesRead = Wire.requestFrom(MPU_ADDR, 14, true); // Read 14 registers
    if (bytesRead == 14) {
      accX_sum += (int16_t)(Wire.read() << 8 | Wire.read());
      accY_sum += (int16_t)(Wire.read() << 8 | Wire.read());
      accZ_sum += (int16_t)(Wire.read() << 8 | Wire.read());
      Wire.read(); Wire.read(); // Skip temperature
      gyroX_sum += (int16_t)(Wire.read() << 8 | Wire.read());
      gyroY_sum += (int16_t)(Wire.read() << 8 | Wire.read());
      gyroZ_sum += (int16_t)(Wire.read() << 8 | Wire.read());
    } else {
      if (DEBUG_SETUP) Serial.println("Calibration: Error reading MPU data!");
      // Handle error, maybe retry or abort calibration
    }
    delay(2); // Small delay between readings
  }

  accX_offset = (float)accX_sum / numSamples;
  accY_offset = (float)accY_sum / numSamples;
  // For accZ, we want it to read 1g (or -1g) when level.
  // The raw value for 1g at +/-2g scale is 16384.
  // So, offset should be (sum/samples) - 16384 (or +16384 if Z is upside down)
  // However, it's often simpler to just offset X and Y to zero,
  // and let Z be what it is, as atan2 handles varying Z magnitudes.
  // For simplicity, let's just make accZ_offset the average reading too, aiming to make it 0.
  // Better: Calibrate Z so that when level, (accZ_raw - accZ_offset) / 16384.0 = 1.0 (or -1.0)
  // For now, simple averaging:
  accZ_offset = (float)accZ_sum / numSamples;
  // More accurate Z offset: (assuming Z should be +1g when level for your orientation)
  // accZ_offset = ((float)accZ_sum / numSamples) - 16384.0;
  // Or if Z should be -1g:
  // accZ_offset = ((float)accZ_sum / numSamples) + 16384.0;
  // Let's stick with simple mean for now and see:
  // accZ_offset = (float)accZ_sum / numSamples; // Aims to make accZ_raw - offset = 0 when still, which is not ideal for angle calc.
                                              // Better to leave accZ_offset = 0 if you don't do the 1g adjustment.

  gyroX_offset = (float)gyroX_sum / numSamples;
  gyroY_offset = (float)gyroY_sum / numSamples;
  gyroZ_offset = (float)gyroZ_sum / numSamples;

  if (DEBUG_SETUP) {
    Serial.println("--- Calibration Complete ---");
    Serial.print("accX_offset: "); Serial.println(accX_offset);
    Serial.print("accY_offset: "); Serial.println(accY_offset);
    Serial.print("accZ_offset: "); Serial.println(accZ_offset); // Note on Z-offset use
    Serial.print("gyroX_offset: "); Serial.println(gyroX_offset);
    Serial.print("gyroY_offset: "); Serial.println(gyroY_offset);
    Serial.print("gyroZ_offset: "); Serial.println(gyroZ_offset);
    Serial.println("---------------------------");
  }
}

void setupMPU6050() {
  if (DEBUG_SETUP) //Serial.println("Setting up MPU6050...");
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00); // Wake up MPU6050
  if (Wire.endTransmission(true) == 0) {
    if (DEBUG_SETUP){
     //Serial.println("MPU6050 Wakeup SUCCESS.");
    }
  } else {
    if (DEBUG_SETUP) //Serial.println("MPU6050 Wakeup FAILED. Check Connections/Address.");
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

  if (DEBUG_SETUP){ 
  //Serial.println("MPU6050 Setup Complete.");
  }
}