#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Variables to store angles
float roll_acc = 0.0;
float pitch_acc = 0.0;

// Gyroscope raw data (for Z-axis rotation rate)
float gyro_z = 0.0;

// For a simple complementary filter (optional, but recommended for better stability)
float gyro_x_rate = 0.0;
float gyro_y_rate = 0.0;
float angle_pitch = 0.0;
float angle_roll = 0.0;

unsigned long prev_time = 0;
float dt = 0.0;

// Complementary filter coefficient (adjust as needed, e.g., 0.98)
const float alpha = 0.98;


void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(10); // wait for serial port to connect. Needed for native USB
  }

  // Initialize MPU6050
  // Wire.begin() is implicitly called by mpu.begin()
  // For Arduino Nano, A4 is SDA, A5 is SCL
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Set accelerometer range (can be ACCEL_RANGE_2_G, ACCEL_RANGE_4_G, ACCEL_RANGE_8_G, ACCEL_RANGE_16_G)
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:  Serial.println("+-2G");   break;
    case MPU6050_RANGE_4_G:  Serial.println("+-4G");   break;
    case MPU6050_RANGE_8_G:  Serial.println("+-8G");   break;
    case MPU6050_RANGE_16_G: Serial.println("+-16G");  break;
  }

  // Set gyroscope range (can be GYRO_RANGE_250_DEG, GYRO_RANGE_500_DEG, GYRO_RANGE_1000_DEG, GYRO_RANGE_2000_DEG)
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:  Serial.println("+- 250 deg/s");  break;
    case MPU6050_RANGE_500_DEG:  Serial.println("+- 500 deg/s");  break;
    case MPU6050_RANGE_1000_DEG: Serial.println("+- 1000 deg/s"); break;
    case MPU6050_RANGE_2000_DEG: Serial.println("+- 2000 deg/s"); break;
  }

  // Set filter bandwidth (optional)
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ: Serial.println("260 Hz"); break;
    case MPU6050_BAND_184_HZ: Serial.println("184 Hz"); break;
    case MPU6050_BAND_94_HZ:  Serial.println("94 Hz");  break;
    case MPU6050_BAND_44_HZ:  Serial.println("44 Hz");  break;
    case MPU6050_BAND_21_HZ:  Serial.println("21 Hz");  break;
    case MPU6050_BAND_10_HZ:  Serial.println("10 Hz");  break;
    case MPU6050_BAND_5_HZ:   Serial.println("5 Hz");   break;
  }
  
  Serial.println("\nReading data...");
  delay(100);
  prev_time = millis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate dt (time difference)
  unsigned long current_time = millis();
  dt = (current_time - prev_time) / 1000.0; // Convert to seconds
  prev_time = current_time;

  // --- Calculate Roll and Pitch from Accelerometer ---
  // These are prone to noise from vibrations/acceleration
  // Roll: rotation around X-axis
  // Pitch: rotation around Y-axis
  // Note: MPU6050 axes might be different from your robot's axes.
  // Typically, for a board lying flat:
  // a.acceleration.x is acceleration along X
  // a.acceleration.y is acceleration along Y
  // a.acceleration.z is acceleration along Z (gravity when flat)

  // Roll calculation from accelerometer
  roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / M_PI;
  
  // Pitch calculation from accelerometer
  // Using sqrt(y^2 + z^2) in the denominator for pitch to avoid issues when roll is large
  pitch_acc = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / M_PI;

  // --- Get Gyroscope Data ---
  // g.gyro.x, g.gyro.y, g.gyro.z are angular velocities in rad/s
  // Convert to degrees/s
  gyro_x_rate = g.gyro.x * 180.0 / M_PI; // Roll rate
  gyro_y_rate = g.gyro.y * 180.0 / M_PI; // Pitch rate
  gyro_z = g.gyro.z * 180.0 / M_PI;      // Yaw rate

  // --- Complementary Filter for Roll and Pitch ---
  // This combines accelerometer (good for long-term, but noisy)
  // and gyroscope (good for short-term, but drifts)
  
  // Roll
  angle_roll = alpha * (angle_roll + gyro_x_rate * dt) + (1.0 - alpha) * roll_acc;
  
  // Pitch
  angle_pitch = alpha * (angle_pitch + gyro_y_rate * dt) + (1.0 - alpha) * pitch_acc;


  // --- Print the data ---
  Serial.print("Roll_Acc: "); Serial.print(roll_acc);
  Serial.print(" | Pitch_Acc: "); Serial.print(pitch_acc);
  
  Serial.print(" | GyroZ_Rate: "); Serial.print(gyro_z); // Yaw rate

  Serial.print(" | CF_Roll: "); Serial.print(angle_roll);
  Serial.print(" | CF_Pitch: "); Serial.print(angle_pitch);

  Serial.println();

  delay(10); // Adjust delay as needed for your application
}