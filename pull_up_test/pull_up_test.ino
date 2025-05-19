#include <Wire.h>

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000); // Wait for Serial Monitor
  Serial.println("I2C Pull-up Test - Initializing Wire library...");

  Wire.begin(); // Initialize I2C, enables internal pull-ups (weak)
                // and configures A4/A5 for I2C.

  Serial.println("Wire library initialized. A4 (SDA) and A5 (SCL) should be HIGH if pull-ups are working.");
  Serial.println("Measure voltage on A4 and A5 with respect to GND.");
}

void loop() {
  // Do nothing, just keep the I2C bus idle
  delay(1000);
}