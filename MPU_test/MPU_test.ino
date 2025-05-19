#include <Wire.h>

void setup() {
  Wire.begin(); // Initialize I2C bus

  Serial.begin(115200);
  while (!Serial && millis() < 2000); // Wait for Serial Monitor to open

  Serial.println("\nI2C Scanner by Arduino");
  Serial.println("Scanning for I2C devices...");
}

void loop() {
  byte error, address;
  int nDevices;

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    // The I2C scanner uses the return value of
    // Wire.endTransmission() to see if a device did acknowledge
    // to the address.
     Serial.println("beginning wire");
    Wire.beginTransmission(address);
    Serial.println("wire ok");
    error = Wire.endTransmission();
    Serial.println("wire error end");

    if (error == 0) { // Device acknowledged
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0"); // Add leading zero for single-digit hex
      }
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    } else if (error == 4) { // Other error
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
    // error == 2: NACK on transmit of address (no device there)
    // error == 3: NACK on transmit of data (shouldn't happen here as we send no data)
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("Scan complete.\n");
  }

  delay(5000); // Wait 5 seconds before scanning again
}