// ...existing code...
// L298N Motor Driver Pin Definitions
// Motor A (Left Motor)
const int ENA_PIN = 9;
const int IN1_PIN = 4;
const int IN2_PIN = 7;

const int ENB_PIN = 3;
const int IN3_PIN = 6;
const int IN4_PIN = 8;

// Test Speeds
const int TEST_SPEED_LOW = 100;  // Low speed for testing (0-255)
const int TEST_SPEED_MED = 180; // Medium speed
const int TEST_SPEED_HIGH = 255; // Full speed

void setupMotorTest() {
  // Set all motor control pins as outputs
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  // Initialize Serial for debugging messages
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.println("L298N Motor Driver Test");
  Serial.println("Ensure motor power supply is connected to L298N.");
  Serial.println("Motors will run through a test sequence.");

  // Ensure motors are stopped initially
  stopAllMotorsTest();
  delay(1000); // Wait a bit before starting
}

// Function to control Motor A
// direction: 1 for forward, -1 for backward, 0 for stop
void controlMotorA(int direction, int speed) {
  if (direction == 1) { // Forward
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, speed);
    Serial.print("Motor A Forward, Speed: "); Serial.println(speed);
  } else if (direction == -1) { // Backward
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    analogWrite(ENA_PIN, speed);
    Serial.print("Motor A Backward, Speed: "); Serial.println(speed);
  } else { // Stop
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, 0);
    Serial.println("Motor A Stop");
  }
}

// Function to control Motor B
// direction: 1 for forward, -1 for backward, 0 for stop
void controlMotorB(int direction, int speed) {
  if (direction == 1) { // Forward
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    analogWrite(ENB_PIN, speed);
    Serial.print("Motor B Forward, Speed: "); Serial.println(speed);
  } else if (direction == -1) { // Backward
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    analogWrite(ENB_PIN, speed);
    Serial.print("Motor B Backward, Speed: "); Serial.println(speed);
  } else { // Stop
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
    analogWrite(ENB_PIN, 0);
    Serial.println("Motor B Stop");
  }
}

void stopAllMotorsTest() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(ENA_PIN, 0);
  analogWrite(ENB_PIN, 0);
  Serial.println("All Motors Stopped");
}

void loopMotorTest() {
  Serial.println("\n--- Testing Motor A ---");
  controlMotorA(1, TEST_SPEED_MED); // Motor A Forward Medium
  delay(2000);
  controlMotorA(-1, TEST_SPEED_MED); // Motor A Backward Medium
  delay(2000);
  controlMotorA(0, 0); // Stop Motor A
  delay(1000);

  Serial.println("\n--- Testing Motor B ---");
  controlMotorB(1, TEST_SPEED_MED); // Motor B Forward Medium
  delay(2000);
  controlMotorB(-1, TEST_SPEED_MED); // Motor B Backward Medium
  delay(2000);
  controlMotorB(0, 0); // Stop Motor B
  delay(1000);

  Serial.println("\n--- Testing Both Motors Forward ---");
  controlMotorA(1, TEST_SPEED_LOW);
  controlMotorB(1, TEST_SPEED_LOW);
  delay(2000);
  Serial.println("Increasing speed...");
  controlMotorA(1, TEST_SPEED_HIGH);
  controlMotorB(1, TEST_SPEED_HIGH);
  delay(2000);
  stopAllMotorsTest();
  delay(1000);

  Serial.println("\n--- Testing Both Motors Backward ---");
  controlMotorA(-1, TEST_SPEED_LOW);
  controlMotorB(-1, TEST_SPEED_LOW);
  delay(2000);
  Serial.println("Increasing speed...");
  controlMotorA(-1, TEST_SPEED_HIGH);
  controlMotorB(-1, TEST_SPEED_HIGH);
  delay(2000);
  stopAllMotorsTest();
  delay(1000);
  
  Serial.println("\n--- Testing Opposite Directions (Turning) ---");
  controlMotorA(1, TEST_SPEED_MED);    // Motor A Forward
  controlMotorB(-1, TEST_SPEED_MED);   // Motor B Backward
  delay(2000);
  stopAllMotorsTest();
  delay(500);
  controlMotorA(-1, TEST_SPEED_MED);   // Motor A Backward
  controlMotorB(1, TEST_SPEED_MED);    // Motor B Forward
  delay(2000);
  stopAllMotorsTest();
  delay(1000);

  Serial.println("\nTest sequence complete. Restarting in 5 seconds...");
  delay(5000);
}


// To use this test, you can replace your existing setup() and loop()
// with setupMotorTest() and loopMotorTest() like this:

void setup() {
  setupMotorTest();
}

void loop() {
  loopMotorTest();
}

// Make sure to comment out or remove your original setup() and loop()
// functions when using this test.

// ...existing code...