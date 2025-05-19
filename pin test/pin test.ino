// Your L298N Pin Definitions:
const int ENA_PIN = 9;
const int IN1_PIN = 4;
const int IN2_PIN = 7;

const int ENB_PIN = 3;
const int IN3_PIN = 6;
const int IN4_PIN = 8;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Starting Pin-by-Pin L298N Input Test");

  // Set only ONE group of motor pins as output initially
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  // Keep ENB, IN3, IN4 as INPUT or comment out their pinMode for now

  // Ensure other pins are not accidentally driven if they are still OUTPUT
  // For pins not being tested, set them to INPUT to be safe if you don't want to control their state
  // pinMode(ENB_PIN, INPUT);
  // pinMode(IN3_PIN, INPUT);
  // pinMode(IN4_PIN, INPUT);
}

void loop() {
  Serial.println("\n--- Testing IN1_PIN (Pin 4) ---");
  Serial.println("Setting IN1_PIN HIGH");
  digitalWrite(IN1_PIN, HIGH);
  delay(2000);



  Serial.println("Setting IN1_PIN LOW");
  digitalWrite(IN1_PIN, LOW);
  delay(2000);



  Serial.println("\n--- Testing IN2_PIN (Pin 5) ---");
  Serial.println("Setting IN2_PIN HIGH");
  digitalWrite(IN2_PIN, HIGH);
  delay(2000);


  Serial.println("Setting IN2_PIN LOW");
  digitalWrite(IN2_PIN, LOW);
  delay(2000);



  Serial.println("\n--- Testing ENA_PIN (Pin 9) ---");
  Serial.println("Setting ENA_PIN PWM 180");
  analogWrite(ENA_PIN, 180);
  delay(2000);


  Serial.println("Setting ENA_PIN PWM 0");
  analogWrite(ENA_PIN, 0);
  delay(2000);


  Serial.println("\n--- Testing for IN3_PIN (Pin 6) ---");
  Serial.println("Setting IN3_PIN HIGH");
  digitalWrite(IN3_PIN, HIGH);
  delay(2000);

  Serial.println("Setting IN3_PIN LOW");
  digitalWrite(IN3_PIN, LOW);
  delay(2000);

  Serial.println("\n--- Testing IN4_PIN (Pin 8) ---");
  Serial.println("Setting IN4_PIN HIGH");
  digitalWrite(IN4_PIN, HIGH);
  delay(2000);

  Serial.println("Setting IN4_PIN LOW");
  digitalWrite(IN4_PIN, LOW);
  delay(2000);

  Serial.println("\n--- Testing ENB_PIN (Pin 3) ---");
  Serial.println("Setting ENB_PIN PWM 180");
  analogWrite(ENB_PIN, 180);
  delay(2000);

  Serial.println("Setting ENB_PIN PWM 0");
  analogWrite(ENB_PIN, 0);
  delay(2000);


  Serial.println("\n--- Test Cycle Complete for Motor A Pins ---");
  Serial.println("Waiting 5 seconds before repeating...");
  delay(5000);
}