
/*
X	54	55	38
Y	60	61	56
Z	46	48	62
E0	26	28	24
E1	36	34	30
*/

// Define the pins for the stepper motor (e.g., X-axis pins on RAMPS 1.4)
#define STEP_PIN 46 // Z_STEP_PIN
#define DIR_PIN 48  // Z_DIR_PIN
#define ENABLE_PIN 62 // Z_ENABLE_PIN

void setup() {
  // Set pin modes
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  // Enable the stepper driver (LOW to enable for DRV8825)
  digitalWrite(ENABLE_PIN, LOW);

  Serial.begin(115200); // Initialize serial communication for debugging
}

void loop() {
  // Rotate in one direction
  digitalWrite(DIR_PIN, HIGH); // Set direction (HIGH or LOW for clockwise/counter-clockwise)
  Serial.println("Moving in one direction...");
  for (int i = 0; i < 1600; i++) { // 1600 steps for a common NEMA 17 with 1/16 microstepping
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(500); // Adjust delay for speed
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(500);
  }
  delay(1000); // Pause for 1 second

  // Rotate in the opposite direction
  digitalWrite(DIR_PIN, LOW); // Change direction
  Serial.println("Moving in the opposite direction...");
  for (int i = 0; i < 1600; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(500);
  }
  delay(1000); // Pause for 1 second
}