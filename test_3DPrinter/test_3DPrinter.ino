// Define pin connections & motor's steps per revolution
const int dirPin = 55;
const int stepPin = 54;
const int enaPin = 38;

const int stepsPerRevolution = 200;

void setup() {
  // Declare pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enaPin, OUTPUT);
  //digitalWrite(enaPin, LOW);
  
}

void loop() {
  // Set motor direction clockwise
  digitalWrite(dirPin, HIGH);

  // Spin motor slowly
  for (int x = 0; x < stepsPerRevolution; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(40);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(40);
  }
  delay(1000);  // Wait a second

  // Set motor direction counterclockwise
  /*digitalWrite(dirPin, LOW);

  // Spin motor quickly
  for (int x = 0; x < stepsPerRevolution; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }
  delay(1000);  // Wait a second
  */
}