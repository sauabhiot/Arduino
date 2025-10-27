const int ledPin = 26;
const int frequency = 10000;
const int resolution = 8; // For 8-bit, duty cycle is 0-255

void setup() {
  // Attach the PWM configuration to the pin in a single function
  ledcAttach(ledPin, frequency, resolution);
}

void loop() {
  // Write the duty cycle directly to the pin
  ledcWrite(ledPin, 128); // 50% duty cycle
}
