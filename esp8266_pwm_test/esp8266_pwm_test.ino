// Define the GPIO pin you want to use for PWM output
const int pwmPin = 23; 

// Define the PWM properties
const int pwmFreq = 30000;   // 30 kHz
const int pwmResolution = 8; // 8-bit resolution (0-255)
const int pwmChannel = 0;    // Use one of the 16 available LEDC channels (0-15)

void setup() {
  // Configure the LEDC timer for the specified frequency and resolution

}

void loop() {
  // Set the duty cycle to 50% (half of 255 for 8-bit resolution)
  int dutyCycle = 128; 

}
