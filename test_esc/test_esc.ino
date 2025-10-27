#define low_pin 25
#define high_pin 26

const int frequency = 5000;     // Set frequency to 5 KHz
const int resolution = 8;       // Set resolution to 8 bits (0-255)
int dutyCycle = 0;
int step_delay=10;


void setup() {
  Serial.begin(115200);
  pinMode(high_pin, OUTPUT);
  pinMode(low_pin, OUTPUT);
  ledcAttach(high_pin, frequency, resolution);
  
  
}

void activate_high(){
  digitalWrite(low_pin, LOW);
  delayMicroseconds(50);
  ledcWrite(high_pin, dutyCycle);

}

void activate_low(){
  digitalWrite(high_pin, LOW);
  delayMicroseconds(50);
  digitalWrite(low_pin, HIGH);

}

void loop() {
 activate_low();
 delay(step_delay);
 activate_high();
 delay(step_delay);
  
}

