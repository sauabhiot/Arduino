#define high_side_phase_A_pwm 11
#define low_side_phase_A 5

#define high_side_phase_B_pwm 10
#define low_side_phase_B 4

#define high_side_phase_C_pwm 9
#define low_side_phase_C 3



const int freq = 1000;
const int resolution = 8;
const int dutyCycle = 255; // Example PWM duty cycle (0-255 for 8-bit resolution)
const int stepDelay = 1000;  // Milliseconds to delay between each step for speed control




void setup() {
Serial.begin(115200);
  pinMode(high_side_phase_A_pwm, OUTPUT);
  pinMode(high_side_phase_B_pwm, OUTPUT);
  pinMode(high_side_phase_C_pwm, OUTPUT);
  pinMode(low_side_phase_A, OUTPUT);
  pinMode(low_side_phase_B, OUTPUT);
  pinMode(low_side_phase_C, OUTPUT);
}

void AH_BL(){
  digitalWrite(high_side_phase_B_pwm, LOW);
  //digitalWrite(high_side_phase_C_pwm, LOW);
  digitalWrite(low_side_phase_A, LOW);
  //digitalWrite(low_side_phase_C, LOW);
  analogWrite(high_side_phase_A_pwm, dutyCycle);
  digitalWrite(low_side_phase_B, HIGH);
}

void AH_CL(){
  //digitalWrite(high_side_phase_B_pwm, LOW);
  digitalWrite(high_side_phase_C_pwm, LOW);
  digitalWrite(low_side_phase_A, LOW);
  //digitalWrite(low_side_phase_B, LOW);
  analogWrite(high_side_phase_A_pwm, dutyCycle);
  digitalWrite(low_side_phase_C, HIGH);
}

void BH_CL(){
  //digitalWrite(high_side_phase_A_pwm, LOW);
  digitalWrite(high_side_phase_C_pwm, LOW);
  //digitalWrite(low_side_phase_A, LOW);
  digitalWrite(low_side_phase_B, LOW);
  analogWrite(high_side_phase_B_pwm, dutyCycle);
  digitalWrite(low_side_phase_C, HIGH);
}

void BH_AL(){
  digitalWrite(high_side_phase_A_pwm, LOW);
  //digitalWrite(high_side_phase_C_pwm, LOW);
  digitalWrite(low_side_phase_B, LOW);
  //digitalWrite(low_side_phase_C, LOW);
  analogWrite(high_side_phase_B_pwm, dutyCycle);
  digitalWrite(low_side_phase_A, HIGH);
}

void CH_AL(){
  digitalWrite(high_side_phase_A_pwm, LOW);
  //digitalWrite(high_side_phase_B_pwm, LOW);
  //digitalWrite(low_side_phase_B, LOW);
  digitalWrite(low_side_phase_C, LOW);  
  analogWrite(high_side_phase_C_pwm, dutyCycle);
  digitalWrite(low_side_phase_A, HIGH);
}

void CH_BL(){
  //digitalWrite(high_side_phase_A_pwm, LOW);
  digitalWrite(high_side_phase_B_pwm, LOW);
  //digitalWrite(low_side_phase_A, LOW);
  digitalWrite(low_side_phase_C, LOW);  
  analogWrite(high_side_phase_C_pwm, dutyCycle);
  digitalWrite(low_side_phase_B, HIGH);
}


void loop() {
  AH_BL();
  delay(stepDelay);

  AH_CL();
  delay(stepDelay);
  
  BH_CL();
  delay(stepDelay);
  
  BH_AL();
  delay(stepDelay);
  
  CH_AL();
  delay(stepDelay);
  
  CH_BL();
  delay(stepDelay);
}

