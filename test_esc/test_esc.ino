//#define low_pin 25
//#define high_pin 26

#define AH 13
#define AL 12
#define BH 14
#define BL 27
#define CH 26
#define CL 25


const int frequency = 10000;     // Set frequency to 5 KHz
const int resolution = 8;       // Set resolution to 8 bits (0-255)
int dutyCycle = 255;
int step_delay=10;
byte bldc_step = 0;
unsigned int i;
/*
void setup() {
  Serial.begin(115200);
  pinMode(high_pin, OUTPUT);
  pinMode(low_pin, OUTPUT);
  ledcAttach(high_pin, frequency, resolution);
  
  
}

void activate_high(){
 // Serial.println("High Side..");
  digitalWrite(low_pin, LOW);
  delayMicroseconds(50);
  ledcWrite(high_pin, dutyCycle);

}

void activate_low(){
  digitalWrite(high_pin, LOW);
  delayMicroseconds(50);
  digitalWrite(low_pin, HIGH);

}

*/

void setup() {
   Serial.begin(115200);
   
   pinMode(AH, OUTPUT);
   pinMode(AL, OUTPUT);
   pinMode(BH, OUTPUT);
   pinMode(BL, OUTPUT);
   pinMode(CH, OUTPUT);
   pinMode(CL, OUTPUT);

   ledcAttach(AH, frequency, resolution);
   ledcAttach(BH, frequency, resolution);
   ledcAttach(CH, frequency, resolution);



  
}



void AH_BL(){
 ledcWrite(CH, 0);

  digitalWrite(BL, LOW);
  delay(10);
  ledcWrite(AH, dutyCycle);


  
}

void AH_CL(){
  digitalWrite(BL, HIGH);
  
  digitalWrite(CL, LOW);
  delay(10);
  ledcWrite(AH, dutyCycle);

}

void BH_CL(){
 ledcWrite(AH, 0);

  digitalWrite(CL, LOW);
  delay(10);
  ledcWrite(BH, dutyCycle);

}

void BH_AL(){

  digitalWrite(CL, HIGH);
 
  digitalWrite(AL, LOW);
  delay(10);
  ledcWrite(BH, dutyCycle);

}

void CH_AL(){
  ledcWrite(BH, 0);
 
  digitalWrite(AL, LOW);
  delay(10);
  ledcWrite(CH, dutyCycle);

}

void CH_BL(){
  digitalWrite(AL, HIGH);
 
  digitalWrite(BL, LOW);
  delay(10);
  ledcWrite(CH, dutyCycle);

}


void bldc_move(){    
  //Serial.println(bldc_step);    // BLDC motor commutation function
  switch(bldc_step){
    
    case 0:

      AH_BL();
      break;
    case 1:

      AH_CL();
      break;
    case 2:

      BH_CL();
      break;
    case 3:

      BH_AL();
      break;
    case 4:

      CH_AL();
      break;
    case 5:

      CH_BL();
      break;
  }
}



void loop(){
  i = 10000;
  while(i > 100) {
    delayMicroseconds(i);
   
    bldc_move();
    //delay(50);
    Serial.println(i);
    bldc_step++;
    bldc_step %= 6;
    i = i - 20;
    //i--;
  }

}

/*
void loop(){
  
  AH_BL();
  delayMicroseconds(step_delay);
  AH_CL();
  delayMicroseconds(step_delay);

  BH_CL();
  delayMicroseconds(step_delay);
  BH_AL();
  delayMicroseconds(step_delay);

  CH_AL();
  delayMicroseconds(step_delay);
  CH_BL();
  delayMicroseconds(step_delay);

}
*/


/*
void loop(){
  digitalWrite(AL, LOW);
  delay(1000);
  digitalWrite(AL, HIGH);
  delay(1000);
  dutyCycle = 100;
  ledcWrite(AH, dutyCycle);
  delay(15000);
  dutyCycle = 0;
  ledcWrite(AH, dutyCycle);
  //digitalWrite(AH, LOW);
  delay(15000);
}
*/
/*
void loop() {
 activate_low();
 delay(step_delay);
 activate_high();
 delay(step_delay);
  
}
*/
