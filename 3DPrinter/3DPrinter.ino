#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>

#define base_frequency 16000000
#define steps_pre_revolution 200
#define microstepping 16
#define screw_pitch 2
#define no_of_starts 4
#define clock_prescalar 1
#define timer_frequency base_frequency/clock_prescalar

double count=0.0;
long times=0;
double time_elapsed=0.0;

long steps_per_mm = (steps_pre_revolution * microstepping)/(screw_pitch * no_of_starts); // (200 *16)/(2*4) = 400
double mm_per_step = 1.00/(double)steps_per_mm; // 0.000625

volatile double time_increment = 0.0;

char *gcodes[50] = {
  //"G1 X10 Y0 E10 F1200",
  "G1 X4000 Y0 E10 F3000"
};

volatile long c=1,x=0,y=0,e=0,f=0;
volatile int prev_x=0;
volatile int prev_y=0;
volatile double duration=0;
volatile double distance=0;
volatile double travelled=0;
double slope = 0.0;
double dY = 0.0;
double dX = 0.0;
double p = 0.0;
int x_primary=0;

void delay_1us_nop() {
  // For a 16MHz clock, 1 us = 16 clock cycles.
  // Each NOP is 1 clock cycle.
  // Adjust the number of NOPs to account for the overhead of the function call itself, 
  // which might be 2-4 cycles.
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  //asm("nop"); asm("nop"); asm("nop"); asm("nop");
  //asm("nop"); asm("nop"); asm("nop"); asm("nop"); 
  // You might need slightly fewer NOPs to get exactly 1us due to function call overhead.
  // Fine-tuning with an oscilloscope is highly recommended.
}


ISR(TIMER1_COMPA_vect){
  time_elapsed = time_elapsed + time_increment;
  travelled=travelled+mm_per_step;
  if(travelled>distance){
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); 
    Serial.println(travelled);
    //Serial.println(time_elapsed);
  }else{
    if(x_primary){
      PORTF |= (1 << 0);
      delay_1us_nop();
      PORTF &= ~(1 << 0);
      //PORTF ^= (1<<PF0);
      
      //PORTF |= (1 << 1);
      //Serial.println("x");
      if(p<0) 
        p = p + 2 * dY;
      else{
        p = p + (2 * dY) - (2 * dX);
        PORTA |= (1 << 6);
        PORTA |= (1 << 7);
        //Serial.println("y");
      }
    }else{
       //Serial.println("y");
       PORTA |= (1 << 6);
       PORTA |= (1 << 7);
      if(p<0) 
        p = p + 2 * dX;
      else{
        p = p + (2 * dX) - (2 * dY);
        //PORTF ^= (1<<PF0);
        PORTF |= (1 << 0);
        delay_1us_nop();
        PORTF &= ~(1 << 0);
        //PORTF |= (1 << 1);
        //Serial.println("x");
      }     
    }

  }
} 

void set_duration(){
  distance = sqrt((x*x) + (y*y));
  duration = distance/f;
  dY = (y-prev_y);
  dX =  (x-prev_x);
  slope = (double)(y-prev_y)/(x-prev_x);
  Serial.println(slope);
  x_primary=abs(dX)>=abs(dY); 
  p = (2 * dY) -dX;
 }

void set_target_freq(){
  count =  round(timer_frequency/(steps_per_mm * f))-1;
  Serial.println(count);
  //time_increment = 1.0 / ((steps_per_mm) * f);
  time_increment = (1.0 / (timer_frequency)) * (count+1);
  //Serial.println(time_increment,DEC);
  unsigned char sreg;
  sreg = SREG;
  cli();
  OCR1A = (unsigned int)count;
  SREG = sreg;
  sei();
}




void parse_gcodes(){
  for(int i=0;i<1;i++){

    char temp_string[50];
    strcpy(temp_string, gcodes[i]);
    char *token;
    const char *delimiter = " ";
    token = strtok(temp_string, delimiter);

    while (token != NULL) {
      char type = token[0];
      char* val= token+1;

      switch(type){
        case 'G':
          c=atoi(val);
          break;
        case 'X':
          prev_x=x;
          x=atoi(val);
          break;
        case 'Y':
          prev_y=y;
          y=atoi(val);
          break;
        case 'E':
          e=atoi(val);
          break;
        case 'F':
          f=atoi(val)/60;
          break;
      }
      token = strtok(NULL, delimiter); // Get the next token
    }
    set_duration();  
    set_target_freq();
  }
}


void setup() {
  Serial.begin(115200);

  pinMode(38, OUTPUT);
  digitalWrite(38, LOW);
  
  //pinMode(54, OUTPUT);
  //pinMode(55, OUTPUT);
  
  DDRF |= (1 << 0); 
  DDRF |= (1 << 1);



  DDRA |= (1 << 6);
  DDRA |= (1 << 7);

  TCCR1A = 0;
  TCCR1B = (1<<WGM12) | (1<<CS10);
  TIMSK1=  (1<<OCIE1A);
  parse_gcodes();
  sei(); 
}

void loop() {
  //Serial.println(digitalRead(54));
  //delay(500);
}


