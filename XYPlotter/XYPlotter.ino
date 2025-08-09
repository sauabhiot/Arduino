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

#define total_gcodes 1

double count=0.0;
long times=0;
double time_elapsed=0.0;

long steps_per_mm = (steps_pre_revolution * microstepping)/(screw_pitch * no_of_starts); // (200 *16)/(2*4) = 400
double mm_per_step = 1.00/(double)steps_per_mm; // 0.000625

volatile double time_increment = 0.0;

char *gcodes[total_gcodes] = {
  "G01 X0 Y50 F1000",
  //"G02 X0 Y50 I0 J-50",
  //"G02 X-50 Y0 I50 J0"
};
int c=1;
int gcode_indx=0,prev_gcode_indx=0;
volatile double x=0,y=0,e=0,f=0;
volatile double prev_x=0;
volatile double prev_y=0;
volatile double duration=0;
volatile double distance=0;
volatile double travelled=0;
double slope = 0.0;
double dY = 0.0;
double dX = 0.0;
double p = 0.0;
int x_primary=0;

double ins_x=0, ins_y=0;
double p_x = 0,p_y = 0;

double formula_denominator;
int linear_movement=1;

double chord_length=0.0;
double arc_length=0.0;
double radius=0.0;
double I=0.0,J=0.0;
double arc_center_x=0;
double arc_center_y=0;
long count2=0;


void delay_1us_nop() {
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
}


ISR(TIMER1_COMPA_vect){
  //time_elapsed = time_elapsed + time_increment;
  travelled=travelled+mm_per_step;
  if(travelled>distance){
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); 
    if(gcode_indx<total_gcodes-1) gcode_indx++;
    Serial.println(travelled);
    Serial.println(p_x);
    Serial.println(p_y);
    //Serial.println(count2);
    travelled=0;
    ins_x=0;
    ins_y=0;
    PORTB |= (1 << 0);
  }else{
    PORTB &= ~(1<<0);  
    if(linear_movement==1){
      //Serial.println("linear..");
      ins_x = travelled/formula_denominator;
      ins_y = slope * ins_x;
      //PORTD |= (1 << 5);
      if(abs(ins_x - p_x) >= mm_per_step){
        p_x = p_x + mm_per_step;
        PORTD |= (1 << 2);
        delay_1us_nop();
        PORTD &= ~(1 << 2);
        //Serial.println("x");
        //count2++;
      }
      if(abs(ins_y - p_y) >= mm_per_step){
        p_y = p_y + mm_per_step;
        PORTD |= (1 << 3);
        delay_1us_nop();
        PORTD &= ~(1 << 3);
        //Serial.println("y");
      }
    }else{
      double theta = travelled/radius;
      ins_x= cos(theta)*radius;
      ins_y= sin(theta)*radius;
      
      if((p_x - ins_x) >= mm_per_step){
        p_x = p_x - mm_per_step;
        //Serial.println(ins_x);
        //set_clockwise_for_x(); // TODO DIR is HARDCODED
        set_direction();
        PORTD |= (1 << 2);
        delay_1us_nop();
        PORTD &= ~(1 << 2);
      }
      if(abs(ins_y - p_y) >= mm_per_step){
        p_y = p_y + mm_per_step;
        //set_anti_clockwise_for_y(); // TODO DIR is HARDCODED
        set_direction();
        PORTD |= (1 << 3);
        delay_1us_nop();
        PORTD &= ~(1 << 3);
      }
    }
  }
} 

void set_duration(){
  
  if(linear_movement){
    //Serial.println(linear_movement);
    distance = sqrt((x*x) + (y*y));
    duration = distance/f;
    dY = (y-prev_y);
    dX =  (x-prev_x)==0?0.00001:(x-prev_x);
    slope = ((double)dY/dX);
    formula_denominator =sqrt(1+slope*slope);
  }else{
    //Serial.println(linear_movement);
    arc_center_x = I + prev_x;
    arc_center_y = J + prev_y;
    radius = sqrt(pow(prev_x - arc_center_x,2) + pow(prev_y - arc_center_y,2));  
    chord_length = sqrt(pow(prev_x-x,2) + pow(prev_y-y,2));
    double theta = (asin(chord_length/(2*radius)))*2;
    arc_length = theta * radius;
    distance = arc_length;
    //Serial.println(duration);
  }
  //Serial.println(duration);
}

void set_target_freq(){
  count =  round(timer_frequency/(steps_per_mm * f))-1;
  //Serial.println(count);
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

void set_clockwise_for_x(){
  PORTD |= (1 << 5); //X axis dir
}

void set_anti_clockwise_for_x(){
  PORTD &= ~(1 << 5); //X axis dir
}

void set_clockwise_for_y(){
  PORTD |= (1 << 6); //X axis dir
}

void set_anti_clockwise_for_y(){
  PORTD &= ~(1 << 6); //Y axis dir
}


void set_direction(){
  if(x<p_x){ 
    set_anti_clockwise_for_x();
  }else{
    set_clockwise_for_x();
  }
  if(y<p_y){ 
    set_clockwise_for_y();
  }else{
    set_anti_clockwise_for_y();
  }
}


void parse_gcodes(int indx){
  char temp_string[50];
  strcpy(temp_string, gcodes[indx]);
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
      case 'I':
        I=atoi(val);
        break;
      case 'J':
        J=atoi(val);
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

  linear_movement = (c==1);
  set_duration();  
  set_target_freq();
  set_direction(); 
}


void setup() {
  Serial.begin(115200);
  //delay(1000);
  DDRD |= (1<<2);  // pin 2 step pin for X axis
  DDRD |= (1<<5); // pin 5 dir pin for X axis

  DDRD |= (1<<3);  // pin 3 step for Y axis
  DDRD |= (1<<6); // pin 6 dir for Y axis
  
  DDRB |= (1<<0); // pin 8 enable
  PORTB &= ~(1<<0);  


  //DISABLED FOR TESTING, remove to start testing
  //PORTB |= (1 << 0);
  
  
  TCCR1A = 0;
  TCCR1B = (1<<WGM12) | (1<<CS10);
  TIMSK1=  (1<<OCIE1A);
  parse_gcodes(0);
  //sei(); 
}

void loop() {
  //Serial.println(travelled);
  if(gcode_indx>prev_gcode_indx){
    parse_gcodes(gcode_indx);
    prev_gcode_indx=gcode_indx;
    TCCR1B = (1<<WGM12) | (1<<CS10);
  }
}


