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

#define total_gcodes 6

double count=0.0;
long times=0;
double time_elapsed=0.0;

long steps_per_mm = (steps_pre_revolution * microstepping)/(screw_pitch * no_of_starts); // (200 *16)/(2*4) = 400
double mm_per_step = 1.00/(double)steps_per_mm; // 0.000625

volatile double time_increment = 0.0;


/*
 // Anti clockwise 0 - 90
char *gcodes[total_gcodes] = {
  "G01 X100 Y100 F1000",
  "G01 X150 Y100 F1000",
  "G02 X100 Y150 I-50 J0",
  
};
*/

 /*
 // Anti clockwise 90 - 180
char *gcodes[total_gcodes] = {
  "G01 X100 Y100 F1000",
  "G01 X100 Y150 F1000",
  "G02 X50 Y100 I0 J-50",
  
};
*/

/*
// Anti clockwise Semi circle 0 - 180

char *gcodes[total_gcodes] = {
  "G01 X100 Y100 F1000",
  "G01 X150 Y100 F1000",
  "G02 X100 Y150 I-50 J0",
  "G02 X50 Y100 I0 J-50",
};

*/

/*
// Anti clockwise 180 - 270
char *gcodes[total_gcodes] = {
  "G01 X100 Y100 F1000",
  "G01 X50 Y100 F1000",
  "G02 X100 Y50 I50 J0",
  
};

*/

/*
// Anti clockwise 270 - 360
char *gcodes[total_gcodes] = {
  "G01 X100 Y100 F1000",
  "G01 X100 Y50 F1000",
  "G02 X150 Y100 I0 J50",
  
};
*/

// Anti clockwise Full circle 0 - 360

char *gcodes[total_gcodes] = {
  "G01 X100 Y100 F1000",
  "G01 X150 Y100 F1000",
  "G02 X100 Y150 I-50 J0",
  "G02 X50 Y100 I0 J-50",
  "G02 X100 Y50 I50 J0",
  "G02 X150 Y100 I0 J50",
};


int c=1;
int gcode_indx=0,prev_gcode_indx=0;
volatile double x=0,y=0,e=0,f=0;
volatile double prev_x=0;
volatile double prev_y=0;
volatile double duration=0;
volatile double distance=0;
volatile double travelled=0;;
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
double theta;
int x_decrease=0;
int y_decrease=0;

long count2=0;


void delay_1us_nop() {
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
}


ISR(TIMER1_COMPA_vect){
  if(travelled>distance){
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); 
    if(gcode_indx<total_gcodes-1) gcode_indx++;
    Serial.println(travelled);
    Serial.println(p_x);
    Serial.println(p_y);
    //Serial.println(count2);
    
    travelled=0.0;
    ins_x=0.0;
    ins_y=0.0;
    PORTB |= (1 << 0);
  }else{
    //Serial.println(get_direction_x());
    travelled = travelled + mm_per_step;
    PORTB &= ~(1<<0);  
    if(linear_movement==1){
      ins_x = travelled/formula_denominator;
      ins_y = slope * ins_x;
      //Serial.println(ins_y,DEC);
      if(abs(ins_x)>mm_per_step){      
        if(abs(abs(ins_x) - p_x) >= mm_per_step){
          p_x = p_x + mm_per_step*get_direction_x();
          step_x();
        }
      }

      if(abs(ins_y)>mm_per_step){
        if(abs(abs(ins_y) - p_y) >= mm_per_step){
          p_y = p_y + mm_per_step*get_direction_y();
          step_y();
        }
      }
    }else{
      double arc_theta = travelled/radius;
      
      //step_theta = get_direction_x() < 0 ? step_theta:(theta-step_theta);
      //ins_x= arc_center_x+cos(arc_theta)*radius;
      if(get_direction_x() > 0 && get_direction_y() > 0) { // BOTH INCREASING
          //Serial.println(5);
          ins_x= arc_center_x + sin(arc_theta)*radius;
          ins_y= arc_center_y - cos(arc_theta)*radius;
      } else if(get_direction_x() < 0 && get_direction_y() > 0) { // X DECREASING Y INCREASING
          //Serial.println(6);
          ins_x= arc_center_x + cos(arc_theta)*radius;
          ins_y= arc_center_y + sin(arc_theta)*radius;
      } else if(get_direction_x() > 0 && get_direction_y() < 0) { // X INCREASING  Y DECREASING
          ins_x= arc_center_x - cos(arc_theta)*radius;
          ins_y= arc_center_y - sin(arc_theta)*radius;
          //Serial.println(7);
      }else if(get_direction_x() < 0 && get_direction_y() < 0){ // BOTH DECREASING
          ins_x= arc_center_x - sin(arc_theta)*radius;
          ins_y= arc_center_y + cos(arc_theta)*radius;
          //Serial.println(8);
      }
      
      
      //Serial.println(abs(abs(ins_y) - abs(p_y)),DEC);
      
      
      if(abs(abs(ins_x) - abs(p_x)) >= mm_per_step){
        p_x = p_x + (mm_per_step  * get_direction_x());
        step_x();
        
      }

      //step_theta = get_direction_y() > 0 ? (theta-step_theta):step_theta;
      //ins_y= arc_center_y + sin(arc_theta)*radius;
      //Serial.println(abs(abs(ins_y) - abs(p_y)),DEC);
      if(abs(abs(ins_y) - abs(p_y)) >= mm_per_step){
        p_y = p_y + (mm_per_step * get_direction_y());
        step_y();
        
      }
      
    }
  }
} 

void step_x(){
    PORTD |= (1 << 2);
    delay_1us_nop();
    PORTD &= ~(1 << 2);
}

void step_y(){
    PORTD |= (1 << 3);
    delay_1us_nop();
    PORTD &= ~(1 << 3);
}

void set_duration(){
  
  if(linear_movement){
    double c_x = x - p_x;
    double c_y = y - p_y;
    distance = sqrt((c_x * c_x) + (c_y * c_y));
    duration = distance/f;
    dY = (y-prev_y);
    dX =  (x-prev_x)==0?0.00001:(x-prev_x);
    slope = ((double)dY/dX);
    formula_denominator =sqrt(1+slope*slope);
    //Serial.println(slope);
  }else{
    //Serial.println(linear_movement);
    arc_center_x = I + prev_x;
    arc_center_y = J + prev_y;
    //Serial.println(arc_center_x);
    //Serial.println(arc_center_y);
    radius = sqrt(pow(prev_x - arc_center_x,2) + pow(prev_y - arc_center_y,2));  
    
    chord_length = sqrt(pow(prev_x-x,2) + pow(prev_y-y,2));
    
    theta = (asin(chord_length/(2*radius)))*2;
    //Serial.println(theta);
    
    arc_length = theta * radius;
    //Serial.println(arc_length);
    distance = arc_length;
    
    set_direction();
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
  if((x - p_x) < 0){ 
    set_anti_clockwise_for_x();
  }else if((x - p_x) > 0){
    set_clockwise_for_x();
  }
  if((y-p_y)<0){ 
    set_clockwise_for_y();
  }else if((y - p_y) > 0){
    set_anti_clockwise_for_y();
  }
}

int get_direction_x(){
  double inc = x - p_x;
  //Serial.println(inc);
  if(inc < 0)
    return -1;
  if(inc > 0)
    return 1;
  else
    return 0;
  //return (inc < mm_per_step) ? -1 : (inc > mm_per_step) ? 1 : 0;
}

int get_direction_y(){
  double inc = y - p_y;
  if(inc < 0)
    return -1;
  if(inc > 0)
    return 1;
  else
    return 0;
  //return (inc < mm_per_step) ? -1 : (inc > mm_per_step) ? 1 : 0;
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


