/*

ENDSTOP X-MIN	3
ENDSTOP X-MAX	2

ENDSTOP Y-MIN	14
ENDSTOP Y-MAX	15

ENDSTOP Z-MIN	18
ENDSTOP Z-MAX	19
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>
#include <SD.h>

#define FAST_MOVE 5000/60
#define base_frequency 16000000
#define steps_pre_revolution 200
#define microstepping 16
#define screw_pitch 2
#define no_of_starts 4
#define clock_prescalar 1
#define timer_frequency base_frequency/clock_prescalar

#define total_gcodes 100

#define ENABLE_Z_MOTOR_PIN 62 

#define GCODE_BUFFER_LENGTH 5

const int chipSelect = 53;

int homing = 0;

volatile int previous_batch_gcode_indx = 0;

int is_linear_motion = 1;
volatile int is_gcode_motion_command = 0 ;
double count=0.0;
long times=0;
double time_elapsed=0.0;
double x_distance_ratio, y_distance_ratio, z_distance_ratio, z_circular_distance_ratio;

long steps_per_mm = (steps_pre_revolution * microstepping)/(screw_pitch * no_of_starts); // (200 *16)/(2*4) = 400
double mm_per_step = 1.00/(double)steps_per_mm; // 0.000625

volatile double time_increment = 0.0;

File gcode_file;
int is_gcode_file_closed = 0;



char *gcodes[GCODE_BUFFER_LENGTH];


/*
char *gcodes[total_gcodes] = {
  "G01 X100 Y100 F500",
  "G01 X149.9 Y99.9",
  "G02 X149.9 Y100.1 Z1 I-49 J1",
};
*/

int c=1;
int gcode_indx = 0 , prev_gcode_indx = 0;
int end_gcode_sub_indx = 0 , gcode_sub_indx = 0 , prev_gcode_sub_indx = 0;

volatile double x=0,y=0,z=0,e=0,f=10;

volatile double prev_x=0;
volatile double prev_y=0;
volatile double prev_z=0;
volatile double duration = 0, duration_z = 0;
volatile double distance = 0, distance_z = 0;
volatile double travelled=0, travelled_x = 0, travelled_y = 0,travelled_z=0;
double slope = 0.0;
double dY = 0.0;
double dX = 0.0;
double p = 0.0;
int x_primary=0;

double ins_x = 0, ins_y = 0, ins_z = 0;
double p_x = 0, p_y = 0, p_z = 0;
double delta_x = 0.0, delta_y = 0.0, delta_z = 0.0;
double gcode_step_x = 0, gcode_step_y = 0;

double formula_denominator;
int g_code=1;

double chord_length=0.0;
double arc_length=0.0;
double radius=0.0;
double I=0.0,J=0.0;
double arc_center_x=0;
double arc_center_y=0;
double theta;
int x_previouse=0;
int y_previous=0;

long count2=0;

int current_quadrant = 1, start_quadrant = 0, end_quadrant = 0;

int clockwise = 0;

double q1_x1_limit = 0;
double q1_y1_limit = 0;

double q1_x2_limit = 0;
double q1_y2_limit = 0;

double q2_x1_limit = 0;
double q2_y1_limit = 0;

double q2_x2_limit = 0;
double q2_y2_limit = 0;


double q3_x1_limit = 0;
double q3_y1_limit = 0;

double q3_x2_limit = 0;
double q3_y2_limit = 0;

double q4_x1_limit = 0;
double q4_y1_limit = 0;

double q4_x2_limit = 0;
double q4_y2_limit = 0;

double intermediate_x = 0.0, intermediate_y = 0.0;

volatile void delay_1us_nop() {
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop");

}


ISR(TIMER1_COMPA_vect){
   
  if(travelled > distance){
    if(end_gcode_sub_indx == gcode_sub_indx) {
      
      gcode_indx++;
      //Serial.println(gcode_indx);
      //process_next_gcode = 1;
    }
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); 
    //Serial.println(travelled,DEC);
    //Serial.println(p_x,DEC);
    //Serial.println(p_y,DEC);
    //Serial.println(p_z,DEC);
    travelled = 0.0;
    travelled_x = 0.0;
    travelled_y = 0.0;
    travelled_z = 0.0;
    PORTD |= (1 << 7);
    PORTF |= (1 << 2);
    digitalWrite(ENABLE_Z_MOTOR_PIN, HIGH);
   
    if(!is_linear_motion) next_quadrant();
  }else{
    travelled = travelled + mm_per_step;
    PORTD &= ~(1 << 7);
    PORTF &= ~(1 << 2);
    //Serial.println("called..");
    if(is_linear_motion){
      digitalWrite(ENABLE_Z_MOTOR_PIN, LOW);
      delta_x = (travelled * x_distance_ratio) - travelled_x;
      delta_y = (travelled * y_distance_ratio) - travelled_y;
      delta_z = (travelled * z_distance_ratio) - travelled_z;

      if(abs(delta_x) > mm_per_step){
        p_x = p_x + mm_per_step * get_direction_x();
        travelled_x = travelled_x + mm_per_step;
        step_x();
      }
      if(abs(delta_y) > mm_per_step){
        p_y = p_y + mm_per_step * get_direction_y();
        travelled_y = travelled_y + mm_per_step;
        step_y();
      }
      if(abs(delta_z) > mm_per_step){
        p_z = p_z + mm_per_step * get_direction_z();
        travelled_z = travelled_z + mm_per_step;
        step_z();
      }
    }else{
      double arc_theta = travelled/radius;
      if(current_quadrant == 1) { // X INCREASING Y DECREASING CLOCKWISE 
        if(clockwise){
          if(current_quadrant == start_quadrant){
            ins_x= arc_center_x + cos(theta - arc_theta)*radius;
            ins_y= arc_center_y + sin(theta - arc_theta)*radius;
          }else{
            ins_x= arc_center_x + sin(arc_theta)*radius;
            ins_y= arc_center_y + cos(arc_theta)*radius;
          }          
        }else{  // X DECREASING Y INCREASING ANTI CLOCKWISE
          if(current_quadrant == start_quadrant){
            ins_x= arc_center_x + sin(theta - arc_theta)*radius;
            ins_y= arc_center_y + cos(theta - arc_theta)*radius;
          }else{
            ins_x= arc_center_x + cos(arc_theta)*radius;
            ins_y= arc_center_y + sin(arc_theta)*radius;
          }
        }
      }else if(current_quadrant == 2){ // BOTH INCREASING CLOCKWISE
        if(clockwise){
          if(current_quadrant == start_quadrant){
            ins_x= arc_center_x - sin(theta - arc_theta)*radius;
            ins_y= arc_center_y + cos(theta - arc_theta)*radius;
          }else{
            ins_x= arc_center_x - cos(arc_theta)*radius;
            ins_y= arc_center_y + sin(arc_theta)*radius;
          }

        }else{ // BOTH DECREASING ANTI CLOCKWISE
          if(current_quadrant == start_quadrant){
            ins_x= arc_center_x - cos(theta - arc_theta)*radius;
            ins_y= arc_center_y + sin(theta - arc_theta)*radius;
          }else{
            ins_x= arc_center_x - sin(arc_theta)*radius;
            ins_y= arc_center_y + cos(arc_theta)*radius;
          }
        }
      }else if(current_quadrant == 3) { 
          if(clockwise){ // X INCREASING  Y DECREASING  CLOCKWISE
            if(current_quadrant == start_quadrant){
              ins_x= arc_center_x - cos(theta - arc_theta)*radius;
              ins_y= arc_center_y - sin(theta - arc_theta)*radius;
            }else{
              ins_x= arc_center_x - sin(arc_theta)*radius;
              ins_y= arc_center_y - cos(arc_theta)*radius;
            }
          }else{ // X DECREASING  Y INCREASING ANTI CLOCKWISE
            if(current_quadrant == start_quadrant){
              ins_x= arc_center_x - sin(theta - arc_theta)*radius;
              ins_y= arc_center_y - cos(theta - arc_theta)*radius;

            }else{
              ins_x= arc_center_x - cos(arc_theta)*radius;
              ins_y= arc_center_y - sin(arc_theta)*radius;
            }
          }
      }else if(current_quadrant == 4) { 
          if(clockwise) {// clockwise BOTH DECREASING
            if(current_quadrant == start_quadrant){
              ins_x= arc_center_x + sin(theta - arc_theta)*radius;
              ins_y= arc_center_y - cos(theta - arc_theta)*radius;
            }else{
              ins_x= arc_center_x + cos(arc_theta)*radius;
              ins_y= arc_center_y - sin(arc_theta)*radius;
            }
          }else{ // Anti clockwise BOTH INCREASING
            if(current_quadrant == start_quadrant){
              ins_x= arc_center_x + cos(theta - arc_theta)*radius;
              ins_y= arc_center_y - sin(theta - arc_theta)*radius;
            }else{
              ins_x= arc_center_x + sin(arc_theta)*radius;
              ins_y= arc_center_y - cos(arc_theta)*radius;
            }
          }
      }  
      if(abs(abs(ins_x) - abs(p_x)) >= mm_per_step){
        p_x = p_x + (mm_per_step  * get_direction_x());
        step_x();
        
      }
      if(abs(abs(ins_y) - abs(p_y)) >= mm_per_step){
        p_y = p_y + (mm_per_step * get_direction_y());
        step_y();
        
      }
      delta_z = (travelled * z_circular_distance_ratio) - travelled_z;
      if(abs(delta_z) > mm_per_step){
        p_z = p_z + mm_per_step * get_direction_z();
        travelled_z = travelled_z + mm_per_step;
        step_z();
      }
    }
  }
}

void step_x(){
    PORTF |= (1 << 0);
    delay_1us_nop();
    PORTF &= ~(1 << 0);
   
}

void step_y(){
    PORTF |= (1 << 6);
    delay_1us_nop();
    PORTF &= ~(1 << 6);
}

void step_z(){
    PORTL |= (1 << 3);
    delay_1us_nop();
    PORTL &= ~(1 << 3);
}



void set_quadrants(){
  q1_x1_limit = (arc_center_x + radius);
  q1_y1_limit = (arc_center_y);

  q1_x2_limit = (arc_center_x);
  q1_y2_limit = (arc_center_y + radius);

  q2_x1_limit = (arc_center_x);
  q2_y1_limit = (arc_center_y + radius);

  q2_x2_limit = (arc_center_x - radius);
  q2_y2_limit = (arc_center_y);


  q3_x1_limit = (arc_center_x - radius);
  q3_y1_limit = (arc_center_y);

  q3_x2_limit = (arc_center_x);
  q3_y2_limit = (arc_center_y - radius);

  q4_x1_limit = (arc_center_x);
  q4_y1_limit = (arc_center_y - radius);

  q4_x2_limit = (arc_center_x + radius);
  q4_y2_limit = (arc_center_y);

  if((q1_x1_limit >= prev_x) && (prev_x >= q1_x2_limit) && (q1_y1_limit <= prev_y) && (prev_y <= q1_y2_limit)){
    start_quadrant = 1;
  }else if((q2_x1_limit >= prev_x) && (prev_x >= q2_x2_limit) && (q2_y1_limit >= prev_y) && (prev_y >= q2_y2_limit)){
    start_quadrant = 2;
  }else if((q3_x1_limit <= prev_x) && (prev_x <= q3_x2_limit) && (q3_y1_limit >= prev_y) && (prev_y >= q3_y2_limit)){
    start_quadrant = 3;
  }else if((q4_x1_limit <= prev_x) && (prev_x <= q4_x2_limit) && (q4_y1_limit <= prev_y) && (prev_y <= q4_y2_limit)){
    start_quadrant = 4;
  }


  if((q1_x1_limit >= x) && (x >= q1_x2_limit) && (q1_y1_limit <= y) && (y <= q1_y2_limit)){
    end_quadrant = 1;
  }else if((q2_x1_limit >= x) && (x >= q2_x2_limit) && (q2_y1_limit >= y) && (y >= q2_y2_limit)){
    end_quadrant = 2;
  }else if((q3_x1_limit <= x) && (x <= q3_x2_limit) && (q3_y1_limit >= y) && (y >= q3_y2_limit)){
    end_quadrant = 3;
  }else if((q4_x1_limit <= x) && (x <= q4_x2_limit) && (q4_y1_limit <= y) && (y <= q4_y2_limit)){
    end_quadrant = 4;
  }


  int start_indx = (g_code == 3 && start_quadrant== 4) ? 0 : start_quadrant;
  start_indx = (g_code == 2 && start_quadrant== 1) ? 0 : start_quadrant;
  int end_indx = end_quadrant;
  end_gcode_sub_indx = (end_indx - start_indx) ;
  current_quadrant = start_quadrant;
  //Serial.println(start_quadrant);
  //Serial.println(end_quadrant);
}


void next_quadrant(){
  if(current_quadrant!=end_quadrant){
    if(g_code == 3)
      current_quadrant = (current_quadrant == 4) ? 1 : (current_quadrant + 1);
    else if(g_code == 2)
      current_quadrant = (current_quadrant == 1) ? 4 : (current_quadrant - 1);
    gcode_sub_indx++;
  }
}

void set_intermediate_coordinates(){
  switch (current_quadrant) {
    case 1:
      x = clockwise ? q1_x1_limit : q1_x2_limit;
      y = clockwise ? q1_y1_limit : q1_y2_limit;
    break;
    case 2:
      x = clockwise ? q2_x1_limit : q2_x2_limit;
      y = clockwise ? q2_y1_limit : q2_y2_limit;
    break;
    case 3:
      x = clockwise ? q3_x1_limit : q3_x2_limit;
      y = clockwise ? q3_y1_limit : q3_y2_limit;
    break;
    case 4:
      x = clockwise ? q4_x1_limit : q4_x2_limit;
      y = clockwise ? q4_y1_limit : q4_y2_limit;
    break;

  }

}

void process_quadrant_arc(){
 
  if((current_quadrant == start_quadrant) && (current_quadrant == end_quadrant)){
    //intermediate_x = x;
    //intermediate_y = y;
  }else if(current_quadrant != end_quadrant){
    if(current_quadrant == start_quadrant){
      set_intermediate_coordinates();
    }else{
      prev_x = x;
      prev_y = y;
      set_intermediate_coordinates();
    }
  }else{
    if(current_quadrant!=start_quadrant){
      prev_x = x;
      prev_y = y;
      x = intermediate_x;
      y = intermediate_y;
    }
  }
  set_sub_circular_motion_params();
}



void set_sub_circular_motion_params(){
  
    chord_length = sqrt(pow(prev_x-x,2) + pow(prev_y-y,2));
    
    theta = (asin(chord_length/(2*radius)))*2;
    arc_length = theta * radius;
    distance = arc_length;
    //Serial.println("theta: ");
    //Serial.println(theta);
    //Serial.println(distance);
    
    set_direction();
}


void set_clockwise_for_x(){
  PORTF |= (1 << 1);
}

void set_anti_clockwise_for_x(){
  PORTF &= ~(1 << 1);
}

void set_clockwise_for_y(){
  PORTF |= (1 << 7); //Y axis dir
}

void set_anti_clockwise_for_y(){
  PORTF &= ~(1 << 7); //Y axis dir
  
}


void set_clockwise_for_z(){
  PORTL |= (1 << 1); //Y axis dir
}

void set_anti_clockwise_for_z(){
  PORTL &= ~(1 << 1); //Y axis dir
  
}


void set_direction(){
  if((x - p_x) < 0){ 
    set_anti_clockwise_for_x();
  }else if((x - p_x) > 0){
    set_clockwise_for_x();
  }
  if((y-p_y)<0){ 
    set_anti_clockwise_for_y();
  }else if((y - p_y) > 0){
    set_clockwise_for_y();
  }
  if((z-p_z)<0){ 
    set_clockwise_for_z();
  }else if((z - p_z) > 0){
    set_anti_clockwise_for_z();
  }  
}

int get_direction_x(){
  double inc = x - p_x;
  if(inc < 0)
    return -1;
  if(inc > 0)
    return 1;
  else
    return 0;
}

int get_direction_y(){
  double inc = y - p_y;
  if(inc < 0)
    return -1;
  if(inc > 0)
    return 1;
  else
    return 0;
}

int get_direction_z(){
  double inc = z - p_z;
  if(inc < 0)
    return -1;
  if(inc > 0)
    return 1;
  else
    return 0;
}



void pre_process(){
    double steps_per_sec = steps_per_mm * f; // conversion of feedrate which is Speed of mm/sec to Speed in steps/sec
    count =  round(timer_frequency/(steps_per_sec)) - 1;
    duration = (count * 1/timer_frequency);
    double z_diff = abs(z - p_z);

  if(is_linear_motion){
    double x_diff = abs(x - p_x);
    double y_diff = abs(y - p_y);
    

    distance = sqrt((x_diff * x_diff) + (y_diff * y_diff) + (z_diff * z_diff));
    
  
    x_distance_ratio = x_diff/distance;
    y_distance_ratio = y_diff/distance;
    z_distance_ratio = z_diff/distance;
  }else{
    arc_center_x = I + prev_x;
    arc_center_y = J + prev_y;
    intermediate_x = x;
    intermediate_y = y;
    radius = sqrt(pow(prev_x - arc_center_x,2) + pow(prev_y - arc_center_y,2));  
    
    double v1_x = prev_x - arc_center_x;
    double v1_y = prev_y - arc_center_y;

    double v2_x = x - arc_center_x;
    double v2_y = y - arc_center_y;

    double theta1 = atan2(v1_y,v1_x);
    double theta2 = atan2(v2_y,v2_x);

    double theta_diff = theta2 - theta1;
    double two_pie = (2 * 3.14);
    theta_diff = (theta_diff < 0)?  two_pie + theta_diff : theta_diff;
    theta_diff = clockwise ? (two_pie- theta_diff) : theta_diff;
    double arc_distance = radius * theta_diff;
    
    z_circular_distance_ratio = z_diff/arc_distance;
    //Serial.println(z_circular_distance_ratio,DEC);
    set_quadrants();
    process_quadrant_arc();    
     
  }
  set_direction();
  unsigned char sreg;
  sreg = SREG;
  cli();
  OCR1A = (unsigned int)count;
  SREG = sreg;
  TCCR1B = (1<<WGM12) | (1<<CS10);
  sei();
 
}

void set_gcode_modes(){
  is_gcode_motion_command = 0;
  
  switch(g_code){
    case 0:
      f = FAST_MOVE;
      is_linear_motion = 1; 
      is_gcode_motion_command = 1;
      break;
    case 1:
      is_linear_motion = 1;    
      is_gcode_motion_command = 1;
      break;
    case 2:
      is_linear_motion = 0;
      is_gcode_motion_command = 1;
      clockwise = 1;
      break;

    case 3:
      is_linear_motion = 0;  
      is_gcode_motion_command = 1;
      clockwise = 0;    
      break;
  }
}


void parse_gcodes(){
  char temp_string[500];
  Serial.println(gcode_indx);
  strcpy(temp_string, gcodes[gcode_indx-previous_batch_gcode_indx]);
  char *token;
  const char *delimiter = " ";
  token = strtok(temp_string, delimiter);

  while (token != NULL) {
    char type = token[0];
    char* val= token+1;
    switch(type){
      case 'G':
        g_code=atoi(val);
        //Serial.println(g_code);
        set_gcode_modes();
        break;
      case 'X':
        prev_x=x;
        x=atoi(val);
        break;
      case 'Y':
        prev_y=y;
        y=atoi(val);
        y = (y==0) ? 0.00001 : y;
        break;
      case 'Z':
        prev_z=z;
        z=atoi(val);
        z = (z==0) ? 0.00001 : z;
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
  if(is_gcode_motion_command) { 
    pre_process();
    
  } else {
    //Serial.println(gcode_indx);
    gcode_indx++;
    
    //process_next_gcode= 1;
  }
}


int readNextNLines(){
  int lines= 0;
  int batch_read = 0;
  //Serial.println("called");
  if(gcode_file.available()){
    for(int i=0;i<GCODE_BUFFER_LENGTH;i++){
      String line = gcode_file.readStringUntil('\n');
      int len = line.length() + 1;
      char gcode_ary [len];
      line.toCharArray(gcode_ary, len);
      free(gcodes[i]);
      gcodes[i] = malloc(len);
      strcpy(gcodes[i], gcode_ary);
      //Serial.println(line);
      //Serial.println(gcode_indx);
      
    }
    batch_read = 1;
  }else{
    if(!is_gcode_file_closed){
      gcode_file.close();
      is_gcode_file_closed=1;
    }
  }
  return batch_read;
}

 
    
  
void home(){
  while(!(PINE & (1 << 5))){ //X Axis
    set_clockwise_for_x();
    step_x();
    delayMicroseconds(100);
  }
  int k=0;
  while(true){
    k++;
    set_anti_clockwise_for_x();
    step_x();
    delayMicroseconds(100);
    //Serial.println(k);
    if(k>5000) break;
  }
  PORTD |= (1 << 7);

  k = 0;
  while(!(PINJ & (1 << 0))){ //Y Axis
    set_clockwise_for_y();
    step_y();
    delayMicroseconds(100);
  }
  while(true){
    k++;
    set_anti_clockwise_for_y();
    step_y();
    delayMicroseconds(100);
    //Serial.println(k);
    if(k>5000) break;
  }
  PORTF |= (1 << 2);
  
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("SD Card initialization failed.");
    while(true);
  }
  Serial.println("SD Card initialized..");
  // Enable the stepper driver (LOW to enable for DRV8825)
  gcode_file = SD.open("s_gcode.gcd");
  
  DDRD |= (1 << 7);   // PIN 38 as output ENABLE Pin of X
  PORTD &= ~(1 << 7); // ENABLE Pin set to LOW

  DDRF |= (1 << 0);  // PIN 54 as output STEP Pin of X
  DDRF |= (1 << 1);  // PIN 55 as output DIR pin of X



  DDRF |= (1 << 2);   // PIN 56 as output ENABLE Pin of Y
  PORTF &= ~(1 << 2); // ENABLE Pin set to LOW

  DDRF |= (1 << 6);  // PIN 60 as output STEP Pin of Y
  DDRF |= (1 << 7);  // PIN 61 as output DIR Pin of Y

  pinMode(ENABLE_Z_MOTOR_PIN, OUTPUT);
  digitalWrite(ENABLE_Z_MOTOR_PIN, LOW);
  
  DDRL |= (1 << 3);  // PIN 46 as output STEP Pin of Z
  DDRL |= (1 << 1);  // PIN 48 as output DIR Pin of Z

  DDRE &= ~(1 << 5); // Pin 3 for X Min Endstop
  DDRJ &= ~(1 << 0); // Pin 14 for Y Min Endstop
  
  //pinMode(14,INPUT_PULLUP);

  TCCR1A = 0;
  //TCCR1B = (1<<WGM12) | (1<<CS10);
  home();
  TIMSK1=  (1<<OCIE1A);
  readNextNLines();
  parse_gcodes();
  //sei(); 
}

void check_end_stops(){
  if (PINE & (1 << 5)) {
    PORTD |= (1 << 7);
    
  }
  if (PINJ & (1 << 0)) {
    PORTF |= (1 << 2);
  }
}

void loop() {

  if((gcode_indx - previous_batch_gcode_indx)<GCODE_BUFFER_LENGTH){
    if(gcode_indx>prev_gcode_indx){
    //process_next_gcode = 0;
        //Serial.println(gcode_indx);
     //Serial.println(prev_gcode_indx);
     prev_gcode_indx = gcode_indx;
     parse_gcodes();
    //Serial.println(gcode_indx);
    
    //Serial.println(gcode_indx);
     //Serial.println(prev_gcode_indx);
   // TCCR1B = (1<<WGM12) | (1<<CS10);
    }
  }else{
    int x = readNextNLines();
    //Serial.println(gcode_indx);
    //Serial.println(previous_batch_gcode_indx);
    //Serial.println(x);
    if(x){
      previous_batch_gcode_indx = gcode_indx;
    }
    //Serial.println(gcode_indx);
    //Serial.println(previous_batch_gcode_indx);
  }
  /*
  if(gcode_indx>prev_gcode_indx){
    parse_gcodes(gcode_indx);
    prev_gcode_indx=gcode_indx;

    TCCR1B = (1<<WGM12) | (1<<CS10);
  }
  
  if(gcode_sub_indx>prev_gcode_sub_indx){
    //Serial.println(gcode_sub_indx);
    process_quadrant_arc();
    prev_gcode_sub_indx = gcode_sub_indx;
    TCCR1B = (1<<WGM12) | (1<<CS10);
  }
*/

check_end_stops();

}


