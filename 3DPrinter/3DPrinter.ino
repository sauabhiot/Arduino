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
#define BASE_FREQUENCY 16000000
#define STEPS_PER_REVOLUTION 200
#define MICROSTEPPING 16
#define SCREW_PITCH 2
#define NO_OF_STARTS 4
#define CLOCK_PRESCALAR 1
#define TIMER_FREQUENCY BASE_FREQUENCY/CLOCK_PRESCALAR

#define TOTAL_GCODES 100
#define ENABLE_Z_MOTOR_PIN 62 
#define GCODE_BUFFER_LENGTH 5
#define ACC_PROFILE 1000

#define THERMISTOR_PIN A13 
#define HEATER_BED_PIN A14
#define CHIP_SELECT_PIN 53

#define HEATER_BLOCK_PIN 40 

#define TEMP_TOLERANCE 3
#define PRINT_MODE 0 // 0 for XY Plootter 1 for 3D Printer


const float SERIES_RESISTOR = 4700.0;
const float VCC = 5.0;
const float NOMINAL_RESISTANCE = 100000.0; // Resistance at 25°C
const float NOMINAL_TEMPERATURE = 25.0;   // Nominal temperature in Celsius
const float B_PARAMETER = 3950;         // Beta parameter


int temp_tolerance_count_bed = 50;
int temp_tolerance_count_extruder = 50;

int z_lifted = 0;

int actual_gcode_buffer_length = 0;





volatile int previous_batch_gcode_indx = 0;

int is_linear_motion = 1;
//volatile int is_gcode_motion_command = 0 ;
double count = 0.0;

double x_distance_ratio, y_distance_ratio, z_distance_ratio, e_distance_ratio, z_circular_distance_ratio, e_circular_distance_ratio;

const int STEPS_PER_MM = (STEPS_PER_REVOLUTION * MICROSTEPPING)/(SCREW_PITCH * NO_OF_STARTS); // (200 *16)/(2*4) = 400
const double MM_PER_STEP = 1.00/(double)STEPS_PER_MM; // 0.0025
const double MM_PER_STEP_EXTRUDER = MM_PER_STEP * (21.0/5.0);


File gcode_file;
int is_gcode_file_closed = 0;

int proportional = 0;
int integral = 0;
int derivative = 0;
int pid = 0;
int accumulated_temp_error = 0;
int prev_temp_error = 0;


char *gcodes[GCODE_BUFFER_LENGTH];

int gcode_indx = 0 , prev_gcode_indx = 0;
int end_gcode_sub_indx = 0 , gcode_sub_indx = 0 , prev_gcode_sub_indx = 0;

volatile double x=0,y=0,z=0,extruder=0;
double feed_rate=3;

volatile double prev_x=0;
volatile double prev_y=0;
volatile double prev_z=0;
volatile double prev_e=0;
double prev_feed_rate=feed_rate;


volatile double distance = 0;
long total_distance_in_steps = 0;
long constant_velocity_distance_in_steps = 0;
long deceleration_start_step=0;
long current_step_count =0;
double expected_deceleration_distance = 0;
double expected_distance = 0;
volatile double travelled=0, travelled_x = 0, travelled_y = 0,travelled_z=0,travelled_e=0;


double p = 0.0;

double ins_x = 0, ins_y = 0;
double p_x = 0, p_y = 0, p_z = 0, p_e = 0;
double delta_x = 0.0, delta_y = 0.0, delta_z = 0.0, delta_e = 0.0;;
double target_hotend_temp = -1;
double target_bed_temp = 55;
double part_cooling_fan_speed = -1;
int g_code = -1;


double radius = 0.0;
double I = 0.0, J = 0.0;
double arc_center_x=0;
double arc_center_y=0;
double theta;

long gcode_counter = 0;

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


int disable_homing = 1;

int accel_steps = 0;
int current_accel_step = 0;
double time_increment = 0.0;


int skip(){
  if(!is_linear_motion) return 0; 
  if(accel_steps >= 0){
    accel_steps--;
    current_accel_step++;
    double time = (time_increment * current_accel_step);
    double expected_velocity = ACC_PROFILE * time;
    expected_distance = expected_distance + (expected_velocity * time_increment);
    return ((travelled + MM_PER_STEP) > expected_distance);
  }else if(current_step_count>deceleration_start_step){
    double time = time_increment * (current_step_count-deceleration_start_step);
    double expected_velocity = feed_rate-(ACC_PROFILE * time);
    expected_deceleration_distance = expected_deceleration_distance + (expected_velocity * time_increment);
    double actual_deceleration_distance = MM_PER_STEP * (current_step_count-deceleration_start_step);
    return (actual_deceleration_distance - expected_deceleration_distance)>MM_PER_STEP;
  }
  return 0;
}

void  maintain_extruder_temperature(){
  float kP = 1;
  float kI = 0.1;
  float kD = 20;

  float temp = read_extruder_temperature();
  float err = target_hotend_temp - temp;

  if(part_cooling_fan_speed < 0){
    if(err<0) 
      OCR2B = 255;
    else
      OCR2B = 0;
  } else{
    OCR2B = part_cooling_fan_speed;
  }

  proportional = (kP * err);
  proportional = (proportional<204) ? proportional : 178;

  
  integral = (kI * accumulated_temp_error);
  if((integral<=51  && integral>=0) || (integral>=51 && err<=0) || (integral<=0 && err>=0)){
    accumulated_temp_error += err;
  }

  derivative = kD * (err - prev_temp_error);
  derivative = (derivative < 25) ? derivative : 25;

  pid = (proportional + integral + derivative);
  if(pid < 0) pid = 0;
  if(pid > 255) pid = 255;


  OCR2A = pid;
  temp_tolerance_count_extruder = abs(temp-target_hotend_temp) < TEMP_TOLERANCE ? temp_tolerance_count_extruder > -50 ? (temp_tolerance_count_extruder - 1):temp_tolerance_count_extruder : 50;
  prev_temp_error = err;
}

void maintain_bed_temperature(){

  float kP = 140;
  float kI = 0.3;
  float kD = 20;

  float temp = read_bed_temperature();
  float err = target_bed_temp - temp;

  proportional = (kP * err);
  proportional = (proportional<150) ? proportional : 150;

  
  integral = (kI * accumulated_temp_error);
  if((integral<=81  && integral>=0) || (integral>=81 && err<=0) || (integral<=0 && err>=0)){
    accumulated_temp_error += err;
  }

  derivative = kD * (err - prev_temp_error);
  derivative = (derivative < 25) ? derivative : 25;

  pid = (proportional + integral + derivative);
  if(pid < 0) pid = 0;
  if(pid > 255) pid = 255;
/*
  Serial.print(proportional);
  Serial.print("----");
  Serial.print(integral);
  Serial.print("----");
  Serial.println(derivative);
  */
   //Serial.println(pid);

  //delay(500);
  OCR4C = pid;

  temp_tolerance_count_bed = abs(temp-target_bed_temp) < TEMP_TOLERANCE ? temp_tolerance_count_bed > -50 ? (temp_tolerance_count_bed - 1):temp_tolerance_count_bed : 50;
  prev_temp_error = err;


}

ISR(TIMER1_COMPA_vect){
  //Serial.println((travelled ,DEC);
  //if(((travelled + (5.0 * MM_PER_STEP))  >= distance) && (distance >(5.0 * MM_PER_STEP) )){
  if(((travelled  >= distance))){
    if(end_gcode_sub_indx == gcode_sub_indx) {
      gcode_indx++;
      
    }
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); 
    travelled = 0.0;
    travelled_x = 0.0;
    travelled_y = 0.0;
    travelled_z = 0.0;
    travelled_e = 0.0;
    current_step_count = 0;
    PORTD |= (1 << 7);
    PORTF |= (1 << 2);
    digitalWrite(ENABLE_Z_MOTOR_PIN, HIGH);
    PORTA |= (1 << 2); 
    gcode_counter++;
    if(!is_linear_motion) next_quadrant();
  }else{
    if(!skip()){
      travelled = travelled + MM_PER_STEP;
      PORTD &= ~(1 << 7);
      PORTF &= ~(1 << 2);
      digitalWrite(ENABLE_Z_MOTOR_PIN, LOW);
      PORTA &= ~(1 << 2); 
      if(is_linear_motion){
        delta_x = (travelled * x_distance_ratio) - travelled_x;
        delta_y = (travelled * y_distance_ratio) - travelled_y;
        delta_z = (travelled * z_distance_ratio) - travelled_z;
        delta_e = (travelled * e_distance_ratio) - travelled_e;
        current_step_count++;
        if(abs(delta_x) > MM_PER_STEP){
          p_x = p_x + MM_PER_STEP * get_direction_x();
          travelled_x = travelled_x + MM_PER_STEP;
          step_x();
        }
        if(abs(delta_y) > MM_PER_STEP){
          p_y = p_y + MM_PER_STEP * get_direction_y();
          travelled_y = travelled_y + MM_PER_STEP;
          step_y();
        }
        if(abs(delta_z) > MM_PER_STEP){
          p_z = p_z + MM_PER_STEP * get_direction_z();
          travelled_z = travelled_z + MM_PER_STEP;
          step_z();
        }
        if(abs(delta_e) > MM_PER_STEP_EXTRUDER){
          p_e = p_e + MM_PER_STEP_EXTRUDER * get_direction_e();
          travelled_e = travelled_e + MM_PER_STEP_EXTRUDER;
          step_e();
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
        if(abs(abs(ins_x) - abs(p_x)) >= MM_PER_STEP){
          p_x = p_x + (MM_PER_STEP  * get_direction_x());
          step_x();
        }
        if(abs(abs(ins_y) - abs(p_y)) >= MM_PER_STEP){
          p_y = p_y + (MM_PER_STEP * get_direction_y());
          step_y();
        }
        delta_z = (travelled * z_circular_distance_ratio) - travelled_z;
        if(abs(delta_z) > MM_PER_STEP){
          p_z = p_z + MM_PER_STEP * get_direction_z();
          travelled_z = travelled_z + MM_PER_STEP;
          step_z();
        }
        delta_e = (travelled * e_circular_distance_ratio) - travelled_e;
        if(abs(delta_e) > MM_PER_STEP_EXTRUDER){
          p_e = p_e + MM_PER_STEP_EXTRUDER * get_direction_e();
          travelled_e = travelled_e + MM_PER_STEP_EXTRUDER;
          step_e();
        }
      }
    }else{
      // Keep steppermotors disabled during skip phase
      PORTD |= (1 << 7);
      PORTF |= (1 << 2);
      digitalWrite(ENABLE_Z_MOTOR_PIN, HIGH);
      PORTA |= (1 << 2); 
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

void step_e(){
    PORTA |= (1 << 4);
    delay_1us_nop();
    PORTA &= ~(1 << 4);
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
    double chord_length = sqrt(pow(prev_x-x,2) + pow(prev_y-y,2));
    theta = (asin(chord_length/(2*radius)))*2;
    double arc_length = theta * radius;
    distance = arc_length;
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
  PORTL &= ~(1 << 1); //Z axis dir
  
}

void set_anti_clockwise_for_z(){
  PORTL |= (1 << 1); //Z axis dir
}

void set_clockwise_for_e(){
  PORTA &= ~(1 << 6); //E dir
  
}

void set_anti_clockwise_for_e(){
  PORTA |= (1 << 6); //E dir
}


void set_direction(){
  if(get_direction_x() < 0){ 
    set_clockwise_for_x();
  }else if(get_direction_x() > 0){
    set_anti_clockwise_for_x();
  }
  
  if(get_direction_y() < 0){ 
    set_anti_clockwise_for_y();
  }else if(get_direction_y() > 0){
    set_clockwise_for_y();
  }

  if(get_direction_z() < 0){ 
    set_anti_clockwise_for_z();
  }else if(get_direction_z() > 0){
    set_clockwise_for_z();
  }  

  if(get_direction_e() < 0){ 
    set_clockwise_for_e();
  }else if(get_direction_e() > 0){
    set_anti_clockwise_for_e();
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

int get_direction_e(){
  double inc = extruder - p_e;
  if(inc < 0)
    return -1;
  if(inc > 0)
    return 1;
  else
    return 0;
}



void pre_process(){
    double steps_per_sec = STEPS_PER_MM * feed_rate; // conversion of feedrate which is Speed of mm/sec to Speed in steps/sec
    count =  round(TIMER_FREQUENCY/(steps_per_sec)) - 1;
    time_increment = count/TIMER_FREQUENCY;
    double transition_duration = feed_rate/ACC_PROFILE;
    accel_steps = round(transition_duration * feed_rate * STEPS_PER_MM);
    double z_diff = abs(z - p_z);
    double e_diff = abs(extruder - p_e);
  if(is_linear_motion){
    double x_diff = abs(x - p_x);
    double y_diff = abs(y - p_y);
    distance = sqrt((x_diff * x_diff) + (y_diff * y_diff) + (z_diff * z_diff) + (e_diff * e_diff));
    total_distance_in_steps = round(distance * STEPS_PER_MM);
    constant_velocity_distance_in_steps = total_distance_in_steps -(2 * accel_steps); 
    deceleration_start_step = accel_steps + constant_velocity_distance_in_steps;
    x_distance_ratio = x_diff/distance;
    y_distance_ratio = y_diff/distance;
    z_distance_ratio = z_diff/distance;
    e_distance_ratio = e_diff/distance;
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
    e_circular_distance_ratio = e_diff/arc_distance;
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
  switch(g_code){
    case 0:
      prev_feed_rate = feed_rate;
      feed_rate = FAST_MOVE;
      is_linear_motion = 1; 
      if(PRINT_MODE == 0 && !z_lifted){ 
          moveZ(1);
      }
      break;
    case 1:
      feed_rate = prev_feed_rate;
      is_linear_motion = 1;    
      if(PRINT_MODE == 0 && z_lifted){ 
        moveZ(0);
      }
      break;
    case 2:
      feed_rate = prev_feed_rate;
      is_linear_motion = 0;
      clockwise = 1;
      if(PRINT_MODE == 0 && z_lifted){ 
        moveZ(0);
      }
      break;
    case 3:
      feed_rate = prev_feed_rate;
      is_linear_motion = 0;  
      clockwise = 0;    
      if(PRINT_MODE == 0 && z_lifted){ 
        moveZ(0);
      }
      break;
    case 20:
      //Serial.println("Inch system");
      break;  
    case 21:
      //Serial.println("mm system");
      break;  
    case 28:
      Serial.println("Homing..");
      home();
      break;  
    case 90:
      //Serial.println("absolute positioning");
      break;        
    case 107:
      //part_cooling_fan_speed = 0;
      break;    
  }
}

int is_gcode_motion_command(){
  if(abs(x-prev_x) >0) return 1;
  if(abs(y-prev_y) >0) return 1;
  if(abs(z-prev_z) >0) return 1;
  if(abs(extruder-prev_e) >0) return 1;
  return 0;
}

void parse_gcodes(){
  char temp_string[500];
  g_code = -1;
  distance = 0.0;
  prev_x = x;
  prev_y = y;
  prev_z = z;
  prev_e = extruder;

  strcpy(temp_string, gcodes[gcode_indx-previous_batch_gcode_indx]);
  char *token;
  const char *delimiter = " ";
  token = strtok(temp_string, delimiter);
 
  size_t length = strlen(token);
  while (token != NULL) {
    
    char type = token[0];
    char* val= token+1;
    
    switch(type){
      case ';':
        set_gcode_modes();
        break;
      case 'G':
        g_code = atoi(val);
        set_gcode_modes();
        break;
      case 'M':
        g_code = atoi(val);
        set_gcode_modes();
        break;
      case 'X':
        x=atof(val);
        break;
      case 'Y':
        y=atof(val);
        y = (y==0) ? 0.00001 : y;
        break;
      case 'Z':
        z=atof(val);
        z = (z==0) ? 0.00001 : z;
        break;        
      case 'I':
        I=atof(val);
        break;
      case 'J':
        J=atof(val);
        break;            
      case 'E':
        extruder = atof(val);
        //e = (e==0) ? 0.00001 : e;      
        if(g_code == 92) p_e = extruder;
        break;
      case 'F':
         feed_rate=atoi(val)/60;
         prev_feed_rate = feed_rate;
         break;
      case 'S':
        if(g_code == 104 || g_code== 109){
          target_hotend_temp = atof(val) + 5;
        }else if(g_code == 140 || g_code == 190){
          target_bed_temp = atof(val);
        }else if(g_code == 106){
          part_cooling_fan_speed = atof(val);
        }
        break;    
    }
    token = strtok(NULL, delimiter); // Get the next token
  }

    Serial.print(gcode_indx);
    Serial.print(" ### ");
    Serial.print(g_code);
    Serial.print(" ### ");
    //Serial.print(target_hotend_temp);
    //Serial.print(" ### ");
    //Serial.print(distance,DEC);
    //Serial.print(" ### ");
    Serial.print(extruder, DEC);
    Serial.print(" ### ");
    Serial.print(p_e, DEC);
    Serial.print(" ### ");

    Serial.print(get_direction_e());
    Serial.print(" ### ");
    Serial.println(feed_rate);

  if(is_gcode_motion_command()) { 
    pre_process();
  }else if(g_code == 109){
    while(target_hotend_temp >0 && temp_tolerance_count_extruder > 0){
      maintain_extruder_temperature();
    }
    Serial.print("Hotend Temp: ");
    Serial.println(read_extruder_temperature());
    gcode_indx++;
  }else if(g_code == 190){
    while(target_bed_temp > 0 && temp_tolerance_count_bed > 0){
      maintain_bed_temperature();
    }
    Serial.print("Bed Temp: ");
    Serial.println(read_bed_temperature());
    gcode_indx++;
  }
  else {
    gcode_indx++;
  }
}


int readNextNLines(){
  int lines= 0;
  int batch_read = 0;
  if(gcode_file.available()){
        
    for(int i=0;i<GCODE_BUFFER_LENGTH;i++){
      String line = gcode_file.readStringUntil('\n');
      if(line == "") {
        break;
      }
      actual_gcode_buffer_length = i;
      int len = line.length() + 1;
      char gcode_ary [len];
      line.toCharArray(gcode_ary, len);
      free(gcodes[i]);
      gcodes[i] = malloc(len);
      strcpy(gcodes[i], gcode_ary);
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

void moveZ(int z_clockwise){
  int k=0;
  digitalWrite(ENABLE_Z_MOTOR_PIN, LOW);
    if(z_clockwise){
      set_clockwise_for_z();
      z_lifted = 1;
    }else{
      set_anti_clockwise_for_z();
      z_lifted = 0;
    }

    while(true){
      
      k++;
      step_z();
      delayMicroseconds(300);
      if(k>900) break;
  }
}
    
  
void home(){
  if(disable_homing) return; 
  PORTD &= ~(1 << 7);
  PORTF &= ~(1 << 2); 
  digitalWrite(ENABLE_Z_MOTOR_PIN, LOW);

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
    if(k>5000) break;
  }
  PORTD |= (1 << 7);

  k = 0;
  while(!(PINJ & (1 << 0))){ //Y Axis
    set_anti_clockwise_for_y();
    step_y();
    delayMicroseconds(100);
  }
  
  while(true){
    k++;
    set_clockwise_for_y();
    
    step_y();
    delayMicroseconds(100);
    if(k>5000) break;
  }
  PORTF |= (1 << 2);

  k = 0;
  
  while(!(PIND & (1 << 3))){ //Z Axis
    set_anti_clockwise_for_z();
    step_z();
    delayMicroseconds(100);
  }


  while(true){
    k++;
    set_clockwise_for_z();
    step_z();
    delayMicroseconds(100);
    if(k>2000) break;
  }
  digitalWrite(ENABLE_Z_MOTOR_PIN, HIGH);
  
}


void z_test(){
  int k=0;
  while(true){
    k++;
    set_clockwise_for_z();
    step_z();
    delayMicroseconds(300);
    if(k>5000) break;
  }
  digitalWrite(ENABLE_Z_MOTOR_PIN, HIGH);
}

double read_extruder_temperature(){
  double analog_val = analogRead(THERMISTOR_PIN); 
  float v_out = (analog_val * VCC) / 1023.0;
  float thermistor_resistance = (SERIES_RESISTOR * v_out) / (VCC - v_out);
  float temp_kelvin = 1.0 / ( (1.0 / (NOMINAL_TEMPERATURE + 273.15)) + (log(thermistor_resistance / NOMINAL_RESISTANCE) / B_PARAMETER) );
  double curr_temp = (temp_kelvin-273.15);
  Serial.print("Extruder Temp: ");
  Serial.println(curr_temp);
  return curr_temp; 
  //return 205.00;
}
double read_bed_temperature(){
  double analog_val = analogRead(HEATER_BED_PIN); 
  float v_out = (analog_val * VCC) / 1023.0;
  float thermistor_resistance = (SERIES_RESISTOR * v_out) / (VCC - v_out);
  float temp_kelvin = 1.0 / ( (1.0 / (NOMINAL_TEMPERATURE + 273.15)) + (log(thermistor_resistance / NOMINAL_RESISTANCE) / B_PARAMETER) );
  double curr_temp = (temp_kelvin-273.15);
  Serial.print("Bed Temp: ");
  Serial.println(curr_temp);
  //return curr_temp; 
  return 60.00;
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.print("Initializing SD card...");
  if (!SD.begin(CHIP_SELECT_PIN)) {
    Serial.println("SD Card initialization failed.");
    while(true);
  }
  Serial.println("SD Card initialized..");
  gcode_file = SD.open("tri2.gcd");
  // Enable the stepper driver (LOW to enable for DRV8825)
  
  DDRD |= (1 << 7);   // PIN 38 as output ENABLE Pin of X
  PORTD &= ~(1 << 7); // ENABLE Pin set to LOW

  DDRF |= (1 << 0);  // PIN 54 as output STEP Pin of X
  DDRF |= (1 << 1);  // PIN 55 as output DIR pin of X

  DDRF |= (1 << 2);   // PIN 56 as output ENABLE Pin of Y
  PORTF &= ~(1 << 2); // ENABLE Pin set to LOW

  DDRF |= (1 << 6);  // PIN 60 as output STEP Pin of Y
  DDRF |= (1 << 7);  // PIN 61 as output DIR Pin of Y


  pinMode(ENABLE_Z_MOTOR_PIN, OUTPUT);
  
  //pinMode(HEATER_BLOCK_PIN, OUTPUT);
  
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  
  digitalWrite(ENABLE_Z_MOTOR_PIN, LOW);

  
  
  DDRL |= (1 << 3);  // PIN 46 as output STEP Pin of Z
  DDRL |= (1 << 1);  // PIN 48 as output DIR Pin of Z

  DDRE &= ~(1 << 5); // Pin 3 for X Min Endstop
  DDRJ &= ~(1 << 0); // Pin 14 for Y Min Endstop
  DDRD &= ~(1 << 3); // Pin 18 for Z Min Endstop
  
  DDRA |= (1 << 2);   // PIN 24 as output ENABLE Pin of E
  PORTA &= ~(1 << 2); // ENABLE Pin set to LOW
  //PORTA |= (1 << 2); 

  DDRA |= (1 << 4);  // PIN 26 as output STEP Pin of E
  DDRA |= (1 << 6);  // PIN 28 as output DIR Pin of E

  TCCR1A = 0;

  TCCR2A = 0;
  TCCR2B = 0;

  TCCR2A |= (1 << WGM21) | (1 << WGM20);
  TCCR2A |= (1 << COM2A1);


  TCCR2A |= (1 << COM2B1);

  TCCR2B |= (1 << CS20);


  // HEATER BED
  TCCR4A = 0;
  TCCR4B = 0;


  TCCR4B |= (1 << WGM42);  
  TCCR4A |= (1 << WGM40); 
  
  TCCR4A |= (1 << COM4C1); 

  
  TCCR4B |= (1 << CS40);


  OCR4C = 0;
  home();
  readNextNLines();
  TIMSK1=  (1<<OCIE1A);
  parse_gcodes();
}

void check_end_stops(){
  if (PINE & (1 << 5)) {
    PORTD |= (1 << 7);
  }
  if (PINJ & (1 << 0)) {
    PORTF |= (1 << 2);
  }
  if (PIND & (1 << 3)) {
    digitalWrite(ENABLE_Z_MOTOR_PIN, HIGH);
  }
}


void manage_linear_motion(){
  if((gcode_indx - previous_batch_gcode_indx)<=actual_gcode_buffer_length){
    if(gcode_indx>prev_gcode_indx){
     prev_gcode_indx = gcode_indx;
     parse_gcodes();
    }
  }else{
    int x = readNextNLines();
    if(x){
      previous_batch_gcode_indx = gcode_indx;
    }
  }
}

void manage_circular_motion(){
  if(gcode_sub_indx>prev_gcode_sub_indx){
    process_quadrant_arc();
    prev_gcode_sub_indx = gcode_sub_indx;
    TCCR1B = (1<<WGM12) | (1<<CS10);
  }
}

void loop() {
  maintain_extruder_temperature();
  maintain_bed_temperature();
  if((gcode_indx - previous_batch_gcode_indx)<=actual_gcode_buffer_length){
    if(gcode_indx>prev_gcode_indx){
     prev_gcode_indx = gcode_indx;
     parse_gcodes();
    }
  }else{
    int x = readNextNLines();
    
    if(x){
      previous_batch_gcode_indx = gcode_indx;
    }
  }
  
  if(gcode_sub_indx>prev_gcode_sub_indx){
    process_quadrant_arc();
    prev_gcode_sub_indx = gcode_sub_indx;
    TCCR1B = (1<<WGM12) | (1<<CS10);
  }
 
}


