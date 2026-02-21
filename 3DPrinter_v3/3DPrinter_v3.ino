#include "Config.h"

#include "Stepper.h"
#include "Utils.h"
#include "Gcode.h"
#include "Parser.h"
#include "RingBuffer.h"
#include "SerialCommunication.h"

volatile float travelled = 0.0f;
volatile long travelled_in_steps = 0; 

volatile float distance  = 0.0f;
volatile long distance_in_steps = 0; 

float x_increment = 0.0f, x_move_distance = 0.0f, x_travelled = 0.0f;
float y_increment = 0.0f, y_move_distance = 0.0f, y_travelled = 0.0f;
float z_increment = 0.0f, z_move_distance = 0.0f, z_travelled = 0.0f;
float e_increment = 0.0f, e_move_distance = 0.0f, e_travelled = 0.0f;

bool absolute_positioning = true;  
bool print_head_lifted = false;

uint16_t timer_counter = 0;
uint16_t timer_increment = 0;
float time_increment = 0.0f;
bool is_linear_motion = false;
bool is_circular_motion = false;
long deceleration_start_step = 0;
volatile bool isr_completed = true;
volatile bool plate_touched = false;
uint16_t motion_timer_count = 0;
float radius = 0.0f, center_x = 0.0f, center_y = 0.0f, delta_cos = 0.0f, delta_sin = 0.0f;
float start_angle = 0.0f;
uint8_t quadrant = -1;
char temp_string[COMMAND_SIZE];
long i = 0;

float ARC_SEGMENT_LENGTH_NEW = 0;

uint16_t accel_steps = 0, current_accel_step = 0, current_step_count = 0;
float expected_distance = 0.0f, expected_deceleration_distance = 0.0f;
Gcode gcode; 
StepperX xStepper(&gcode);
StepperY yStepper(&gcode);
StepperZ zStepper(&gcode);
StepperE eStepper(&gcode);

Parser parser(&gcode);
RingBuffer ringBuffer;
SerialCommunication serialCommunication(&ringBuffer);

void preprocess(){
  float start_x_coord = xStepper.get_previous_position();
  float start_y_coord = yStepper.get_previous_position();
  float start_z_coord = zStepper.get_previous_position();
  float start_e_coord = eStepper.get_previous_position();

  float end_x_coord = xStepper.get_coord(absolute_positioning);
  float end_y_coord = yStepper.get_coord(absolute_positioning);
  float end_z_coord = zStepper.get_coord(absolute_positioning);
  float end_e_coord = eStepper.get_coord(absolute_positioning);

  //if(gcode.get_print_mode() == false) zStepper.moveNozzle();
  float steps_per_sec = STEPS_PER_MM * gcode.get_feed_rate();
  motion_timer_count =  round(TIMER_FREQUENCY/(steps_per_sec)) - 1;
  time_increment = (float) motion_timer_count/TIMER_FREQUENCY;
  float transition_duration = gcode.get_feed_rate()/ACC_PROFILE;
  accel_steps = round(transition_duration * gcode.get_feed_rate() * STEPS_PER_MM);  
  float z_diff = abs(end_z_coord - start_z_coord);
  float e_diff = abs(end_e_coord - start_e_coord);
  if(is_linear_motion){
    float x_diff = abs(end_x_coord - start_x_coord);
    float y_diff = abs(end_y_coord - start_y_coord);
    distance = sqrt((x_diff * x_diff) + (y_diff * y_diff) + (z_diff * z_diff) + (e_diff * e_diff));
    distance_in_steps = distance *STEPS_PER_MM;

    float total_distance_in_steps = round(distance * STEPS_PER_MM);
    float constant_velocity_distance_in_steps = total_distance_in_steps -(2 * accel_steps); 
    deceleration_start_step = accel_steps + constant_velocity_distance_in_steps;
    x_move_distance = x_diff/distance;
    y_move_distance = y_diff/distance;
    z_move_distance = z_diff/distance;
    e_move_distance = e_diff/distance;
    current_step_count++;
  }else if(is_circular_motion){
    center_x = start_x_coord + gcode.get_i();
    center_y = start_y_coord + gcode.get_j();
    radius = sqrt(pow((center_x - start_x_coord) , 2) + pow((center_y - start_y_coord) , 2));
    start_angle = atan2((start_y_coord - center_y), (start_x_coord - center_x));
    float end_angle = atan2((end_y_coord - center_y), (end_x_coord - center_x));
    float arc_angle = end_angle - start_angle;
    if(gcode.get_code() == 2){ // ClockWise
      arc_angle = (arc_angle > 0) ?  ((2 * (22.0/7.0)) - arc_angle): abs(arc_angle);
    }
    if(gcode.get_code() == 3){ // Anti ClockWise
      arc_angle = (arc_angle < 0) ? (arc_angle + (2 * (22.0/7.0))) : arc_angle;
    }
    arc_angle = (arc_angle == 0) ? (2 * (22.0/7.0)) : arc_angle;
    
    float arc_length = arc_angle * radius;
    distance = arc_length;
    distance_in_steps = distance *STEPS_PER_MM;
    ARC_SEGMENT_LENGTH_NEW = ARC_SEGMENT_LENGTH;   // NEED TO INVESTIGATE THIS
    float delta_arc_angle = (ARC_SEGMENT_LENGTH_NEW/radius);
    delta_cos = cos(delta_arc_angle);
    delta_sin = sin(delta_arc_angle);

/*
    Serial.println(start_angle,DEC);
    Serial.println(end_angle,DEC);
        
    Serial.println(center_x,DEC);
    Serial.println(center_y,DEC);
  
    Serial.println(radius,DEC);
    Serial.println(arc_angle,DEC);
    Serial.println(distance,DEC);
    Serial.println(ARC_SEGMENT_LENGTH_NEW,DEC);
    Serial.println(delta_arc_angle,DEC);
    Serial.println(delta_cos,DEC);
    Serial.println(delta_sin,DEC);
    */
   /* 
    if(radius > 50) { // HIJACK from circular to straight line
      is_linear_motion = true;
      preprocess();
    }
    */
  }

  set_direction();
  unsigned char sreg;
  sreg = SREG;
  cli();
  OCR1A = (unsigned int)motion_timer_count;
  SREG = sreg;
  TCCR1B = (1<<WGM12) | (1<<CS10);
  sei();
}

void reset_move_distances(){
  travelled = 0.0f;
  travelled_in_steps = 0;
  x_travelled = 0.0f;
  y_travelled = 0.0f;
  z_travelled = 0.0f;
  e_travelled = 0.0f;
  current_step_count = 0;
  quadrant = -1;
  gcode.set_x(NEGATIVE_THOUSAND);
  gcode.set_y(NEGATIVE_THOUSAND);
  gcode.set_z(NEGATIVE_THOUSAND);
  gcode.set_e(NEGATIVE_THOUSAND);
  
}

void set_direction(){
  xStepper.set_direction(absolute_positioning);
  yStepper.set_direction(absolute_positioning);
  zStepper.set_direction(absolute_positioning);
  eStepper.set_direction(absolute_positioning);
}

void disable_all_motors(){
  xStepper.disable();
  yStepper.disable();
  zStepper.disable();
  eStepper.disable();
}
void enable_all_motors(){
  xStepper.enable();
  yStepper.enable();
  zStepper.enable();
  eStepper.enable();
}

void update_previous_move_positions(){
  if(is_circular_motion){
    float ap = xStepper.get_previous_position();
    float ep = gcode.get_x();
    if(abs(ap-ep) > 0.1){
      Serial.print(gcode.get_line_no());
      Serial.print("----");
      Serial.println(abs(ap-ep),DEC);
    }
  }
  
  xStepper.set_previous_position(absolute_positioning);
  yStepper.set_previous_position(absolute_positioning);
  zStepper.set_previous_position(absolute_positioning);
  eStepper.set_previous_position(absolute_positioning);
}

void reset_for_next_move(){
  TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
  TIFR1 |= (1 << OCF1A);
  isr_completed = true;
  update_previous_move_positions(); 
  is_linear_motion = false;
  is_circular_motion = false;
  disable_all_motors();
  reset_move_distances();
  gcode.reset();
  x_increment = 0.0f, y_increment = 0.0f, z_increment = 0.0f, e_increment = 0.0f;
  x_travelled = 0.0f, y_travelled = 0.0f, z_travelled = 0.0f, e_travelled = 0.0f;
  Serial.println("ok");
}

void actuate_linear_motion(){
  float delta_x = (travelled * x_move_distance) - x_travelled;
  float delta_y = (travelled * y_move_distance) - y_travelled;
  float delta_z = (travelled * z_move_distance) - z_travelled;
  float delta_e = (travelled * e_move_distance) - e_travelled;
  if(abs(delta_x) > MM_PER_STEP){
    x_increment = x_increment + MM_PER_STEP * xStepper.get_direction(absolute_positioning);
    x_travelled = x_travelled + MM_PER_STEP;
    xStepper.step();
  }
  if(abs(delta_y) > MM_PER_STEP){
    y_increment = y_increment + MM_PER_STEP * yStepper.get_direction(absolute_positioning);
    y_travelled = y_travelled + MM_PER_STEP;
    yStepper.step();
  }
  if(abs(delta_z) > MM_PER_STEP){
    z_increment = z_increment + MM_PER_STEP * zStepper.get_direction(absolute_positioning);
    z_travelled = z_travelled + MM_PER_STEP;
    zStepper.step();
  }
  if(abs(delta_e) > MM_PER_STEP_EXTRUDER){
    e_increment = e_increment + MM_PER_STEP_EXTRUDER * eStepper.get_direction(absolute_positioning);
    e_travelled = e_travelled + MM_PER_STEP_EXTRUDER;
    eStepper.step();
  }
}

void handle_direction(float delta_x, float delta_y){
  if(delta_x < 0 && delta_y > 0 && quadrant != 1){
    quadrant = 1;
    
    if(gcode.get_code() == 3){
      //Serial.println("Q1");
      xStepper.set_clockwise();
      yStepper.set_clockwise();
    } else {
      //Serial.println("Q4");
      xStepper.set_clockwise();
      yStepper.set_clockwise();
    }
  }else if(delta_x < 0 && delta_y < 0 && quadrant != 2){
    quadrant = 2;
    
    if(gcode.get_code() == 3){
      //Serial.println("Q2");
      xStepper.set_clockwise();
      yStepper.set_anti_clockwise();
    } else {
      //Serial.println("Q3");
      xStepper.set_clockwise();
      yStepper.set_anti_clockwise();      
    }
  }else if(delta_x > 0 && delta_y < 0 && quadrant != 3){
    quadrant = 3;
    
    if(gcode.get_code() == 3){
      //Serial.println("Q3");
      xStepper.set_anti_clockwise();
      yStepper.set_anti_clockwise();
    } else {
      //Serial.println("Q2");
      xStepper.set_anti_clockwise();
      yStepper.set_anti_clockwise();
    }
  }else if(delta_x > 0 && delta_y > 0  && quadrant != 4){
    quadrant = 4;
    
    if(gcode.get_code() == 3){
      //Serial.println("Q4");
      xStepper.set_anti_clockwise();
      yStepper.set_clockwise();
    } else {
      //Serial.println("Q1");
      xStepper.set_anti_clockwise();
      yStepper.set_clockwise();
    }
  }
  //Serial.println(quadrant);
}

void actuate_circular_motion(){

  float x_prev = xStepper.get_previous_position();
  float y_prev = yStepper.get_previous_position();

  int direction = (gcode.get_code() == 2) ? -1 : 1;

  float sweep_angle = (travelled/radius);
  float angle = (start_angle + sweep_angle * direction);
  float x = center_x + (radius * cos(angle));
  float y = center_y + (radius * sin(angle));

  /*
  if(i % 1000){
    Serial.println(x);
  }
  i++;
  */
  //Serial.println(y);
  float delta_x = x - x_prev;
  float delta_y = y - y_prev;

  handle_direction(delta_x, delta_y);
  if(abs(delta_x) > MM_PER_STEP){
    xStepper.step();
    xStepper.set_previous_position(x);
  }
  if(abs(delta_y) > MM_PER_STEP){
    yStepper.step();
    yStepper.set_previous_position(y);
  }
}


bool skip(){
  if(!is_linear_motion) return false; 
  if(accel_steps >= 0){
    accel_steps--;
    current_accel_step++;
    float time = (time_increment * current_accel_step);
    float expected_velocity = ACC_PROFILE * time;
    expected_distance = expected_distance + (expected_velocity * time_increment);
    return ((travelled + MM_PER_STEP) > expected_distance);
  }else if(current_step_count>deceleration_start_step){
    float time = time_increment * (current_step_count-deceleration_start_step);
    float expected_velocity = gcode.get_feed_rate()-(ACC_PROFILE * time);
    expected_deceleration_distance = expected_deceleration_distance + (expected_velocity * time_increment);
    float actual_deceleration_distance = MM_PER_STEP * (current_step_count-deceleration_start_step);
    return (actual_deceleration_distance - expected_deceleration_distance)>MM_PER_STEP;
  }
  return false;
}

bool skip2(){
  return false;
}


ISR(TIMER1_COMPA_vect){
  if(travelled  < distance){
  //if(travelled_in_steps < distance_in_steps){
    isr_completed = false;
    enable_all_motors();
    xStepper.check_endstop();
    yStepper.check_endstop();
    zStepper.check_endstop();
    if(is_linear_motion){
      //skip();
      if(!skip()){
        travelled = travelled + MM_PER_STEP;
        travelled_in_steps++;
        actuate_linear_motion();
      }else{
        // Keep steppermotors disabled during skip phase
        xStepper.disable();
        yStepper.disable();
        zStepper.disable();
        eStepper.disable();
      }        
    }else if(is_circular_motion){
      travelled = travelled + ARC_SEGMENT_LENGTH_NEW;
      travelled_in_steps++;
      actuate_circular_motion();
    }    
  }else{
    reset_for_next_move();
  }
}

void init_motion_timer(){
  TCCR1A = 0;
  TIMSK1=  (1<<OCIE1A);
}

void home(){
    xStepper.home();
    yStepper.home();
    zStepper.home();
    update_previous_move_positions(); 
    reset_move_distances();
    gcode.reset();
}

bool is_motion_command(){
  uint8_t code = gcode.get_code();
  char type =  gcode.get_type();
  is_linear_motion = ((type == 'G') && (code == 0 || code == 1));
  is_circular_motion = (type == 'G') && (code == 2 || code == 3);
  if(is_linear_motion || is_circular_motion ) return true;
  return false;
}

void process(int index){
  
  strlcpy(temp_string, ringBuffer.fetch(index), sizeof(temp_string));
  //Serial.println(temp_string);
  parser.parse_gcode(temp_string);
  if(gcode.get_type() == 'G'){
    switch(gcode.get_code()){
      case 28:
        home();
        break;
      case 90:
        absolute_positioning = true;
        break;
      case 91:
        absolute_positioning = false;  
        break;
    }
  }
  if(is_motion_command()){
    preprocess();
  }else{
    Serial.println("ok");
  }

}

void setup(){
  Serial.begin(115200);
  init_motion_timer();
}

void handle_touch_plate(){
  //zStepper.set_endstop_check_disabled();
  if(digitalRead(Z_HEIGHT_PROBE) == LOW && !plate_touched){
    plate_touched = true;
    Serial.print("ok Touched ");
    Serial.println(zStepper.get_previous_position());
  }
  if(digitalRead(Z_HEIGHT_PROBE) == HIGH && plate_touched){
    plate_touched = false;
  }
}


void loop(){
  if(PCB_PRINTING) handle_touch_plate();
  serialCommunication.handle();
  if(isr_completed){
    int next_read = ringBuffer.get_next_buffer_index_to_read();
    if(next_read > -1) process(next_read);
  }
}
