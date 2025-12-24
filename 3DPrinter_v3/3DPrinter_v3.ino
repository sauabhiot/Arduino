#include "Config.h"

#include "Stepper.h"
#include "Utils.h"
#include "Gcode.h"
#include "Parser.h"
#include "RingBuffer.h"
#include "SerialCommunication.h"

volatile float travelled = 0.0f;
volatile float distance  = 0.0f;

float x_increment = 0.0f, x_move_distance = 0.0f, x_travelled = 0.0f;
float y_increment = 0.0f, y_move_distance = 0.0f, y_travelled = 0.0f;
float z_increment = 0.0f, z_move_distance = 0.0f, z_travelled = 0.0f;
float e_increment = 0.0f, e_move_distance = 0.0f, e_travelled = 0.0f;

bool absolute_positioning = true;  
bool print_head_lifted = false;

uint16_t timer_counter = 0;
uint16_t timer_increment = 0;
float time_increment = 0.0f;
bool is_linear_motion = true;
long deceleration_start_step = 0;
volatile bool isr_completed = true;

volatile bool plate_touched = false;

uint16_t motion_timer_count = 0;

bool PRINT_MODE_3D_PRINTER = false;

Gcode gcode; 
StepperX xStepper(&gcode);
StepperY yStepper(&gcode);
StepperZ zStepper(&gcode);
StepperE eStepper(&gcode);

Parser parser(&gcode);
RingBuffer ringBuffer;
SerialCommunication serialCommunication(&ringBuffer);

void preprocess(){
  if(!PRINT_MODE_3D_PRINTER) zStepper.moveNozzle();
  float steps_per_sec = STEPS_PER_MM * gcode.get_feed_rate();
  motion_timer_count =  round(TIMER_FREQUENCY/(steps_per_sec)) - 1;
  time_increment = motion_timer_count/TIMER_FREQUENCY;
  float transition_duration = gcode.get_feed_rate()/ACC_PROFILE;
  float accel_steps = round(transition_duration * gcode.get_feed_rate() * STEPS_PER_MM);  
  float z_diff = abs(zStepper.get_coord(absolute_positioning)- zStepper.get_previous_position());
  float e_diff = abs(eStepper.get_coord(absolute_positioning)- eStepper.get_previous_position());
  if(is_linear_motion){
    float x_diff = abs(xStepper.get_coord(absolute_positioning)- xStepper.get_previous_position());
    float y_diff = abs(yStepper.get_coord(absolute_positioning)- yStepper.get_previous_position());
    distance = sqrt((x_diff * x_diff) + (y_diff * y_diff) + (z_diff * z_diff) + (e_diff * e_diff));
    float total_distance_in_steps = round(distance * STEPS_PER_MM);
    float constant_velocity_distance_in_steps = total_distance_in_steps -(2 * accel_steps); 
    deceleration_start_step = accel_steps + constant_velocity_distance_in_steps;
    x_move_distance = x_diff/distance;
    y_move_distance = y_diff/distance;
    z_move_distance = z_diff/distance;
    e_move_distance = e_diff/distance;
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
  x_travelled = 0.0f;
  y_travelled = 0.0f;
  z_travelled = 0.0f;
  e_travelled = 0.0f;
  gcode.set_x(MM_PER_STEP);
  gcode.set_y(MM_PER_STEP);
  gcode.set_z(MM_PER_STEP);
  gcode.set_e(MM_PER_STEP);
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
  disable_all_motors();
  reset_move_distances();
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

ISR(TIMER1_COMPA_vect){
  if(travelled  >= distance){
    reset_for_next_move();
  }else{
    isr_completed = false;
    enable_all_motors();
    travelled = travelled + MM_PER_STEP;
    if(is_linear_motion){
      actuate_linear_motion();
    }else{

    }
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
    reset_move_distances();
}

bool is_motion_command(){
  uint8_t code = gcode.get_code();
  if(code == 0 ||  code == 1 || code == 2 ) return true;
  return false;
}

void process(int index){
  char temp_string[COMMAND_SIZE];
  strlcpy(temp_string, ringBuffer.fetch(index), sizeof(temp_string));
  parser.parse_gcode(temp_string);
  PRINT_MODE_3D_PRINTER = gcode.get_print_mode();
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
