
#include "Stepper.h"

StepperZ::StepperZ(Gcode* gcd) : Stepper() {
    // Logic specific to Z
  gcode = gcd;
  DDRK |= (1 << 0);   
  PORTK &= ~(1 << 0);
  
  
  DDRL |= (1 << 3);  // PIN 46 as output STEP Pin of Z
  DDRL |= (1 << 1);  // PIN 48 as output DIR Pin of Z
}

void StepperZ::enable(){
	PORTK &= ~(1 << 0);
}

void StepperZ::disable(){
	PORTK |= (1 << 0);
}

void StepperZ::set_clockwise(){
	PORTL &= ~(1 << 1); //Z axis dir
}
void StepperZ::set_anti_clockwise(){
  PORTL |= (1 << 1); //Z axis dir
}

void StepperZ::step(){
	PORTL |= (1 << 3);
	delay_1us_nop();
	PORTL &= ~(1 << 3);
}

void StepperZ::home(){
  enable();
  while(!(PIND & (1 << 2))){ //Z Axis
    set_anti_clockwise();
    step();
    delayMicroseconds(500);
  }
  backoff();
  disable();
}

void StepperZ::backoff(){
  enable();
  int k = 0;
  while(true){
    k++;
    set_clockwise();
    step();
    delayMicroseconds(500);
    if(k > 2000) break;
  }  
  disable();
  set_previous_position(MM_PER_STEP);
  gcode->set_z(MM_PER_STEP);  
}

void StepperZ::moveNozzle(){
  if(gcode->get_type() == 'G' && gcode->get_code() == 0){ //&& !nozzle_lifted){
    move('u');
    nozzle_lifted = true;
  }
  /*
  if(gcode->get_type() == 'G' && (gcode->get_code() == 1 || gcode->get_code() == 2 || gcode->get_code() == 3) && nozzle_lifted){
    move('d');
    nozzle_lifted = false;    
  }  
  */
}

void StepperZ::move(char dir){
  if(dir == 'u') set_clockwise();
  if(dir == 'd')  set_anti_clockwise();

  int k = 0;
  enable();
  while(true){
    step();
    delayMicroseconds(500);
    if(k>=500) break;
    k++;
  }
  disable();
}

float StepperZ::get_previous_position(){
  return previous_position;
}

void StepperZ::set_previous_position(bool absolute_positioning){
  if(gcode->get_z() == NEGATIVE_THOUSAND) return;
  if(absolute_positioning)
    previous_position = gcode->get_z();
  else
    previous_position += gcode->get_z();
}

void StepperZ::set_previous_position(float prev_pos){
  previous_position = prev_pos;
}

float StepperZ::get_coord(bool absolute_positioning){
  if(gcode->get_z() == NEGATIVE_THOUSAND)
    return previous_position;
  else{  
    if(absolute_positioning)
      return gcode->get_z();
    else
      return previous_position + gcode->get_z();
  }
}

int StepperZ::get_direction(bool absolute_positioning){
  double inc = get_coord(absolute_positioning) - previous_position;
  if(inc < MM_PER_STEP)
    return -1;
  if(inc > MM_PER_STEP)
    return 1;
  else
    return 0;
}

void StepperZ::set_direction(bool absolute_positioning){
  if(get_direction(absolute_positioning) < 0){ 
    set_anti_clockwise();
  }else if(get_direction(absolute_positioning) > 0){
    set_clockwise();
  }
}

void StepperZ::set_endstop_check_enabled() {
  endstop_enabled = true;
}

void StepperZ::set_endstop_check_disabled() {
  endstop_enabled = false;
}

void StepperZ::check_endstop(){
  if(endstop_enabled && (PIND & (1 << 2))){
    backoff();
    Serial.print("hit endpoint z, backing off");
  }  
}
