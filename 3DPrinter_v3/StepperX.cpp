
#include "Stepper.h"

StepperX::StepperX(Gcode* gcd) : Stepper() {
  gcode = gcd;
    // Logic specific to X
  DDRD |= (1 << 7);   // PIN 38 as output ENABLE Pin of X
  PORTD &= ~(1 << 7); // ENABLE Pin set to LOW

  DDRF |= (1 << 0);  // PIN 54 as output STEP Pin of X
  DDRF |= (1 << 1);  // PIN 55 as output DIR pin of X		
}

void StepperX::enable(){
	PORTD &= ~(1 << 7);
}

void StepperX::disable(){
	PORTD |= (1 << 7);
}

void StepperX::set_clockwise(){
	PORTF |= (1 << 1);
}
void StepperX::set_anti_clockwise(){
  PORTF &= ~(1 << 1);
}

void StepperX::step(){
	PORTF |= (1 << 0);
	delay_1us_nop();
	PORTF &= ~(1 << 0);	
}

void StepperX::home(){
  enable();
  while(!(PINE & (1 << 5))){ //X Axis
    set_clockwise();
    step();
    delayMicroseconds(100);
  }
  backoff();
  disable();
}

void StepperX::backoff(){
  enable();
  int k=0;
  while(true){
    k++;
    set_anti_clockwise();
    step();
    delayMicroseconds(100);
    if(k>600) break;
  }
  disable();
  set_previous_position(MM_PER_STEP);
  gcode->set_x(MM_PER_STEP);
}

float StepperX::get_previous_position(){
  return previous_position;
}

void StepperX::set_previous_position(bool absolute_positioning){
  if(gcode->get_x() == NEGATIVE_THOUSAND) return;
  if(absolute_positioning)
    previous_position = gcode->get_x();
  else
    previous_position += gcode->get_x();
}

void StepperX::set_previous_position(float prev_pos){
  previous_position = prev_pos;
}

float StepperX::get_coord(bool absolute_positioning){
  if(gcode->get_x() == NEGATIVE_THOUSAND)
    return previous_position;
  else{
    if(absolute_positioning)
      return gcode->get_x();
    else
      return previous_position + gcode->get_x();
  }
}

int StepperX::get_direction(bool absolute_positioning){
  double inc = get_coord(absolute_positioning) - previous_position;
  if(inc < MM_PER_STEP)
    return -1;
  if(inc > MM_PER_STEP)
    return 1;
  else
    return 0;
}

void StepperX::set_direction(bool absolute_positioning){
  if(get_direction(absolute_positioning) < 0){ 
    set_clockwise();
  }else if(get_direction(absolute_positioning) > 0){
    set_anti_clockwise();
  }
}

void StepperX::set_endstop_check_enabled() {
  endstop_enabled = true;
}

void StepperX::set_endstop_check_disabled() {
  endstop_enabled = false;
}

void StepperX::check_endstop(){
  if(endstop_enabled && PINE & (1 << 5)){
    backoff();
    Serial.print("hit endpoint x, backing off");
  }  
}
