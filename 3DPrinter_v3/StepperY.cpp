
#include "Stepper.h"

StepperY::StepperY(Gcode* gcd) : Stepper() {
    // Logic specific to Y
  gcode = gcd;  
  DDRF |= (1 << 2);   // PIN 56 as output ENABLE Pin of Y
  PORTF &= ~(1 << 2); // ENABLE Pin set to LOW

  DDRF |= (1 << 6);  // PIN 60 as output STEP Pin of Y
  DDRF |= (1 << 7);  // PIN 61 as output DIR Pin of Y		
}

void StepperY::enable(){
	PORTF &= ~(1 << 2);
}

void StepperY::disable(){
	PORTF |= (1 << 2);
}

void StepperY::set_clockwise(){
	PORTF |= (1 << 7); //Y axis dir
}
void StepperY::set_anti_clockwise(){
  PORTF &= ~(1 << 7); //Y axis dir
}

void StepperY::step(){
	PORTF |= (1 << 6);
	delay_1us_nop();
	PORTF &= ~(1 << 6);
}

void StepperY::home(){
  enable();
  while(!(PINJ & (1 << 0))){ //Y Axis
    set_anti_clockwise();
    step();
    delayMicroseconds(100);
  }
  backoff();
  set_previous_position(0.0f);
  gcode->set_y(0.0001f);  
  disable();
}

void StepperY::backoff(){
  enable();
  int k = 0;
  int j = 600;
  while(true){
    k++;
    set_clockwise();
    step();
    delayMicroseconds(100);
    if(k>j) break;
  }
  disable();
}

float StepperY::get_previous_position(){
  return previous_position;
}

void StepperY::set_previous_position(bool absolute_positioning){
  if(absolute_positioning)
    previous_position = gcode->get_y();
  else
    previous_position += gcode->get_y();
}

float StepperY::get_coord(bool absolute_positioning){
  if(absolute_positioning)
    return gcode->get_y();
  else
    return previous_position + gcode->get_y();
}

int StepperY::get_direction(bool absolute_positioning){
  double inc = get_coord(absolute_positioning) - previous_position;
  if(inc < 0)
    return -1;
  if(inc > 0)
    return 1;
  else
    return 0;
}

void StepperY::set_direction(bool absolute_positioning){
  if(get_direction(absolute_positioning) < 0){ 
    set_anti_clockwise();
  }else if(get_direction(absolute_positioning) > 0){
    set_clockwise();
  }
}