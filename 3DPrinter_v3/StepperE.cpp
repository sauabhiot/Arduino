
#include "Stepper.h"

StepperE::StepperE(Gcode* gcd) : Stepper() {
    // Logic specific to E
  gcode = gcd;  
  DDRA |= (1 << 2);   // PIN 24 as output ENABLE Pin of E
  PORTA &= ~(1 << 2); // ENABLE Pin set to LOW
  //PORTA |= (1 << 2); 

  DDRA |= (1 << 4);  // PIN 26 as output STEP Pin of E
  DDRA |= (1 << 6);  // PIN 28 as output DIR Pin of E		
}

void StepperE::enable(){
	PORTA &= ~(1 << 2); 
}

void StepperE::disable(){
	PORTA |= (1 << 2); 
}

void StepperE::set_clockwise(){
	PORTA &= ~(1 << 6); //E dir
}
void StepperE::set_anti_clockwise(){
  PORTA |= (1 << 6); //E dir
}

void StepperE::step(){
	PORTA |= (1 << 4);
	delay_1us_nop();
	PORTA &= ~(1 << 4);
}

float StepperE::get_previous_position(){
  return previous_position;
}

void StepperE::set_previous_position(bool absolute_positioning){
  if(gcode->get_e() == NEGATIVE_THOUSAND) return;
  if(absolute_positioning)
    previous_position = gcode->get_e();
  else
    previous_position += gcode->get_e();
}

float StepperE::get_coord(bool absolute_positioning){
  if(gcode->get_e() == NEGATIVE_THOUSAND)
    return previous_position;
  else{  
    if(absolute_positioning)
      return gcode->get_e();
    else
      return previous_position + gcode->get_e();
  }
}

int StepperE::get_direction(bool absolute_positioning){
  double inc = get_coord(absolute_positioning) - previous_position;
  if(inc < MM_PER_STEP)
    return -1;
  if(inc > MM_PER_STEP)
    return 1;
  else
    return 0;
}

void StepperE::set_direction(bool absolute_positioning){
  if(get_direction(absolute_positioning) < 0){ 
    set_clockwise();
  }else if(get_direction(absolute_positioning) > 0){
    set_anti_clockwise();
  }
}