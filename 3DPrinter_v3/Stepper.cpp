#include "Stepper.h"
Stepper::Stepper() {
    // Shared initialization if any
}

void Stepper::home() {

}

void Stepper::backoff() {

}

void Stepper::move(char dir) {

}

void Stepper::moveNozzle() {

}

float Stepper::get_coord(bool absolute_positioning){
  return 0.0f;

}

int Stepper::get_direction(bool absolute_positioning){
  return 0;

}

void Stepper::set_direction(bool absolute_positioning){


}

float Stepper::get_previous_position(){
  return previous_position;
}

void Stepper::set_previous_position(float prev_pos){

}

void Stepper::check_endstop(){

}

void Stepper::set_endstop_check_enabled() {};
void Stepper::set_endstop_check_disabled() {};

volatile void Stepper::delay_1us_nop() {
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop");

}