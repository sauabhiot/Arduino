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

volatile void Stepper::delay_1us_nop() {
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop");

}