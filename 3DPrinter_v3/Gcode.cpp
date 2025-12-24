#include "Gcode.h"

  Gcode::Gcode(){

  };

  float Gcode::get_x(){
    return x_input;
  }
  
  float Gcode::get_y(){
    return y_input;
  }

  float Gcode::get_z(){
    return z_input;
  }

  float Gcode::get_e(){
    return e_input;
  }

  uint8_t Gcode::get_code(){
    return code;
  }

  char Gcode::get_type(){
    return type;
  }

  float Gcode::get_feed_rate(){
    return (feed_rate == 0) ? DEFAULT_FEED_RATE : feed_rate;
  }

  bool Gcode::get_print_mode(){
    return print_mode_3d;
  }

  void Gcode::set_x(float x){
    x_input = x;
  }
  
  void Gcode::set_y(float y){
    y_input = y;
  }

  void Gcode::set_z(float z){
    z_input = z;
  }

  void Gcode::set_e(float e){
    e_input = e;
  }
  
  void Gcode::set_code(uint8_t c){
    code = c;
  }

  void Gcode::set_type(char t){
    type = t;
  }

  void Gcode::set_feed_rate(float f){
    feed_rate = f;
  }

  void Gcode::set_print_mode(bool pm3d){
    print_mode_3d = pm3d;
  }