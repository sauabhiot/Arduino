#include "Gcode.h"

  Gcode::Gcode(){

  };

  float Gcode::get_x(){
    return x_input;
  }

  long Gcode::get_line_no(){
    return line_no;
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

  float Gcode::get_i(){
    return i_input;
  }

  float Gcode::get_j(){
    return j_input;
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

  void Gcode::set_line_no(long ln){
    line_no = ln;
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

  void Gcode::set_i(float i){
    i_input = i;
  }

  void Gcode::set_j(float j){
    j_input = j;
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

  void Gcode::reset(){
    x_input = NEGATIVE_THOUSAND;
    y_input = NEGATIVE_THOUSAND;
    z_input = NEGATIVE_THOUSAND;
    e_input = NEGATIVE_THOUSAND;
    
    i_input = 0.0f;
    j_input = 0.0f;
    type = 'X';
    code = -1;
  }