#ifndef GCODE_H
#define GCODE_H
#include <Arduino.h>
#include "Config.h"
class Gcode{

  private:
    uint8_t code = -1;
    char type;
    float x_input;
    float y_input;
    float z_input;
    float e_input;
    float feed_rate = 0;
    bool print_mode_3d = false;

  public:
    Gcode();
    uint8_t get_code();
    char get_type();
    float get_x();
    float get_y();
    float get_z();
    float get_e();
    float get_feed_rate();
    bool get_print_mode();

    void set_code(uint8_t code);
    void set_type(char type);
    void set_x(float x);
    void set_y(float y);
    void set_z(float z);
    void set_e(float e);
    void set_feed_rate(float f);
    void set_print_mode(bool print_mode_3d);



};

#endif