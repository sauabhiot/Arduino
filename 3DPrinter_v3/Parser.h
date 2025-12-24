#ifndef PARSER_H
#define PARSER_H
#include "Gcode.h"

class Parser{
  private:
    Gcode* gcode;
  public:
    Parser(Gcode* gcd);
    void parse_gcode(char* gcode_line);
};

#endif