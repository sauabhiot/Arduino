#include "Parser.h"
#include <Arduino.h>
Parser::Parser(Gcode* gcd){
  gcode = gcd;
};

void Parser::parse_gcode(char* gcode_line){
  char *token;
  token = strtok(gcode_line, " ");
  size_t length = strlen(token);
  while (token != NULL) {
    char type = token[0];
    char* val= token+1;
    switch(type){
      case 'N':
        gcode->set_line_no(atoi(val));
        break;
      case 'G':
        gcode->set_type('G');
        gcode->set_code(atoi(val));
        break;
      case 'X':
        gcode->set_x((atof(val) == 0) ? MM_PER_STEP : atof(val));
        break;    
      case 'Y':
        gcode->set_y((atof(val) == 0) ? MM_PER_STEP : atof(val));
        break;    
      case 'Z':
        gcode->set_z((atof(val) == 0) ? MM_PER_STEP : atof(val));
        break;  
      case 'E':
        gcode->set_e((atof(val) == 0) ? MM_PER_STEP : atof(val));
        break;    
      case 'I':
        gcode->set_i(atof(val));
        break;    
      case 'J':
        gcode->set_j(atof(val));
        break;               
      case 'F':
        gcode->set_feed_rate(atof(val));
        break;    
      case 'P':
        gcode->set_print_mode(atoi(val));
        break;    
    }
    token = strtok(NULL, " ");
  }

}