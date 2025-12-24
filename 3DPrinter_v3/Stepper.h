#ifndef STEPPER_H
#define STEPPER_H
#include <Arduino.h>
#include "Gcode.h"
class Stepper{


  public:
    Stepper();
    float previous_position = 0.0f;
    bool nozzle_lifted = false;
    Gcode* gcode;
    virtual void disable();
    virtual void enable();
    virtual void set_clockwise();
    virtual void set_anti_clockwise();
    virtual void step();
    virtual void home();
    virtual void backoff();
    virtual void move(char dir);
    virtual void moveNozzle();
    virtual float get_previous_position();
    virtual void set_previous_position(bool absolute_positioning);
    virtual float get_coord(bool absolute_positioning);
    virtual int get_direction(bool absolute_positioning);
    virtual void set_direction(bool absolute_positioning);
    volatile void delay_1us_nop();
};

class StepperX : public Stepper {
  public:
    StepperX(Gcode* gcode);
    void disable() override;
    void enable() override;
    void set_clockwise() override;
    void set_anti_clockwise() override;
    void step() override;
    void home() override;
    void backoff() override;
    float get_previous_position() override;
    void set_previous_position(bool absolute_positioning) override;
    float get_coord(bool absolute_positioning) override;
    int get_direction(bool absolute_positioning) override;
    void set_direction(bool absolute_positioning) override;
};

class StepperY : public Stepper {
  public:
    StepperY(Gcode* gcode);
    void disable() override;
    void enable() override;
    void set_clockwise() override;
    void set_anti_clockwise() override;
    void step() override;
    void home() override;
    void backoff() override;
    float get_previous_position() override;
    void set_previous_position(bool absolute_positioning) override;
    float get_coord(bool absolute_positioning) override;
    int get_direction(bool absolute_positioning) override;    
    void set_direction(bool absolute_positioning) override;
};

class StepperZ : public Stepper {
  public:
    StepperZ(Gcode* gcode);
    void disable() override;
    void enable() override;
    void set_clockwise() override;
    void set_anti_clockwise() override;
    void step() override;
    void home() override; 
    void move(char dir) override;
    void moveNozzle() override;
    void backoff() override;
    float get_previous_position() override;
    void set_previous_position(bool absolute_positioning) override;
    float get_coord(bool absolute_positioning) override;
    int get_direction(bool absolute_positioning) override;   
    void set_direction(bool absolute_positioning) override;    
};

class StepperE : public Stepper {
  public:
    StepperE(Gcode* gcode);
    void disable() override;
    void enable() override;
    void set_clockwise() override;
    void set_anti_clockwise() override;
    void step() override;
    float get_previous_position() override;
    void set_previous_position(bool absolute_positioning) override;
    float get_coord(bool absolute_positioning) override;
    int get_direction(bool absolute_positioning) override;    
    void set_direction(bool absolute_positioning) override;    
};

#endif