#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>

#define base_frequency 16000000
#define steps_pre_revolution 200
#define microstepping 16
#define screw_pitch 2
#define no_of_starts 4
#define clock_prescalar 1
#define timer_frequency base_frequency/clock_prescalar

#define total_gcodes 3

double count=0.0;
long times=0;
double time_elapsed=0.0;

long steps_per_mm = (steps_pre_revolution * microstepping)/(screw_pitch * no_of_starts); // (200 *16)/(2*4) = 400
double mm_per_step = 1.00/(double)steps_per_mm; // 0.000625

volatile double time_increment = 0.0;


/*
 // Anti clockwise 0 - 90
char *gcodes[total_gcodes] = {
  "G01 X100 Y100 F1000",
  "G01 X150 Y100 F1000",
  "G03 X100 Y150 I-50 J0",
  
};
*/

 /*
 // Anti clockwise 90 - 180
char *gcodes[total_gcodes] = {
  "G01 X100 Y100 F1000",
  "G01 X100 Y150 F1000",
  "G03 X50 Y100 I0 J-50",
  
};
*/

/*
// Anti clockwise Semi circle 0 - 180

char *gcodes[total_gcodes] = {
  "G01 X100 Y100 F1000",
  "G01 X150 Y100 F1000",
  "G03 X100 Y150 I-50 J0",
  "G03 X50 Y100 I0 J-50",
};
*/


/*
// Anti clockwise 180 - 270
char *gcodes[total_gcodes] = {
  "G01 X100 Y100 F1000",
  "G01 X50 Y100 F1000",
  "G03 X100 Y50 I50 J0",
  
};

*/

/*
// Anti clockwise 270 - 360
char *gcodes[total_gcodes] = {
  "G01 X100 Y100 F1000",
  "G01 X100 Y50 F1000",
  "G03 X150 Y100 I0 J50",
  
};
*/




/*
// Anti clockwise Full circle 0 - 360

char *gcodes[total_gcodes] = {
  "G01 X100 Y100 F1000",
  "G01 X150 Y100 F1000",
  "G03 X100 Y150 I-50 J0",
  "G03 X50 Y100 I0 J-50",
  "G03 X100 Y50 I50 J0",
  "G03 X150 Y100 I0 J50",
};
*/
/* // anti clockwise circle
char *gcodes[total_gcodes] = {
  "G01 X100 Y100 F1000",
  "G01 X149.9 Y100.1 F1000",
  "G02 X149.9 Y99.9 I-50 J0",
};
*/
 // Clockwise circle
char *gcodes[total_gcodes] = {
  "G01 X100 Y100 F1000",
  "G01 X149.9 Y99.9 F1000",
  "G02 X149.9 Y100.1 I-49 J1",
};


/*
// From 340 to 20 Anti clockwise
char *gcodes[total_gcodes] = {
   "G01 X100 Y100 F1000",
   "G01 X146.61 Y82.81 F1000",
   "G03 X146.66 Y117.03 I-46 J18",
};
*/

/*
// From 340 to 95 Anti clockwise
char *gcodes[total_gcodes] = {
   "G01 X100 Y100 F1000",
   "G01 X146.61 Y82.81 F1000",
   "G03 X95.64 Y149.81 I-46.6 J18.2",
};
*/

/*
// From 340 to 95 Anti clockwise
char *gcodes[total_gcodes] = {
   "G01 X100 Y100 F1000",
   "G01 X150 Y100 F1000",
   "G03 X146.61 Y82.81 I-50 J0",
};
*/
/*
// Equilateral Triange of 5 cm
char *gcodes[total_gcodes] = {
  "G01 X100 Y100 F1000",
  "G01 X150 Y100",
  "G01 X125 Y143.3",
  "G01 X100 Y100",
};
*/

/* // Square
char *gcodes[total_gcodes] = {
  "G01 X50 Y0 F1000",
  "G01 X50 Y50",
  "G01 X0 Y50",
  "G01 X0 Y0"
};
*/
int c=1;
int gcode_indx = 0 , prev_gcode_indx = 0;
int end_gcode_sub_indx = 0 , gcode_sub_indx = 0 , prev_gcode_sub_indx = 0;
volatile double x=0,y=0,e=0,f=0;
volatile double prev_x=0;
volatile double prev_y=0;
volatile double duration=0;
volatile double distance=0;
volatile double travelled=0;;
double slope = 0.0;
double dY = 0.0;
double dX = 0.0;
double p = 0.0;
int x_primary=0;

double ins_x = 0, ins_y = 0;
double p_x = 0, p_y = 0;
double gcode_step_x = 0, gcode_step_y = 0;

double formula_denominator;
int g_code=1;

double chord_length=0.0;
double arc_length=0.0;
double radius=0.0;
double I=0.0,J=0.0;
double arc_center_x=0;
double arc_center_y=0;
double theta;
int x_previouse=0;
int y_previous=0;

long count2=0;

int current_quadrant = 1, start_quadrant = 0, end_quadrant = 0;

int clockwise = 0;

double q1_x1_limit = 0;
double q1_y1_limit = 0;

double q1_x2_limit = 0;
double q1_y2_limit = 0;

double q2_x1_limit = 0;
double q2_y1_limit = 0;

double q2_x2_limit = 0;
double q2_y2_limit = 0;


double q3_x1_limit = 0;
double q3_y1_limit = 0;

double q3_x2_limit = 0;
double q3_y2_limit = 0;

double q4_x1_limit = 0;
double q4_y1_limit = 0;

double q4_x2_limit = 0;
double q4_y2_limit = 0;

double intermediate_x = 0.0, intermediate_y = 0.0;

void delay_1us_nop() {
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
}


ISR(TIMER1_COMPA_vect){
  if(travelled>distance){
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); 
    if(gcode_indx<total_gcodes-1 && end_gcode_sub_indx == gcode_sub_indx) gcode_indx++;
    //Serial.println(travelled);
    //Serial.println(p_x);
    //Serial.println(p_y);
    //Serial.println(count2);
    
    
    travelled=0.0;
    ins_x=0.0;
    ins_y=0.0;
    gcode_step_x = 0.0;
    gcode_step_y = 0.0;
    PORTB |= (1 << 0);
    if(g_code == 2 || g_code == 3) next_quadrant();
  }else{
    //Serial.println(get_direction_x());
    travelled = travelled + mm_per_step;
    PORTB &= ~(1<<0);  
    if(g_code==1){
      ins_x =  (travelled/formula_denominator);
      ins_y =    ((slope * (ins_x )));
    
      
      if(abs(ins_x)>mm_per_step){      
        if(abs(ins_x - gcode_step_x) >= mm_per_step){
          p_x = p_x + mm_per_step * get_direction_x();
          gcode_step_x = gcode_step_x + (mm_per_step  * get_direction_x());
          step_x();
        }
      }

    
      if(abs(ins_y)>mm_per_step){
        if(abs(ins_y - gcode_step_y) >= mm_per_step){
          p_y = p_y + mm_per_step*get_direction_y();
          gcode_step_y = gcode_step_y + (mm_per_step * get_direction_y());
          step_y();
        }
      }
    }else{
      double arc_theta = travelled/radius;
      if(current_quadrant == 1) { // X INCREASING Y DECREASING CLOCKWISE 
        if(clockwise){
          if(current_quadrant == start_quadrant){
            ins_x= arc_center_x + cos(theta - arc_theta)*radius;
            ins_y= arc_center_y + sin(theta - arc_theta)*radius;
          }else{
            ins_x= arc_center_x + sin(arc_theta)*radius;
            ins_y= arc_center_y + cos(arc_theta)*radius;
          }          
        }else{  // X DECREASING Y INCREASING ANTI CLOCKWISE
          if(current_quadrant == start_quadrant){
            ins_x= arc_center_x + sin(theta - arc_theta)*radius;
            ins_y= arc_center_y + cos(theta - arc_theta)*radius;
          }else{
            ins_x= arc_center_x + cos(arc_theta)*radius;
            ins_y= arc_center_y + sin(arc_theta)*radius;
          }
        }
      }else if(current_quadrant == 2){ // BOTH INCREASING CLOCKWISE
        if(clockwise){
          if(current_quadrant == start_quadrant){
            ins_x= arc_center_x - sin(theta - arc_theta)*radius;
            ins_y= arc_center_y + cos(theta - arc_theta)*radius;
          }else{
            ins_x= arc_center_x - cos(arc_theta)*radius;
            ins_y= arc_center_y + sin(arc_theta)*radius;
          }

        }else{ // BOTH DECREASING ANTI CLOCKWISE
          if(current_quadrant == start_quadrant){
            ins_x= arc_center_x - cos(theta - arc_theta)*radius;
            ins_y= arc_center_y + sin(theta - arc_theta)*radius;
          }else{
            ins_x= arc_center_x - sin(arc_theta)*radius;
            ins_y= arc_center_y + cos(arc_theta)*radius;
          }
        }
      }else if(current_quadrant == 3) { 
          if(clockwise){ // X INCREASING  Y DECREASING  CLOCKWISE
            if(current_quadrant == start_quadrant){
              ins_x= arc_center_x - cos(theta - arc_theta)*radius;
              ins_y= arc_center_y - sin(theta - arc_theta)*radius;
            }else{
              ins_x= arc_center_x - sin(arc_theta)*radius;
              ins_y= arc_center_y - cos(arc_theta)*radius;
            }
          }else{ // X DECREASING  Y INCREASING ANTI CLOCKWISE
            if(current_quadrant == start_quadrant){
              ins_x= arc_center_x - sin(theta - arc_theta)*radius;
              ins_y= arc_center_y - cos(theta - arc_theta)*radius;

            }else{
              ins_x= arc_center_x - cos(arc_theta)*radius;
              ins_y= arc_center_y - sin(arc_theta)*radius;
            }
          }
      }else if(current_quadrant == 4) { 
          if(clockwise) {// clockwise BOTH DECREASING
            if(current_quadrant == start_quadrant){
              ins_x= arc_center_x + sin(theta - arc_theta)*radius;
              ins_y= arc_center_y - cos(theta - arc_theta)*radius;
            }else{
              ins_x= arc_center_x + cos(arc_theta)*radius;
              ins_y= arc_center_y - sin(arc_theta)*radius;
            }
          }else{ // Anti clockwise BOTH INCREASING
            if(current_quadrant == start_quadrant){
              ins_x= arc_center_x + cos(theta - arc_theta)*radius;
              ins_y= arc_center_y - sin(theta - arc_theta)*radius;
            }else{
              ins_x= arc_center_x + sin(arc_theta)*radius;
              ins_y= arc_center_y - cos(arc_theta)*radius;
            }
          }
      }  
      
      
      
      //Serial.println(abs(abs(ins_y) - abs(p_y)),DEC);
      
      
      if(abs(abs(ins_x) - abs(p_x)) >= mm_per_step){
        p_x = p_x + (mm_per_step  * get_direction_x());
        step_x();
        
      }

      //step_theta = get_direction_y() > 0 ? (theta-step_theta):step_theta;
      //ins_y= arc_center_y + sin(arc_theta)*radius;
      //Serial.println(abs(abs(ins_y) - abs(p_y)),DEC);
      if(abs(abs(ins_y) - abs(p_y)) >= mm_per_step){
        p_y = p_y + (mm_per_step * get_direction_y());
        step_y();
        
      }
      
    }
  }
} 

void step_x(){
    PORTD |= (1 << 2);
    delay_1us_nop();
    PORTD &= ~(1 << 2);
}

void step_y(){
    PORTD |= (1 << 3);
    delay_1us_nop();
    PORTD &= ~(1 << 3);
}


void set_quadrants(){
  q1_x1_limit = (arc_center_x + radius);
  q1_y1_limit = (arc_center_y);

  q1_x2_limit = (arc_center_x);
  q1_y2_limit = (arc_center_y + radius);

  q2_x1_limit = (arc_center_x);
  q2_y1_limit = (arc_center_y + radius);

  q2_x2_limit = (arc_center_x - radius);
  q2_y2_limit = (arc_center_y);


  q3_x1_limit = (arc_center_x - radius);
  q3_y1_limit = (arc_center_y);

  q3_x2_limit = (arc_center_x);
  q3_y2_limit = (arc_center_y - radius);

  q4_x1_limit = (arc_center_x);
  q4_y1_limit = (arc_center_y - radius);

  q4_x2_limit = (arc_center_x + radius);
  q4_y2_limit = (arc_center_y);

  if((q1_x1_limit >= prev_x) && (prev_x >= q1_x2_limit) && (q1_y1_limit <= prev_y) && (prev_y <= q1_y2_limit)){
    start_quadrant = 1;
  }else if((q2_x1_limit >= prev_x) && (prev_x >= q2_x2_limit) && (q2_y1_limit >= prev_y) && (prev_y >= q2_y2_limit)){
    start_quadrant = 2;
  }else if((q3_x1_limit <= prev_x) && (prev_x <= q3_x2_limit) && (q3_y1_limit >= prev_y) && (prev_y >= q3_y2_limit)){
    start_quadrant = 3;
  }else if((q4_x1_limit <= prev_x) && (prev_x <= q4_x2_limit) && (q4_y1_limit <= prev_y) && (prev_y <= q4_y2_limit)){
    start_quadrant = 4;
  }


  if((q1_x1_limit >= x) && (x >= q1_x2_limit) && (q1_y1_limit <= y) && (y <= q1_y2_limit)){
    end_quadrant = 1;
  }else if((q2_x1_limit >= x) && (x >= q2_x2_limit) && (q2_y1_limit >= y) && (y >= q2_y2_limit)){
    end_quadrant = 2;
  }else if((q3_x1_limit <= x) && (x <= q3_x2_limit) && (q3_y1_limit >= y) && (y >= q3_y2_limit)){
    end_quadrant = 3;
  }else if((q4_x1_limit <= x) && (x <= q4_x2_limit) && (q4_y1_limit <= y) && (y <= q4_y2_limit)){
    end_quadrant = 4;
  }


  int start_indx = (g_code == 3 && start_quadrant== 4) ? 0 : start_quadrant;
  start_indx = (g_code == 2 && start_quadrant== 1) ? 0 : start_quadrant;
  int end_indx = end_quadrant;
  end_gcode_sub_indx = (end_indx - start_indx) ;
  current_quadrant = start_quadrant;
  //Serial.println(start_quadrant);
  //Serial.println(end_quadrant);
}

void set_duration(){
  
  if(g_code == 1){
    double c_x = x - p_x;
    double c_y = y - p_y;
    distance = sqrt((c_x * c_x) + (c_y * c_y));
    duration = distance/f;
    dY = (y-prev_y);
    dX =  (x-prev_x)==0?0.0000001:(x-prev_x);
    slope = ((double)dY/dX);
    formula_denominator =sqrt(1+slope*slope);
    
  }else{
    clockwise = (g_code==2) ? 1 : 0;
    arc_center_x = I + prev_x;
    arc_center_y = J + prev_y;
    //Serial.println(arc_center_x);
    //Serial.println(arc_center_y);
    intermediate_x = x;
    intermediate_y = y;
    radius = sqrt(pow(prev_x - arc_center_x,2) + pow(prev_y - arc_center_y,2));  
    set_quadrants();
    process_quadrant_arc();
  }
  set_direction();
}

void next_quadrant(){
  if(current_quadrant!=end_quadrant){
    if(g_code == 3)
      current_quadrant = (current_quadrant == 4) ? 1 : (current_quadrant + 1);
    else if(g_code == 2)
      current_quadrant = (current_quadrant == 1) ? 4 : (current_quadrant - 1);
    gcode_sub_indx++;
  }
}

void set_intermediate_coordinates(){
  switch (current_quadrant) {
    case 1:
      x = clockwise ? q1_x1_limit : q1_x2_limit;
      y = clockwise ? q1_y1_limit : q1_y2_limit;
    break;
    case 2:
      x = clockwise ? q2_x1_limit : q2_x2_limit;
      y = clockwise ? q2_y1_limit : q2_y2_limit;
    break;
    case 3:
      x = clockwise ? q3_x1_limit : q3_x2_limit;
      y = clockwise ? q3_y1_limit : q3_y2_limit;
    break;
    case 4:
      x = clockwise ? q4_x1_limit : q4_x2_limit;
      y = clockwise ? q4_y1_limit : q4_y2_limit;
    break;

  }

}

void process_quadrant_arc(){
  if((current_quadrant == start_quadrant) && (current_quadrant == end_quadrant)){
    //intermediate_x = x;
    //intermediate_y = y;
  }else if(current_quadrant != end_quadrant){
    if(current_quadrant == start_quadrant){
      set_intermediate_coordinates();
    }else{
      prev_x = x;
      prev_y = y;
      set_intermediate_coordinates();
    }
  }else{
    if(current_quadrant!=start_quadrant){
      prev_x = x;
      prev_y = y;
      x = intermediate_x;
      y = intermediate_y;
    }
  }
  set_sub_circular_motion_params();
}



void set_sub_circular_motion_params(){
    chord_length = sqrt(pow(prev_x-x,2) + pow(prev_y-y,2));
    theta = (asin(chord_length/(2*radius)))*2;
    arc_length = theta * radius;
    distance = arc_length;
    Serial.println(theta);
    set_direction();
}

void set_target_freq(){
  count =  round(timer_frequency/(steps_per_mm * f))-1;
  //Serial.println(count);
  //time_increment = 1.0 / ((steps_per_mm) * f);
  time_increment = (1.0 / (timer_frequency)) * (count+1);
  //Serial.println(time_increment,DEC);
  unsigned char sreg;
  sreg = SREG;
  cli();
  OCR1A = (unsigned int)count;
  SREG = sreg;
  sei();
}

void set_clockwise_for_x(){
  PORTD |= (1 << 5); //X axis dir
}

void set_anti_clockwise_for_x(){
  PORTD &= ~(1 << 5); //X axis dir
}

void set_clockwise_for_y(){
  PORTD |= (1 << 6); //X axis dir
}

void set_anti_clockwise_for_y(){
  PORTD &= ~(1 << 6); //Y axis dir
}


void set_direction(){
  if((x - p_x) < 0){ 
    set_anti_clockwise_for_x();
  }else if((x - p_x) > 0){
    set_clockwise_for_x();
  }
  if((y-p_y)<0){ 
    set_clockwise_for_y();
  }else if((y - p_y) > 0){
    set_anti_clockwise_for_y();
  }
}

int get_direction_x(){
  double inc = x - p_x;
  //Serial.println(inc);
  if(inc < 0)
    return -1;
  if(inc > 0)
    return 1;
  else
    return 0;
  //return (inc < mm_per_step) ? -1 : (inc > mm_per_step) ? 1 : 0;
}

int get_direction_y(){
  double inc = y - p_y;
  if(inc < 0)
    return -1;
  if(inc > 0)
    return 1;
  else
    return 0;
  //return (inc < mm_per_step) ? -1 : (inc > mm_per_step) ? 1 : 0;
}


void parse_gcodes(int indx){
  char temp_string[50];
  strcpy(temp_string, gcodes[indx]);
  char *token;
  const char *delimiter = " ";
  token = strtok(temp_string, delimiter);

  while (token != NULL) {
    char type = token[0];
    char* val= token+1;
    switch(type){
      case 'G':
        g_code=atoi(val);
        break;
      case 'X':
        prev_x=x;
        x=atoi(val);
        break;
      case 'Y':
        prev_y=y;
        y=atoi(val);
        break;
      case 'I':
        I=atoi(val);
        break;
      case 'J':
        J=atoi(val);
        break;            
      case 'E':
        e=atoi(val);
        break;
      case 'F':
        f=atoi(val)/60;
        break;
    }
    token = strtok(NULL, delimiter); // Get the next token
    
  }

  //Serial.println(indx);
  set_target_freq();
  set_duration();  
  
}

void printXY(double c_x, double c_y, double r, double angle){
  double x = c_x + r * cos(angle);
  double y = c_y + r * sin(angle);
  Serial.println(x,DEC);
  Serial.println(y,DEC);

}

void setup() {
  Serial.begin(115200);
  //delay(1000);
  DDRD |= (1<<2);  // pin 2 step pin for X axis
  DDRD |= (1<<5); // pin 5 dir pin for X axis

  DDRD |= (1<<3);  // pin 3 step for Y axis
  DDRD |= (1<<6); // pin 6 dir for Y axis
  
  DDRB |= (1<<0); // pin 8 enable
  PORTB &= ~(1<<0);  


  //DISABLED FOR TESTING, remove to start testing
  //PORTB |= (1 << 0);
  
  
  TCCR1A = 0;
  TCCR1B = (1<<WGM12) | (1<<CS10);
  TIMSK1=  (1<<OCIE1A);
  parse_gcodes(0);
  //sei(); 
}

void loop() {
  //Serial.println(travelled);
  if(gcode_indx>prev_gcode_indx){
    parse_gcodes(gcode_indx);
    prev_gcode_indx=gcode_indx;
    TCCR1B = (1<<WGM12) | (1<<CS10);
  }
  if(gcode_sub_indx>prev_gcode_sub_indx){
    //Serial.println(gcode_sub_indx);
    process_quadrant_arc();
    prev_gcode_sub_indx = gcode_sub_indx;
    TCCR1B = (1<<WGM12) | (1<<CS10);
  }
}


