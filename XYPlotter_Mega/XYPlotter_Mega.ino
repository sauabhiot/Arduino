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

#define total_gcodes 1



double count=0.0;
long times=0;
double time_elapsed=0.0;

long steps_per_mm = (steps_pre_revolution * microstepping)/(screw_pitch * no_of_starts); // (200 *16)/(2*4) = 400
double mm_per_step = 1.00/(double)steps_per_mm; // 0.000625

volatile double time_increment = 0.0;


char *gcodes[total_gcodes] = {
  "G01 X10 Y0 F1000"
};


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


/*
 // Clockwise circle
char *gcodes[total_gcodes] = {
  "G01 X100 Y100 F500",
  "G01 X149.9 Y99.9",
  "G02 X149.9 Y100.1 I-49 J1",
};
*/

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
/*
 // Square
char *gcodes[total_gcodes] = {
  "G01 X50 Y0 F1000",
  "G01 X50 Y50",
  "G01 X0 Y50",
  "G01 X0 Y0"
};
*/


char *gcodes[total_gcodes] = {
"G01 X0 Y3.269357",
"G01 X1.564718 Y3.377552",
"G01 X1.671153 Y2.920312",
"G01 X1.844874 Y2.520463",
"G01 X2.083436 Y2.179885",
"G01 X2.405187 Y1.88823",
"G01 X2.827257 Y1.635149",
"G01 X3.348422 Y1.423464",
"G01 X3.944213 Y1.262584",
"G01 X4.587717 Y1.16662",
"G01 X5.278933 Y1.134632",
"G01 X5.888182 Y1.159093",
"G01 X6.448495 Y1.231537",
"G01 X6.959872 Y1.353843",
"G01 X7.403964 Y1.517546",
"G01 X7.766087 Y1.717942",
"G01 X8.045021 Y1.954088",
"G01 X8.241987 Y2.215636",
"G01 X8.360657 Y2.492238",
"G01 X8.401028 Y2.782952",
"G01 X8.361879 Y3.072726",
"G01 X8.248104 Y3.336156",
"G01 X8.057256 Y3.575125",
"G01 X7.778321 Y3.788692",
"G01 X7.401517 Y3.978738",
"G01 X6.924395 Y4.144322",
"G01 X6.449719 Y4.264748",
"G01 X5.70345 Y4.421865",
"G01 X4.683142 Y4.616615",
"G01 X3.643259 Y4.826418",
"G01 X2.834597 Y5.027754",
"G01 X2.255934 Y5.220623",
"G01 X1.707854 Y5.483112",
"G01 X1.261317 Y5.784175",
"G01 X0.91632 Y6.123813",
"G01 X0.670418 Y6.496378",
"G01 X0.523612 Y6.89905",
"G01 X0.474676 Y7.329947",
"G01 X0.534622 Y7.807884",
"G01 X0.715684 Y8.265124",
"G01 X1.015415 Y8.700725",
"G01 X1.431368 Y9.092107",
"G01 X1.957426 Y9.414809",
"G01 X2.594813 Y9.669771",
"G01 X3.31172 Y9.853232 ",
"G01 X4.081233 Y9.963308",
"G01 X4.902128 Y10",
"G01 X5.798875 Y9.961427",
"G01 X6.620994 Y9.845705",
"G01 X7.369709 Y9.653778",
"G01 X8.02667 Y9.385644",
"G01 X8.57475 Y9.046006",
"G01 X9.012723 Y8.632985",
"G01 X9.336923 Y8.164456",
"G01 X9.542452 Y7.655471",
"G01 X9.630536 Y7.107913",
"G01 X8.031564 Y7.01289",
"G01 X7.877416 Y7.57362",
"G01 X7.587472 Y8.035563",
"G01 X7.160509 Y8.396839",
"G01 X6.586738 Y8.656506",
"G01 X5.855151 Y8.812682",
"G01 X4.966969 Y8.864428",
"G01 X4.050648 Y8.817387",
"G01 X3.316614 Y8.675323",
"G01 X2.764864 Y8.439177",
"G01 X2.380719 Y8.135291",
"G01 X2.150722 Y7.793772",
"G01 X2.073648 Y7.412739",
"G01 X2.128701 Y7.085333",
"G01 X2.291412 Y6.799323",
"G01 X2.563005 Y6.554709",
"G01 X3.056031 Y6.327971",
"G01 X3.892831 Y6.09841",
"G01 X5.074627 Y5.864146",
"G01 X6.285784 Y5.639289",
"G01 X7.214339 Y5.433249",
"G01 X7.857842 Y5.247907",
"G01 X8.525814 Y4.96566",
"G01 X9.06533 Y4.637313",
"G01 X9.477612 Y4.263806",
"G01 X9.767556 Y3.846082",
"G01 X9.941278 Y3.387901",
"G01 X10 Y2.890206",
"G01 X9.936384 Y2.386866",
"G01 X9.744311 Y1.902343",
"G01 X9.426229 Y1.438517",
"G01 X8.987032 Y1.017029",
"G01 X8.43773 Y0.664221",
"G01 X7.775875 Y0.378211",
"G01 X7.029606 Y0.167466",
"G01 X6.222167 Y0.042337",
"G01 X5.356007 Y0",
"G01 X4.280646 Y0.042337",
"G01 X3.327624 Y0.169348",
"G01 X2.498165 Y0.381974",
"G01 X1.784928 Y0.679274",
"G01 X1.181796 Y1.063129",
"G01 X0.689993 Y1.5326",
"G01 X0.321752 Y2.066987",
"G01 X0.091754 Y2.645592",
"G01 X0 Y3.269357",
"G01 X0 Y3.269357"
};


int c=1;
int gcode_indx = 0 , prev_gcode_indx = 0;
int end_gcode_sub_indx = 0 , gcode_sub_indx = 0 , prev_gcode_sub_indx = 0;

volatile double x=0,y=0,e=0,f=500;

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

volatile void delay_1us_nop() {
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop");

}

/*
ISR(TIMER1_COMPA_vect){
  step_x();
}
*/


ISR(TIMER1_COMPA_vect){
  if(travelled>distance){
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); 
    if(gcode_indx<total_gcodes-1 && end_gcode_sub_indx == gcode_sub_indx) gcode_indx++;
    //Serial.println(travelled,DEC);
    //Serial.println(p_x);
    //Serial.println(p_y,DEC);
    //Serial.println(count2);
    
    
    travelled=0.0;
    ins_x=0.0;
    ins_y=0.0;
    gcode_step_x = 0.0;
    gcode_step_y = 0.0;
    PORTD |= (1 << 7);
    PORTF |= (1 << 2);
    if(g_code == 2 || g_code == 3) next_quadrant();
  }else{
    //Serial.println(get_direction_x());
    travelled = travelled + mm_per_step;
    PORTD &= ~(1 << 7);
    PORTF &= ~(1 << 2);
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
  
    PORTF |= (1 << 0);
    //for(int i=0;i<400;i++){
      delay_1us_nop();
   // }
    PORTF &= ~(1 << 0);
   
}

void step_y(){
    PORTF |= (1 << 6);
    //for(int i=0;i<400;i++){
      delay_1us_nop();
   // }
    PORTF &= ~(1 << 6);
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
  PORTF |= (1 << 1);
}

void set_anti_clockwise_for_x(){
  PORTF &= ~(1 << 1);
}

void set_clockwise_for_y(){
  PORTF |= (1 << 7); //Y axis dir
}

void set_anti_clockwise_for_y(){
  PORTF &= ~(1 << 7); //Y axis dir
}


void set_direction(){
  if((x - p_x) < 0){ 
    set_anti_clockwise_for_x();
  }else if((x - p_x) > 0){
    set_clockwise_for_x();
  }
  if((y-p_y)<0){ 
    set_anti_clockwise_for_y();
  }else if((y - p_y) > 0){
    set_clockwise_for_y();
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


  // Enable the stepper driver (LOW to enable for DRV8825)
  
     
  DDRD |= (1 << 7);   // PIN 38 as output ENABLE Pin of X
  PORTD &= ~(1 << 7); // ENABLE Pin set to LOW

  DDRF |= (1 << 0);  // PIN 54 as output STEP Pin of X
  DDRF |= (1 << 1);  // PIN 55 as output DIR pin of X



  DDRF |= (1 << 2);   // PIN 56 as output ENABLE Pin of Y
  PORTF &= ~(1 << 2); // ENABLE Pin set to LOW

  DDRF |= (1 << 6);  // PIN 60 as output STEP Pin of Y
  DDRF |= (1 << 7);  // PIN 61 as output DIR Pin of Y

  

  TCCR1A = 0;
  TCCR1B = (1<<WGM12) | (1<<CS10);
  TIMSK1=  (1<<OCIE1A);
  parse_gcodes(0);
  sei(); 
}

void loop() {
  
  if(gcode_indx>prev_gcode_indx){
    //Serial.println(gcode_indx);
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


