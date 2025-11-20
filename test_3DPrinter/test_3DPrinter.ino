#include <SD.h>
#define CHIP_SELECT_PIN 53
#define RING_BUFFER_SIZE 64
#define COMMAND_SIZE 128

File gcode_file;
char ring_buffer[RING_BUFFER_SIZE][COMMAND_SIZE];
char command [COMMAND_SIZE];


void setup() {
  Serial.begin(115200);
  while (!Serial);
  if (!SD.begin(CHIP_SELECT_PIN)) {
    Serial.println("SD Card initialization failed.");
    while(true);
  }
  Serial.println("SD Card initialized..");
  gcode_file = SD.open("m3.gcd");
}

char* readLineFromSDCard(){
   unsigned int command_indx = 0;
  while(true){
    if(gcode_file.available()){
      char c = gcode_file.read();
      if(c == '\n' || c == '\r'){
        c = '\0';
      }
      command[command_indx++] = c;
     
      if(c == '\0') break;
    }else{
      command[0] = '\0';
      break;
    }
  }
  for(int i=0;i<command_indx;i++){
    if(command[i] == ';' || command[i] == '#'){
      command[i]='\0';
    }
  }
  return command;
}

void loop() {
  char* x = readLineFromSDCard();
  if(strlen(x)>0){
    Serial.println(x);
  }


}

