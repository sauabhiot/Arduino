#include "SerialCommunication.h"
#include "RingBuffer.h"


SerialCommunication::SerialCommunication(RingBuffer* rb){
  ringBuffer = rb;

}

void SerialCommunication::handle(){
  char command [COMMAND_SIZE];
  if(Serial.available() > 0){
    char incoming_char = Serial.read();
    if(incoming_char == '\n' || incoming_char == '\r') {
      command[buffer_index] = '\0'; 
      buffer_index = 0;
      command_complete = true;
    }else{
      if(buffer_index < COMMAND_SIZE) {
        command[buffer_index++] = incoming_char;
      }
    }
  } 
  if(command_complete){
    if(ringBuffer->insertToRingBuffer(command)) command_complete = false;
  } 

}

