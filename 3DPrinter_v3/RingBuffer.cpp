#include "RingBuffer.h"
#include <Arduino.h>

RingBuffer::RingBuffer(){

}


bool RingBuffer::is_full(){
  if(head == RING_BUFFER_SIZE-1 && tail>0){
    return false;
  }else if(head < (RING_BUFFER_SIZE-1)){
    if(one_round_complete && (tail - head)>=0){
      return false;
    }else if(!one_round_complete){
      return false;
    }
  }
  return true;
}

int RingBuffer::get_next_buffer_index_to_write(){
  if(head == RING_BUFFER_SIZE-1 && tail>0){
    head = 0;
    one_round_complete = 1;
    write_count++;
    return head;
  }else if(head < (RING_BUFFER_SIZE-1)){
    if(one_round_complete && (tail - head)>=0){
      head++;
      write_count++;
      return head;
    }else if(!one_round_complete){
      head++;
      write_count++;
      return head;
    }
  }
  return -1;
}

int RingBuffer::get_next_buffer_index_to_read(){
  if(read_count<write_count){
    if(tail == RING_BUFFER_SIZE-1){
      tail = 0;
    }else{
      tail++;
    }
    read_count++;
    return tail;
  }else{
    return -1;
  }
}

bool RingBuffer::insertToRingBuffer(char command[]){
  if(strlen(command) > 0){
    int write_indx = get_next_buffer_index_to_write();
    if(write_indx >= 0 && write_indx < RING_BUFFER_SIZE){ 
      strlcpy(ring_buffer[write_indx], command,COMMAND_SIZE);
      return true;
    }
  }
  return false;  
}

char* RingBuffer::fetch(int index){
  return ring_buffer[index];
}
