#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include "Config.h"
#include <string.h>
class RingBuffer{

  private:
    bool one_round_complete = false;
    long write_count = 0;
    long read_count = 0;
    int head = -1;
    int tail = -1;
    char ring_buffer[RING_BUFFER_SIZE][COMMAND_SIZE];
    int get_next_buffer_index_to_write();

    


  public:
    RingBuffer();
    bool is_full();
    bool insertToRingBuffer(char[]);
    char* fetch(int index);
    int get_next_buffer_index_to_read();
};

#endif