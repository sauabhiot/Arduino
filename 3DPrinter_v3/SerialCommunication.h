#ifndef SERIALCOMMUNICATION_H
#define SERIALCOMMUNICATION_H
#include <Arduino.h>
#include "Config.h"
#include "RingBuffer.h"

class SerialCommunication{
  public:
    SerialCommunication(RingBuffer* rb);
    void handle();

    private:
      RingBuffer* ringBuffer;
      uint8_t buffer_index = 0;
      bool command_complete = false;
};


#endif