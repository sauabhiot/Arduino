#ifndef CONFIG_H
#define CONFIG_H
#include <stdint.h>


#define ACC_PROFILE 1000
#define Z_HEIGHT_PROBE A5
constexpr float DEFAULT_FEED_RATE = 300/60;
constexpr uint8_t RING_BUFFER_SIZE = 16;
constexpr uint8_t COMMAND_SIZE = 128;
constexpr long BASE_FREQUENCY = 16000000;
constexpr uint8_t CLOCK_PRESCALAR = 1;
constexpr long TIMER_FREQUENCY = BASE_FREQUENCY/CLOCK_PRESCALAR;

constexpr bool PCB_PRINTING = true;
constexpr uint8_t STEPS_PER_REVOLUTION = 200; 
constexpr uint8_t MICROSTEPPING = 16;
constexpr uint8_t SCREW_PITCH = 2;
constexpr uint8_t NUMBER_OF_STARTS = 4;
constexpr uint16_t STEPS_PER_MM = (STEPS_PER_REVOLUTION * MICROSTEPPING)/(SCREW_PITCH * NUMBER_OF_STARTS); // (200 *16)/(2*4) = 400
constexpr float MM_PER_STEP = 1.00/(float)STEPS_PER_MM; // 0.0025
constexpr float NEGATIVE_THOUSAND = -1000.00f;

constexpr float ARC_SEGMENT_LENGTH = (MM_PER_STEP/5);
constexpr float MM_PER_STEP_EXTRUDER = MM_PER_STEP * (21.0f/5.0f);

#endif