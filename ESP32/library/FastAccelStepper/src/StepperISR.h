#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_AVR)
#include <Arduino.h>
#endif
#include <stdint.h>

#include "FastAccelStepper.h"

#if defined(ARDUINO_ARCH_AVR)
#define stepPinStepperA 9  /* OC1A */
#define stepPinStepperB 10 /* OC1B */
#endif

// Here are the global variables to interface with the interrupts

// CURRENT QUEUE IMPLEMENTATION WASTES ONE UNUSED ENTRY => BUG/TODO

#if defined(TEST)
#define NUM_QUEUES 2
#define fas_queue_A fas_queue[0]
#define fas_queue_B fas_queue[1]
#define QUEUE_LEN 16
#elif defined(ARDUINO_ARCH_AVR)
#define NUM_QUEUES 2
#define fas_queue_A fas_queue[0]
#define fas_queue_B fas_queue[1]
#define QUEUE_LEN 16
#elif defined(ARDUINO_ARCH_ESP32)
#define NUM_QUEUES 6
#define QUEUE_LEN 32
#else
#define NUM_QUEUES 6
#define QUEUE_LEN 32
#endif

// These variables control the stepper timing behaviour
#define QUEUE_LEN_MASK (QUEUE_LEN - 1)

#ifndef TEST
#define inject_fill_interrupt(x)
#endif

#define TICKS_FOR_STOPPED_MOTOR 0xffffffff

#if defined(ARDUINO_ARCH_ESP32)
#include <driver/mcpwm.h>
#include <driver/pcnt.h>
#include <soc/mcpwm_reg.h>
#include <soc/mcpwm_struct.h>
#include <soc/pcnt_reg.h>
#include <soc/pcnt_struct.h>
struct mapping_s {
  mcpwm_unit_t mcpwm_unit;
  uint8_t timer;
  mcpwm_io_signals_t pwm_output_pin;
  pcnt_unit_t pcnt_unit;
  int input_sig_index;
  uint32_t timer_tez_int_clr;
  uint32_t timer_tez_int_ena;
};
#endif

struct queue_entry {
  uint8_t steps;  // coding is bit7..1 is nr of steps and bit 0 is direction
  uint8_t n_periods;
  uint16_t period;
};
class StepperQueue {
 public:
  struct queue_entry entry[QUEUE_LEN];
  uint8_t read_idx;  // ISR stops if readptr == next_writeptr
  uint8_t next_write_idx;
  uint32_t on_delay_ticks;
  uint8_t dirPin;
  bool dirHighCountsUp;
  bool isRunning;
#if defined(ARDUINO_ARCH_ESP32)
  const struct mapping_s* mapping;
  // These two variables are for the mcpwm interrupt
  uint8_t current_period;
  uint8_t current_n_periods;
#endif
#if defined(ARDUINO_ARCH_AVR)
  // This is used in the timer compare unit as extension of the 16 timer
  uint8_t skip;
  uint16_t period;
#endif
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
  uint8_t checksum;
#endif

  bool dir_at_queue_end;
  int32_t pos_at_queue_end;     // in steps
  uint32_t ticks_at_queue_end;  // in timer ticks, 0 on stopped stepper

  void init(uint8_t queue_num, uint8_t step_pin);
  inline bool isQueueFull() {
    noInterrupts();
    uint8_t rp = read_idx;
    uint8_t wp = next_write_idx;
    interrupts();
    rp += QUEUE_LEN;
    return (wp == rp);
  }
  inline bool isQueueEmpty() {
    noInterrupts();
    bool res = (next_write_idx == read_idx);
    interrupts();
    inject_fill_interrupt(0);
    return res;
  }
  int addQueueEntry(uint32_t ticks, uint8_t steps, bool dir) {
    if (steps >= 128) {
      return AQE_STEPS_ERROR;
    }
    if (steps == 0) {
      return AQE_STEPS_ERROR;
    }
    if (ticks > ABSOLUTE_MAX_TICKS) {
      return AQE_TOO_HIGH;
    }

    if (!isRunning) {
      if (on_delay_ticks > 0) {
        int res = _addQueueEntry(on_delay_ticks, 1, dir);
        if ((res != AQE_OK) || (steps == 1)) {
          return res;
        }
        steps -= 1;
      }
    }
    return _addQueueEntry(ticks, steps, dir);
  }

  int _addQueueEntry(uint32_t ticks, uint8_t steps, bool dir) {
    if (isQueueFull()) {
      return AQE_FULL;
    }
    uint16_t period;
    uint8_t n_periods;
    if (ticks > 65535) {
      n_periods = ticks >> 16;
      n_periods += 1;
      period = ticks / n_periods;
    } else {
      period = ticks;
      n_periods = 1;
    }

    uint8_t wp = next_write_idx;
    struct queue_entry* e = &entry[wp & QUEUE_LEN_MASK];
    pos_at_queue_end += (dir == dirHighCountsUp) ? steps : -steps;
    ticks_at_queue_end = ticks;
    steps <<= 1;
    e->period = period;
    e->n_periods = n_periods;
    // check for dir pin value change
    e->steps = (dir != dir_at_queue_end) ? steps | 0x01 : steps;
    dir_at_queue_end = dir;
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
    {
      // checksum is in the struct and will updated here
      unsigned char* x = (unsigned char*)e;
      for (uint8_t i = 0; i < sizeof(struct queue_entry); i++) {
        if (checksum & 0x80) {
          checksum <<= 1;
          checksum ^= 0xde;
        } else {
          checksum <<= 1;
        }
        checksum ^= *x++;
      }
    }
#endif
    wp++;
    noInterrupts();
    if (isRunning) {
      next_write_idx = wp;
      interrupts();
    } else {
      interrupts();
      if (!startQueue(e)) {
        next_write_idx = wp;
      }
    }
    return AQE_OK;
  }

  bool startQueue(struct queue_entry* e);
  void _initVars() {
    dirPin = PIN_UNDEFINED;
    on_delay_ticks = 0;
    read_idx = 0;
    next_write_idx = 0;
    dir_at_queue_end = true;
    dirHighCountsUp = true;
    pos_at_queue_end = 0;
    ticks_at_queue_end = TICKS_FOR_STOPPED_MOTOR;
    isRunning = false;
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
    checksum = 0;
#endif
  }
};

extern StepperQueue fas_queue[NUM_QUEUES];
