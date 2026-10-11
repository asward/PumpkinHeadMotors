/*
 * motor.h — Per-motor state machine, command queues, motion profiles and E-stop.
 *
 * This is the foreground (non-ISR) half of the controller. Each motor has its
 * own FIFO of commands; serviceMotor() is called every loop() pass to advance
 * the state machine, run the trapezoidal speed profile, and handle limit trips
 * (automatic backoff until the switch re-opens).
 */
#pragma once
#include <Arduino.h>
#include "config.h"

enum MState : uint8_t { ST_IDLE, ST_RUN, ST_DWELL, ST_BACKOFF, ST_FAULT };
enum CmdType : uint8_t { C_MOVE, C_GOTO, C_JOG, C_WAIT };

struct Cmd {
  uint8_t  type;    // CmdType
  int32_t  arg;     // steps, position, direction, or ms
  uint16_t speed;   // 0 = motor default
  uint16_t tag;     // host correlation id, 0 = none
};

struct Motor {
  MState   state;
  Cmd      q[QLEN];    // ring buffer of pending commands
  uint8_t  qh, qn;     // head index, count
  Cmd      cur;        // command currently executing
  float    speed;      // current commanded rate
  uint16_t target;     // cruise rate for current command
  uint16_t defSpeed;
  uint32_t accel;
  bool     stopping;   // decelerating to a cancel
  uint32_t dwellEnd;
  int8_t   tripDir;    // direction of last limit trip, 0 = unknown
  uint8_t  boPhase;    // backoff: 0 = until switch opens, 1 = clearance steps
  int32_t  boStart;
  bool     lastLim;    // last reported limit state (for LIMSW events)
  uint32_t idleMs;     // idle time before this motor allows driver power-down, 0 = hold
  uint32_t lastBusy;   // millis() when this motor was last not idle
};

extern Motor M[NUM_MOTORS];
extern bool  estopLatched;

// Initialise motor state, driver-enable and E-stop pins. Call once from setup().
void motorsBegin();

// Advance motor i's state machine. tick = true roughly every 1 ms, with dt in seconds.
void serviceMotor(uint8_t i, bool tick, float dt);

// Queue a command. Returns NULL on success or an error name (ESTOP/FAULT/QFULL).
const __FlashStringHelper *enqueue(uint8_t i, uint8_t type, int32_t arg, uint16_t spd, uint16_t tag);

void flushQueue(uint8_t i);                 // discard queue, emitting DROP per command
void haltMotor(uint8_t i);                  // immediate stop, flush queue
void haltAll();
void stopMotor(uint8_t i, bool flush);      // decelerated stop
void startBackoff(uint8_t i);               // drive away from the switch in -tripDir
void driversOn(bool on);                    // shared EN pin
void serviceIdlePower();                    // power drivers down once all motors' idle times pass

void latchEstop();                          // halt everything, disable drivers, latch
bool estopInputActive();                    // hardware E-stop input state
