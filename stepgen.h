/*
 * stepgen.h — Real-time step generation (Timer1 ISR) and limit-switch scanning.
 *
 * The ISR runs at TICK_HZ and owns the step pins. Each motor is driven by a
 * DDS accumulator: the foreground only sets a rate (steps/s), direction and
 * optional step count; the ISR emits the pulses, tracks absolute position,
 * debounces the limit inputs and hard-stops a motor whose switch closes.
 *
 * Multi-byte isr* variables must be read/written with interrupts disabled —
 * use the atomic helpers below rather than touching them directly.
 */
#pragma once
#include <Arduino.h>
#include "config.h"

// ---------------------- shared ISR state ----------------------
extern volatile uint16_t isrRate[NUM_MOTORS];     // steps/s, 0 = stopped
extern volatile int8_t   isrDir[NUM_MOTORS];      // +1 / -1
extern volatile int32_t  isrPos[NUM_MOTORS];      // absolute position, steps
extern volatile uint32_t isrRemain[NUM_MOTORS];   // steps left in a bounded move
extern volatile bool     isrBounded[NUM_MOTORS];  // false = run until told to stop (JOG)
extern volatile bool     isrGuard[NUM_MOTORS];    // limit halts motion when true
extern volatile bool     limActive[NUM_MOTORS];   // filtered switch state
extern volatile bool     limTrip[NUM_MOTORS];     // latched: ISR halted motor on limit

// Configure step/dir/limit pins and start the Timer1 step ISR.
void stepgenBegin();

// ------------------------- atomic helpers -------------------------
uint16_t getRate(uint8_t i);
int32_t  getPos(uint8_t i);
uint32_t getRemain(uint8_t i);

// Stop pulse generation for a motor and forget any bounded move.
void clearMotion(uint8_t i);

// Only update rate if the ISR has not already finished/tripped the move.
void setRateIfActive(uint8_t i, uint16_t r);

// Set DIR, wait setup time, then hand the move to the ISR.
// steps = 0 means unbounded (run until stopped). guard = limit switch interlock.
void startMotion(uint8_t i, int8_t dir, uint32_t steps, uint16_t rate, bool guard);
