/*
 * config.h — Compile-time tunables for stepper4.
 *
 * Everything a user is likely to adjust for their machine lives here:
 * speed/accel limits, limit-switch polarity and filtering, E-stop input,
 * queue depth and per-motor direction inversion.
 */
#pragma once
#include <Arduino.h>

#define FW_VERSION "1.0.0"
#define BAUD 115200UL

// =========================== CONFIGURATION ===========================
#define NUM_MOTORS       4
#define TICK_HZ          20000UL   // step-generation ISR rate (Timer1)
#define MAX_SPEED        10000U    // steps/s hard ceiling (= TICK_HZ / 2)
#define MIN_SPEED        50.0f     // start/stop speed, steps/s
#define DEFAULT_SPEED    1000U     // steps/s
#define DEFAULT_ACCEL    4000UL    // steps/s^2
#define MAX_ACCEL        200000UL  // steps/s^2
#define BACKOFF_SPEED    400U      // steps/s while clearing a limit switch
#define BACKOFF_CLEAR    200UL     // extra steps after the switch re-opens
#define BACKOFF_MAX      4000L     // FAULT if switch still closed after this many steps
#define LIMIT_FILTER     5         // limit scans (5 kHz each) to change state: 5 = 1 ms
#define LIMIT_ACTIVE_LOW 1         // 1: NO switch to GND (pressed = LOW)
                                   // 0: NC switch to GND (pressed/broken wire = HIGH) - fail-safe
#define ESTOP_ENABLED    0         // 1: use "Abort" header (A0) as hardware E-stop input
#define ESTOP_ACTIVE_LOW 0         // 0: NC button to GND (pressed/broken wire = HIGH)
#define QLEN             8         // command queue depth per motor

// Per-motor direction inversion (M1..M4). Only referenced by stepgen.cpp.
const bool DIR_INVERT[NUM_MOTORS] = { false, false, false, false };

static_assert(MAX_SPEED <= TICK_HZ / 2, "MAX_SPEED must be <= TICK_HZ/2");
static_assert(BACKOFF_CLEAR > 0, "BACKOFF_CLEAR must be > 0");
