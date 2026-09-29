/*
 * serial_api.h — ASCII serial protocol (see "stepper4 Serial Interface Spec").
 *
 * Lines are newline-terminated, case-insensitive, optionally prefixed with a
 * "#<tag>" correlation id. Every line gets exactly one "OK"/"ERR" reply; the
 * '!' character is handled immediately as a real-time HALT of all motors.
 * Also owns the host watchdog (WD command): if enabled and no bytes arrive
 * within the timeout, all motors are halted.
 */
#pragma once
#include <Arduino.h>

void serialApiBegin();        // open the port and arm the host-watchdog timer
void pollSerial();            // read and execute any complete lines
void checkHostWatchdog();     // halt all motors if the host has gone quiet
