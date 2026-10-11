/*
 * stepper4.ino — Independent 4-motor stepper controller with ASCII serial API
 *
 * Hardware : Arduino Uno (ATmega328P, 16 MHz) + CNC Shield V3
 *            (RATTMMOTOR GRBL kit), A4988 or DRV8825 drivers.
 *            This sketch REPLACES GRBL; do not load GRBL alongside it.
 * Protocol : 115200 8N1, newline-terminated ASCII. See "stepper4 Serial Interface Spec".
 *
 * Each motor (M1..M4 = X, Y, Z, A) has its own command queue and runs
 * independently of the others. Limit switches halt the tripped motor
 * immediately, then back it off until the switch re-opens.
 *
 * Source layout:
 *   config.h           compile-time tunables (speeds, limits, polarity, queue depth)
 *   pins.h/.cpp        CNC Shield V3 pin map
 *   stepgen.h/.cpp     Timer1 step ISR, limit debounce, atomic ISR-state helpers
 *   motor.h/.cpp       per-motor state machine, queues, accel profile, backoff, E-stop
 *   events.h/.cpp      asynchronous "EVT ..." messages to the host
 *   motor_api.h/.cpp   public motorForward()/motorGoto()/... API for local code
 *   serial_api.h/.cpp  ASCII command parser/dispatcher and host watchdog
 */
#include <Arduino.h>
#include <avr/wdt.h>
#include "config.h"
#include "stepgen.h"
#include "motor.h"
#include "motor_api.h"
#include "serial_api.h"

static uint32_t lastProfileUs = 0;   // last motion-profile update, micros()

// ============================ SETUP / LOOP ============================
void setup() {
  MCUSR = 0; wdt_disable();
  motorsBegin();                               // state, EN pin (drivers off), E-stop pin
  serialApiBegin();
  stepgenBegin();                              // step/dir/limit pins + Timer1 ISR

  wdt_enable(WDTO_250MS);                      // MCU hang -> reset (drivers disabled on boot)
  lastProfileUs = micros();
  Serial.println(F("EVT BOOT 0 0 " FW_VERSION));
}

void loop() {
  wdt_reset();
  pollSerial();

  if (!estopLatched && estopInputActive()) latchEstop();
  checkHostWatchdog();

  // Run the acceleration profile at ~1 kHz; clamp dt so a stall can't cause a speed jump
  uint32_t now = micros();
  float dt = (now - lastProfileUs) * 1e-6f;
  bool tick = dt >= 0.001f;
  if (tick) { lastProfileUs = now; if (dt > 0.02f) dt = 0.02f; }

  for (uint8_t i = 0; i < NUM_MOTORS; i++) serviceMotor(i, tick, dt);
  serviceIdlePower();
}
