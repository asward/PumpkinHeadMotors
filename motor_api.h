/*
 * motor_api.h — Public motor API.
 *
 * Use these from your own code (e.g. buttons, sensors) as well as from the serial layer.
 * motor = 0..3 (M1..M4). speed = steps/s, 0 = motor default. Returns NULL or an error name.
 * All calls are non-blocking: they queue work that runs in the background.
 */
#pragma once
#include <Arduino.h>

const __FlashStringHelper *motorForward (uint8_t motor, uint32_t steps, uint16_t speed = 0, uint16_t tag = 0);
const __FlashStringHelper *motorBackward(uint8_t motor, uint32_t steps, uint16_t speed = 0, uint16_t tag = 0);
const __FlashStringHelper *motorGoto(uint8_t motor, int32_t position, uint16_t speed = 0, uint16_t tag = 0);
const __FlashStringHelper *motorJog(uint8_t motor, int8_t dir, uint16_t speed = 0, uint16_t tag = 0);  // runs until stopped or limit

void    motorStop(uint8_t motor);                      // decelerate, flush queue
void    motorHalt(uint8_t motor);                      // instant stop, flush queue
void    motorSetSpeed(uint8_t motor, uint16_t sps);    // default speed, clamped to 1..MAX_SPEED
void    motorSetAccel(uint8_t motor, uint32_t spss);   // accel, clamped to 1..MAX_ACCEL
bool    motorBusy(uint8_t motor);                      // running, dwelling, or has queued work
int32_t motorPosition(uint8_t motor);
