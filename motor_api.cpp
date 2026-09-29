/*
 * motor_api.cpp — Thin public wrappers over the motor state machine.
 */
#include "motor_api.h"
#include "motor.h"
#include "stepgen.h"

const __FlashStringHelper *motorForward(uint8_t motor, uint32_t steps, uint16_t speed, uint16_t tag) {
  return enqueue(motor, C_MOVE,  (int32_t)steps, speed, tag);
}
const __FlashStringHelper *motorBackward(uint8_t motor, uint32_t steps, uint16_t speed, uint16_t tag) {
  return enqueue(motor, C_MOVE, -(int32_t)steps, speed, tag);
}
const __FlashStringHelper *motorGoto(uint8_t motor, int32_t position, uint16_t speed, uint16_t tag) {
  return enqueue(motor, C_GOTO, position, speed, tag);
}
const __FlashStringHelper *motorJog(uint8_t motor, int8_t dir, uint16_t speed, uint16_t tag) {
  return enqueue(motor, C_JOG, dir >= 0 ? 1 : -1, speed, tag);
}
void motorStop(uint8_t motor)      { stopMotor(motor, true); }
void motorHalt(uint8_t motor)      { haltMotor(motor); }
void motorSetSpeed(uint8_t motor, uint16_t sps)  { M[motor].defSpeed = constrain(sps, 1, MAX_SPEED); }
void motorSetAccel(uint8_t motor, uint32_t spss) { M[motor].accel = constrain(spss, 1UL, MAX_ACCEL); }
bool motorBusy(uint8_t motor)      { return M[motor].state != ST_IDLE || M[motor].qn; }
int32_t motorPosition(uint8_t motor) { return getPos(motor); }
