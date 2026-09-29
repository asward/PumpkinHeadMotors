/*
 * pins.cpp — Storage for the per-motor pin arrays declared in pins.h.
 */
#include "pins.h"

//                                        M1(X) M2(Y) M3(Z) M4(A)
const uint8_t PIN_STEP[NUM_MOTORS]  = {   2,    3,    4,   12 };
const uint8_t PIN_DIR[NUM_MOTORS]   = {   5,    6,    7,   13 };
const uint8_t PIN_LIMIT[NUM_MOTORS] = {   9,   10,   11,   A1 };
