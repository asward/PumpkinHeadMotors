/*
 * pins.h — Pin map for the CNC Shield V3 on an Arduino Uno.
 *
 * The arrays are defined once in pins.cpp so they are not duplicated in RAM
 * by every translation unit that includes this header.
 *
 * NOTE: the step-generation ISR (stepgen.cpp) uses direct port access and
 * hard-codes these same pins. If you change STEP or LIMIT pins here, update
 * limRaw()/stepPin()/stepTick() in stepgen.cpp to match:
 *   STEP: D2=PD2 D3=PD3 D4=PD4 D12=PB4     LIMIT: D9=PB1 D10=PB2 D11=PB3 A1=PC1
 */
#pragma once
#include <Arduino.h>
#include "config.h"

//                                          M1(X) M2(Y) M3(Z) M4(A)
extern const uint8_t PIN_STEP[NUM_MOTORS];   //   2     3     4    12   A: shield jumpers A.STP -> D12
extern const uint8_t PIN_DIR[NUM_MOTORS];    //   5     6     7    13   A: shield jumpers A.DIR -> D13
extern const uint8_t PIN_LIMIT[NUM_MOTORS];  //   9    10    11    A1   X+/X-, Y+/Y-, Z+/Z- share one pin each;
                                             //                         A limit uses the "Hold" header (A1)

const uint8_t PIN_EN    = 8;    // shared driver enable, active LOW
const uint8_t PIN_ESTOP = A0;   // "Abort" header
// Spare: A2 ("Resume"), A3 ("CoolEn"), A4/SDA, A5/SCL ("Probe" on some silkscreens).
// D12/D13 ("SpnEn"/"SpnDir") are consumed by the A axis.
