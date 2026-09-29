/*
 * events.cpp — Formatting of asynchronous EVT lines.
 */
#include "events.h"
#include "stepgen.h"

void evt(const __FlashStringHelper *type, uint8_t i, uint16_t tag) {
  Serial.print(F("EVT ")); Serial.print(type); Serial.print(' ');
  Serial.print(i + 1); Serial.print(' '); Serial.println(tag);
}

void evtCode(const __FlashStringHelper *type, uint8_t i, uint16_t tag, const __FlashStringHelper *code) {
  Serial.print(F("EVT ")); Serial.print(type); Serial.print(' ');
  Serial.print(i + 1); Serial.print(' '); Serial.print(tag); Serial.print(' '); Serial.println(code);
}

void evtPos(const __FlashStringHelper *type, uint8_t i, uint16_t tag) {
  Serial.print(F("EVT ")); Serial.print(type); Serial.print(' ');
  Serial.print(i + 1); Serial.print(' '); Serial.print(tag); Serial.print(' '); Serial.println(getPos(i));
}
