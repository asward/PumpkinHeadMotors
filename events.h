/*
 * events.h — Asynchronous "EVT <type> <motor> <tag> [extra]" lines sent to the host.
 *
 * Motor index i is 0-based internally and printed 1-based (M1..M4).
 */
#pragma once
#include <Arduino.h>

// EVT <type> <motor> <tag>
void evt(const __FlashStringHelper *type, uint8_t i, uint16_t tag);
// EVT <type> <motor> <tag> <code>
void evtCode(const __FlashStringHelper *type, uint8_t i, uint16_t tag, const __FlashStringHelper *code);
// EVT <type> <motor> <tag> <position>
void evtPos(const __FlashStringHelper *type, uint8_t i, uint16_t tag);
