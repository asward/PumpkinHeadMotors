/*
 * stepgen.cpp — Timer1 step-generation ISR, limit debounce, and atomic accessors.
 */
#include "stepgen.h"
#include "pins.h"

// ============================ ISR STATE ============================
volatile uint16_t isrRate[NUM_MOTORS];
volatile int8_t   isrDir[NUM_MOTORS] = { 1, 1, 1, 1 };
volatile int32_t  isrPos[NUM_MOTORS];
volatile uint32_t isrRemain[NUM_MOTORS];
volatile bool     isrBounded[NUM_MOTORS];
volatile bool     isrGuard[NUM_MOTORS] = { true, true, true, true };
volatile bool     limActive[NUM_MOTORS];
volatile bool     limTrip[NUM_MOTORS];
static uint16_t   isrAcc[NUM_MOTORS];   // DDS phase accumulators
static uint8_t    limCnt[NUM_MOTORS];   // debounce integrators
static uint8_t    limDiv;               // round-robin limit scan index

// ======================= DIRECT PORT ACCESS =======================
// Templated on motor index so each call compiles to a single port instruction.
// Must match the pin map in pins.h/pins.cpp.
template <uint8_t i> static inline bool limRaw() {
  bool v = (i == 0) ? (PINB & _BV(1)) : (i == 1) ? (PINB & _BV(2)) : (i == 2) ? (PINB & _BV(3)) : (PINC & _BV(1));
  return LIMIT_ACTIVE_LOW ? !v : v;
}
template <uint8_t i> static inline void stepPin() {
  if (i == 0) PORTD |= _BV(2); else if (i == 1) PORTD |= _BV(3); else if (i == 2) PORTD |= _BV(4); else PORTB |= _BV(4);
}

// Integrating debounce: state changes only after LIMIT_FILTER consecutive-ish agreeing scans
template <uint8_t i> static inline void limScan() {
  if (limRaw<i>()) { if (limCnt[i] < LIMIT_FILTER && ++limCnt[i] == LIMIT_FILTER) limActive[i] = true; }
  else             { if (limCnt[i] > 0 && --limCnt[i] == 0) limActive[i] = false; }
}

template <uint8_t i> static inline void tickMotor() {
  uint16_t r = isrRate[i];
  if (!r) return;
  if (limActive[i] && isrGuard[i]) {     // interlock: hard stop, no further steps
    isrRate[i] = 0;
    limTrip[i] = true;
    return;
  }
  uint16_t a = isrAcc[i] + r;            // DDS: one step each time acc passes TICK_HZ
  if (a >= TICK_HZ) {
    a -= TICK_HZ;
    stepPin<i>();
    if (isrDir[i] > 0) isrPos[i]++; else isrPos[i]--;
    if (isrBounded[i] && --isrRemain[i] == 0) isrRate[i] = 0;
  }
  isrAcc[i] = a;
}

static inline void stepTick() {
  PORTD &= ~(_BV(2) | _BV(3) | _BV(4));  // end previous step pulses (width = 1 tick = 50 us)
  PORTB &= ~_BV(4);
  switch (limDiv++ & 3) {                // scan one limit input per tick: each at 5 kHz
    case 0: limScan<0>(); break; case 1: limScan<1>(); break;
    case 2: limScan<2>(); break; default: limScan<3>(); break;
  }
  tickMotor<0>(); tickMotor<1>(); tickMotor<2>(); tickMotor<3>();
}
ISR(TIMER1_COMPA_vect) { stepTick(); }

// ============================== SETUP ==============================
void stepgenBegin() {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    pinMode(PIN_STEP[i], OUTPUT);
    pinMode(PIN_DIR[i], OUTPUT);
    pinMode(PIN_LIMIT[i], INPUT_PULLUP);
  }

  // Timer1 CTC, prescaler 8 -> 2 MHz, OCR1A for TICK_HZ
  cli();
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS11);
  OCR1A  = (uint16_t)(F_CPU / 8 / TICK_HZ - 1);
  TCNT1  = 0;
  TIMSK1 = _BV(OCIE1A);
  sei();
}

// ------------------------- atomic helpers -------------------------
uint16_t getRate(uint8_t i)   { uint8_t s = SREG; cli(); uint16_t v = isrRate[i];   SREG = s; return v; }
int32_t  getPos(uint8_t i)    { uint8_t s = SREG; cli(); int32_t  v = isrPos[i];    SREG = s; return v; }
uint32_t getRemain(uint8_t i) { uint8_t s = SREG; cli(); uint32_t v = isrRemain[i]; SREG = s; return v; }

void clearMotion(uint8_t i) {
  uint8_t s = SREG; cli();
  isrRate[i] = 0; isrBounded[i] = false; isrRemain[i] = 0;
  SREG = s;
}

void setRateIfActive(uint8_t i, uint16_t r) {
  uint8_t s = SREG; cli();
  if ((!isrBounded[i] || isrRemain[i]) && !limTrip[i]) isrRate[i] = r;
  SREG = s;
}

void startMotion(uint8_t i, int8_t dir, uint32_t steps, uint16_t rate, bool guard) {
  digitalWrite(PIN_DIR[i], ((dir > 0) != DIR_INVERT[i]) ? HIGH : LOW);
  delayMicroseconds(5);                       // DIR setup time before first STEP
  uint8_t s = SREG; cli();
  isrDir[i] = dir; isrRemain[i] = steps; isrBounded[i] = (steps != 0);
  isrGuard[i] = guard; isrAcc[i] = 0; isrRate[i] = rate;
  SREG = s;
}
