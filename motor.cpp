/*
 * motor.cpp — Motor state machine, queueing, trapezoidal profile, limit backoff, E-stop.
 */
#include "motor.h"
#include "pins.h"
#include "stepgen.h"
#include "events.h"

Motor M[NUM_MOTORS];
bool  estopLatched = false;
static bool driversEnabled = false;

void motorsBegin() {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    memset(&M[i], 0, sizeof(Motor));
    M[i].state = ST_IDLE;
    M[i].defSpeed = DEFAULT_SPEED;
    M[i].accel = DEFAULT_ACCEL;
  }
  pinMode(PIN_EN, OUTPUT);
  driversOn(false);
#if ESTOP_ENABLED
  pinMode(PIN_ESTOP, INPUT_PULLUP);
#endif
}

void driversOn(bool on) {
  if (on && !driversEnabled) { digitalWrite(PIN_EN, LOW); delay(2); }   // wake-up time
  if (!on) digitalWrite(PIN_EN, HIGH);
  driversEnabled = on;
}

// ------------------------------ queue ------------------------------
// Empty a motor's queue, reporting every discarded command so the host can reconcile tags
void flushQueue(uint8_t i) {
  Motor &m = M[i];
  while (m.qn) {
    evt(F("DROP"), i, m.q[m.qh].tag);
    m.qh = (m.qh + 1) % QLEN; m.qn--;
  }
}

const __FlashStringHelper *enqueue(uint8_t i, uint8_t type, int32_t arg, uint16_t spd, uint16_t tag) {
  Motor &m = M[i];
  if (estopLatched) return F("ESTOP");
  if (m.state == ST_FAULT) return F("FAULT");
  if (m.qn >= QLEN) return F("QFULL");
  m.q[(m.qh + m.qn) % QLEN] = Cmd{ type, arg, spd, tag };
  m.qn++;
  return NULL;                                 // started by serviceMotor() after OK is sent
}

// ------------------------- motion control -------------------------
static void beginNext(uint8_t i);

static void complete(uint8_t i) {
  Motor &m = M[i];
  m.speed = 0;
  m.state = ST_IDLE;
  evt(F("DONE"), i, m.cur.tag);
  beginNext(i);
}

static void finishCancel(uint8_t i) {
  Motor &m = M[i];
  clearMotion(i);
  m.speed = 0; m.stopping = false; m.state = ST_IDLE;
  evt(F("CANCEL"), i, m.cur.tag);
}

// Pop and start the next queued command (zero-length moves complete immediately)
static void beginNext(uint8_t i) {
  Motor &m = M[i];
  while (m.qn && m.state == ST_IDLE && !estopLatched) {
    m.cur = m.q[m.qh]; m.qh = (m.qh + 1) % QLEN; m.qn--;
    uint16_t spd = m.cur.speed ? m.cur.speed : m.defSpeed;

    if (m.cur.type == C_WAIT) {
      m.state = ST_DWELL; m.dwellEnd = millis() + (uint32_t)m.cur.arg;
      return;
    }
    int32_t delta = 0; int8_t dir;
    if (m.cur.type == C_JOG) {
      dir = m.cur.arg > 0 ? 1 : -1;
    } else {
      delta = (m.cur.type == C_MOVE) ? m.cur.arg : m.cur.arg - getPos(i);
      if (delta == 0) { evt(F("DONE"), i, m.cur.tag); continue; }
      dir = delta > 0 ? 1 : -1;
    }
    // Interlock: while switch is closed only allow motion away from it
    bool guard = true;
    if (limActive[i]) {
      if (m.tripDir == 0 || dir == m.tripDir) {
        evtCode(F("FAIL"), i, m.cur.tag, F("LIMIT"));
        flushQueue(i);
        return;
      }
      guard = false;   // escaping; re-armed once switch opens
    }
    driversOn(true);
    m.target = spd; m.stopping = false;
    m.speed = (spd < MIN_SPEED) ? spd : MIN_SPEED;
    startMotion(i, dir, delta ? (uint32_t)labs(delta) : 0, (uint16_t)m.speed, guard);
    m.state = ST_RUN;
    return;
  }
}

// Trapezoidal profile, called every ~1 ms
static void runProfile(uint8_t i, float dt) {
  Motor &m = M[i];
  if (!isrGuard[i] && !limActive[i]) isrGuard[i] = true;   // escaped the switch: re-arm

  bool bounded = isrBounded[i];
  uint32_t rem = bounded ? getRemain(i) : 0;
  if (bounded && rem == 0) { complete(i); return; }

  float floorSpd = (m.target < MIN_SPEED) ? (float)m.target : MIN_SPEED;
  float tgt = m.stopping ? 0.0f : (float)m.target;
  if (bounded && !m.stopping && (m.speed * m.speed) / (2.0f * (float)m.accel) >= (float)rem)
    tgt = 0.0f;                                  // start decelerating to land on target

  float dv = (float)m.accel * dt;
  if (m.speed < tgt) m.speed = (m.speed + dv > tgt) ? tgt : m.speed + dv;
  else               m.speed = (m.speed - dv < tgt) ? tgt : m.speed - dv;

  if (m.speed < floorSpd) {
    if (m.stopping) { finishCancel(i); return; }
    m.speed = floorSpd;                          // creep out remaining steps
  }
  if (m.speed > MAX_SPEED) m.speed = MAX_SPEED;
  setRateIfActive(i, (uint16_t)m.speed);
}

// ------------------------- limit backoff -------------------------
void startBackoff(uint8_t i) {
  Motor &m = M[i];
  m.boPhase = 0; m.boStart = getPos(i); m.stopping = false;
  m.speed = BACKOFF_SPEED; m.state = ST_BACKOFF;
  driversOn(true);
  startMotion(i, -m.tripDir, 0, BACKOFF_SPEED, false);   // unbounded, interlock off
}

static void runBackoff(uint8_t i) {
  Motor &m = M[i];
  if (m.boPhase == 0) {
    if (!limActive[i]) {                        // switch opened: add clearance steps
      uint8_t s = SREG; cli();
      isrRemain[i] = BACKOFF_CLEAR; isrBounded[i] = true;
      SREG = s;
      m.boPhase = 1;
    } else if (labs(getPos(i) - m.boStart) > BACKOFF_MAX) {
      clearMotion(i); m.speed = 0; m.state = ST_FAULT;
      evtCode(F("FAULT"), i, m.cur.tag, F("LIMIT_STUCK"));
      flushQueue(i);
    }
  } else if (getRemain(i) == 0) {
    clearMotion(i); isrGuard[i] = true; m.speed = 0; m.state = ST_IDLE;
    evtPos(F("CLEAR"), i, m.cur.tag);
  }
}

// --------------------------- stopping ---------------------------
void haltMotor(uint8_t i) {
  Motor &m = M[i];
  clearMotion(i);
  isrGuard[i] = true;
  m.speed = 0; m.stopping = false;
  if (m.state == ST_RUN || m.state == ST_DWELL || m.state == ST_BACKOFF) evt(F("CANCEL"), i, m.cur.tag);
  if (m.state != ST_FAULT) m.state = ST_IDLE;
  flushQueue(i);
}

void haltAll() { for (uint8_t i = 0; i < NUM_MOTORS; i++) haltMotor(i); }

void stopMotor(uint8_t i, bool flush) {
  Motor &m = M[i];
  if (flush) flushQueue(i);
  if (m.state == ST_RUN) m.stopping = true;
  else if (m.state == ST_DWELL) { m.state = ST_IDLE; evt(F("CANCEL"), i, m.cur.tag); }
}

// ----------------------------- E-stop -----------------------------
void latchEstop() {
  haltAll();
  estopLatched = true;
  driversOn(false);
  Serial.println(F("EVT ESTOP 0 0"));
}

bool estopInputActive() {
#if ESTOP_ENABLED
  return digitalRead(PIN_ESTOP) == (ESTOP_ACTIVE_LOW ? LOW : HIGH);
#else
  return false;
#endif
}

// -------------------------- main service --------------------------
void serviceMotor(uint8_t i, bool tick, float dt) {
  Motor &m = M[i];
  bool la = limActive[i];
  if (la != m.lastLim) {                       // report switch edges
    m.lastLim = la;
    Serial.print(F("EVT LIMSW ")); Serial.print(i + 1); Serial.print(' '); Serial.println(la ? 1 : 0);
  }
  if (limTrip[i]) {                            // ISR halted this motor on a limit
    m.tripDir = isrDir[i];
    limTrip[i] = false;
    clearMotion(i);
    flushQueue(i);
    evtPos(F("LIMIT"), i, m.cur.tag);
    startBackoff(i);
    return;
  }
  switch (m.state) {
    case ST_IDLE:    if (m.qn) beginNext(i); break;
    case ST_DWELL:   if ((int32_t)(millis() - m.dwellEnd) >= 0) complete(i); break;
    case ST_RUN:     if (tick) runProfile(i, dt); break;
    case ST_BACKOFF: runBackoff(i); break;
    case ST_FAULT:   break;
  }
}
