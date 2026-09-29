/*
 * serial_api.cpp — Line reader, tokenizer, command dispatch, replies, host watchdog.
 */
#include "serial_api.h"
#include "config.h"
#include "motor.h"
#include "stepgen.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// Host watchdog state
static uint32_t hostWdMs = 0, lastRx = 0;
static bool     hostWdFired = false;

// Line assembly buffer
static char    lineBuf[64];
static uint8_t lineLen = 0;
static bool    lineOverflow = false;

// ----------------------------- parsing -----------------------------
// Strict base-10 integer: whole string must parse
static bool parseNum(const char *s, long *out) {
  if (!*s) return false;
  char *e; long v = strtol(s, &e, 10);
  if (*e) return false;
  *out = v; return true;
}

#define ALL 4   // parseMotor() result for "0" = every motor
static int8_t parseMotor(const char *s, bool allowAll) {
  long v;
  if (*s == 'M') s++;                          // accept "M1" or "1"
  if (!parseNum(s, &v)) return -1;
  if (v == 0 && allowAll) return ALL;
  if (v < 1 || v > NUM_MOTORS) return -1;
  return (int8_t)(v - 1);
}

static int8_t parseDir(const char *s) {
  if (!strcmp(s, "+") || !strcmp(s, "1") || !strcmp(s, "+1") || !strcmp(s, "F")) return 1;
  if (!strcmp(s, "-") || !strcmp(s, "-1") || !strcmp(s, "R")) return -1;
  return 0;
}

// ----------------------------- output -----------------------------
static const __FlashStringHelper *stateName(MState s) {
  switch (s) {
    case ST_IDLE: return F("IDLE"); case ST_RUN: return F("RUN"); case ST_DWELL: return F("DWELL");
    case ST_BACKOFF: return F("BACKOFF"); default: return F("FAULT");
  }
}

// STA <motor> <state> <pos> <signed rate> <queued> <limit> <tag>
static void printStatus(uint8_t i) {
  Motor &m = M[i];
  Serial.print(F("STA ")); Serial.print(i + 1); Serial.print(' ');
  Serial.print(estopLatched ? F("ESTOP") : stateName(m.state)); Serial.print(' ');
  Serial.print(getPos(i)); Serial.print(' ');
  Serial.print((long)getRate(i) * isrDir[i]); Serial.print(' ');
  Serial.print(m.qn); Serial.print(' ');
  Serial.print(limActive[i] ? 1 : 0); Serial.print(' ');
  Serial.println((m.state == ST_IDLE) ? 0 : m.cur.tag);
}

// ---------------------------- dispatch ----------------------------
// Iterate i over one motor, or all of them when mm == ALL
#define FOR_MOTORS(mm) for (uint8_t i = ((mm) == ALL ? 0 : (mm)); i < ((mm) == ALL ? NUM_MOTORS : (mm) + 1); i++)

// Execute one command. c = verb, a = na arguments. Returns NULL (OK) or an error name.
static const __FlashStringHelper *dispatch(const char *c, char **a, uint8_t na, uint16_t tag) {
  long v, w; int8_t m;

  if (!strcmp(c, "PING")) return NULL;

  if (!strcmp(c, "INFO")) {
    Serial.print(F("INF FW " FW_VERSION " MOTORS ")); Serial.print(NUM_MOTORS);
    Serial.print(F(" MAXSPEED ")); Serial.print(MAX_SPEED);
    Serial.print(F(" QLEN ")); Serial.print(QLEN);
    Serial.print(F(" ESTOP_IN ")); Serial.print(ESTOP_ENABLED);
    Serial.print(F(" WD ")); Serial.println(hostWdMs);
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      Serial.print(F("CFG ")); Serial.print(i + 1);
      Serial.print(' '); Serial.print(M[i].defSpeed);
      Serial.print(' '); Serial.println(M[i].accel);
    }
    return NULL;
  }

  if (!strcmp(c, "STATUS")) {
    if (na == 0) m = ALL;
    else if ((m = parseMotor(a[0], true)) < 0) return F("BADMOTOR");
    FOR_MOTORS(m) printStatus(i);
    return NULL;
  }

  if (!strcmp(c, "MOVE") || !strcmp(c, "GOTO") || !strcmp(c, "JOG")) {
    if (na < 2 || na > 3) return F("SYNTAX");
    if ((m = parseMotor(a[0], false)) < 0) return F("BADMOTOR");
    uint8_t type;
    if (c[0] == 'J') { if (!(v = parseDir(a[1]))) return F("BADARG"); type = C_JOG; }
    else { if (!parseNum(a[1], &v)) return F("BADARG"); type = (c[0] == 'M') ? C_MOVE : C_GOTO; }
    uint16_t spd = 0;
    if (na == 3) {
      if (!parseNum(a[2], &w) || w < 1 || w > (long)MAX_SPEED) return F("BADSPEED");
      spd = (uint16_t)w;
    }
    return enqueue(m, type, v, spd, tag);
  }

  if (!strcmp(c, "WAIT")) {
    if (na != 2) return F("SYNTAX");
    if ((m = parseMotor(a[0], false)) < 0) return F("BADMOTOR");
    if (!parseNum(a[1], &v) || v < 0 || v > 3600000L) return F("BADARG");
    return enqueue(m, C_WAIT, v, 0, tag);
  }

  if (!strcmp(c, "SPEED") || !strcmp(c, "ACCEL")) {
    if (na != 2) return F("SYNTAX");
    if ((m = parseMotor(a[0], true)) < 0) return F("BADMOTOR");
    if (!parseNum(a[1], &v)) return F("BADARG");
    if (c[0] == 'S') {
      if (v < 1 || v > (long)MAX_SPEED) return F("BADSPEED");
      FOR_MOTORS(m) M[i].defSpeed = (uint16_t)v;
    } else {
      if (v < 1 || v > (long)MAX_ACCEL) return F("BADARG");
      FOR_MOTORS(m) M[i].accel = (uint32_t)v;
    }
    return NULL;
  }

  if (!strcmp(c, "STOP") || !strcmp(c, "HALT") || !strcmp(c, "SKIP") || !strcmp(c, "CLEAR")) {
    if (na == 0) m = ALL;
    else if ((m = parseMotor(a[0], true)) < 0) return F("BADMOTOR");
    FOR_MOTORS(m) {
      if (c[0] == 'H') haltMotor(i);
      else if (c[1] == 'T') stopMotor(i, true);   // STOP
      else if (c[1] == 'K') stopMotor(i, false);  // SKIP
      else flushQueue(i);                        // CLEAR
    }
    return NULL;
  }

  if (!strcmp(c, "ZERO")) {
    if (na < 1 || na > 2) return F("SYNTAX");
    if ((m = parseMotor(a[0], true)) < 0) return F("BADMOTOR");
    v = 0;
    if (na == 2 && !parseNum(a[1], &v)) return F("BADARG");
    FOR_MOTORS(m) if (M[i].state != ST_IDLE && M[i].state != ST_FAULT) return F("BUSY");
    FOR_MOTORS(m) { uint8_t s = SREG; cli(); isrPos[i] = v; SREG = s; }
    return NULL;
  }

  if (!strcmp(c, "EN")) {
    if (na != 1 || !parseNum(a[0], &v) || v < 0 || v > 1) return F("SYNTAX");
    if (v) { if (estopLatched) return F("ESTOP"); driversOn(true); }
    else   { haltAll(); driversOn(false); }
    return NULL;
  }

  if (!strcmp(c, "RELEASE")) {                 // manual escape from a closed switch
    if (na != 2) return F("SYNTAX");
    if ((m = parseMotor(a[0], false)) < 0) return F("BADMOTOR");
    int8_t d = parseDir(a[1]);
    if (!d) return F("BADARG");
    if (estopLatched) return F("ESTOP");
    if (!limActive[m]) return F("NOTLIMIT");
    if (M[m].state != ST_IDLE && M[m].state != ST_FAULT) return F("BUSY");
    M[m].tripDir = -d;
    M[m].cur.tag = tag;
    startBackoff(m);
    return NULL;
  }

  if (!strcmp(c, "RESET")) {
    if (na == 0) m = ALL;
    else if ((m = parseMotor(a[0], true)) < 0) return F("BADMOTOR");
    if (estopLatched) {
      if (estopInputActive()) return F("ESTOP");
      estopLatched = false;
    }
    FOR_MOTORS(m) if (M[i].state == ST_FAULT) M[i].state = ST_IDLE;
    return NULL;
  }

  if (!strcmp(c, "ESTOP")) { latchEstop(); return NULL; }

  if (!strcmp(c, "WD")) {
    if (na != 1 || !parseNum(a[0], &v) || v < 0 || v > 60000L) return F("SYNTAX");
    hostWdMs = (uint32_t)v; hostWdFired = false;
    return NULL;
  }

  return F("UNKNOWN");
}

// OK [#tag] | ERR [#tag] <code>
static void reply(uint16_t tag, const __FlashStringHelper *err) {
  Serial.print(err ? F("ERR") : F("OK"));
  if (tag) { Serial.print(F(" #")); Serial.print(tag); }
  if (err) { Serial.print(' '); Serial.print(err); }
  Serial.println();
}

// Tokenize a complete line, strip optional "#tag", dispatch, reply
static void handleLine(char *s) {
  char *tok[6]; uint8_t n = 0;
  for (char *t = strtok(s, " \t"); t && n < 6; t = strtok(NULL, " \t")) tok[n++] = t;
  if (!n) return;
  uint16_t tag = 0; uint8_t k = 0;
  if (tok[0][0] == '#') {
    long v;
    if (!parseNum(tok[0] + 1, &v) || v < 1 || v > 65535L) { reply(0, F("BADTAG")); return; }
    tag = (uint16_t)v; k = 1;
  }
  if (k >= n) { reply(tag, F("SYNTAX")); return; }
  reply(tag, dispatch(tok[k], &tok[k + 1], n - k - 1, tag));
}

// ----------------------------- public -----------------------------
void serialApiBegin() {
  Serial.begin(BAUD);
  lastRx = millis();
}

void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    lastRx = millis(); hostWdFired = false;
    if (c == '!') {                            // real-time: halt everything now
      haltAll();
      Serial.println(F("EVT HALT 0 0"));
      continue;
    }
    if (c == '\n' || c == '\r') {
      if (lineOverflow) Serial.println(F("ERR LINE_TOO_LONG"));
      else if (lineLen) { lineBuf[lineLen] = 0; handleLine(lineBuf); }
      lineLen = 0; lineOverflow = false;
      continue;
    }
    if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = (char)toupper((unsigned char)c);
    else lineOverflow = true;
  }
}

void checkHostWatchdog() {
  if (hostWdMs && !hostWdFired && (millis() - lastRx) > hostWdMs) {
    haltAll();
    hostWdFired = true;
    Serial.println(F("EVT WDOG 0 0"));
  }
}
