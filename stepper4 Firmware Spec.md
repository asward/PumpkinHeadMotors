# stepper4 Firmware Spec

Sep 28, 2026 · @Andrew

## Purpose and scope

`stepper4.ino` turns an Arduino Uno + CNC Shield V3 (RATTMMOTOR GRBL kit) into a 4-channel independent stepper controller driven over USB serial by a host such as a Raspberry Pi. It **replaces GRBL**: there is no G-code, no coordinated multi-axis interpolation, and each motor runs its own command queue.

- 4 motors (M1–M4 = shield X, Y, Z, A), each with an 8-deep command queue
- Relative moves, absolute moves, continuous jog, timed dwell
- Per-motor speed and acceleration (trapezoidal ramps)
- Per-motor limit switch interlock with automatic back-off
- Cancel per motor (decelerated or instant), software and optional hardware E-stop
- Verified: compiles with no warnings (14.5 KB flash, 923 B RAM); behavior exercised in the simavr ATmega328P simulator

## Hardware and pin map

All four drivers share one enable pin; M4 (A axis) needs the shield's A.STP/A.DIR jumpers fitted to route D12/D13.

| Function    | M1 (X)     | M2 (Y)      | M3 (Z)      | M4 (A)             |
| ----------- | ---------- | ----------- | ----------- | ------------------ |
| STEP        | D2         | D3          | D4          | D12 (jumper)       |
| DIR         | D5         | D6          | D7          | D13 (jumper)       |
| Limit input | D9 (X+/X−) | D10 (Y+/Y−) | D11 (Z+/Z−) | A1 ("Hold" header) |

| Other pin      | Use                               |
| -------------- | --------------------------------- |
| D8             | Driver enable, active LOW, shared |
| A0 ("Abort")   | Optional hardware E-stop input    |
| A2, A3, A4, A5 | Spare (Resume, CoolEn, SDA/SCL)   |

- Both switches of an axis (+ and −) share one input; the firmware knows which end tripped from the direction of travel.
- Default wiring: normally-open switch to GND, internal pull-up. Set `LIMIT_ACTIVE_LOW 0` for normally-closed switches, which also trip on a broken wire (recommended).
- Set driver microstepping with the MS1–MS3 jumpers under each driver; the firmware works in steps only.
- Timer1 is used for stepping, so the Servo library and PWM on D9/D10 are unavailable.

## Architecture

Three layers run concurrently, so any motor can start, stop or be cancelled while the others keep moving.

1. **Step ISR (Timer1, 20 kHz).** A phase accumulator per motor emits a STEP pulse whenever it passes 20,000, counts position, stops bounded moves at zero remaining steps, and enforces the limit interlock. Worst case 243 cycles per tick.
2. **Profile loop (main loop, \~1 kHz).** Recomputes each running motor's speed for the acceleration ramp and starts decelerating when the remaining distance equals the braking distance.
3. **Serial parser and queues (main loop).** Parses one line at a time, answers immediately, and appends motion commands to that motor's 8-slot FIFO. A motor pulls its next command when it returns to IDLE.

&#91;embedded content: per-motor state machine · 5 states\]

Only RUN produces steps under the interlock; BACKOFF steps with it suppressed until the switch opens.

## Motion behavior and speed/accel

Every motion command ramps from 50 steps/s up to its cruise speed at the motor's acceleration, and ramps down so a bounded move lands exactly on its target step count.

| Command type     | Ends when               | Speed used                      |
| ---------------- | ----------------------- | ------------------------------- |
| MOVE (relative)  | step count reached      | per-command, else motor default |
| GOTO (absolute)  | position reached        | per-command, else motor default |
| JOG (continuous) | STOP/SKIP/HALT or limit | per-command, else motor default |
| WAIT (dwell)     | time elapsed            | n/a                             |

- Speed range 1–10,000 steps/s per motor; default 1,000. Acceleration 1–200,000 steps/s²; default 4,000.
- Changing the default speed affects commands that start afterwards, not the one running.
- Positions are signed 32-bit step counts, set by ZERO; they are lost at power-off and when drivers are disabled while a load can move.
- Public C++ API for local use: `motorForward`, `motorBackward`, `motorGoto`, `motorJog`, `motorStop`, `motorHalt`, `motorSetSpeed`, `motorSetAccel`, `motorBusy`, `motorPosition`. All are non-blocking.

## Limit switch interlock and backoff

A closed limit switch stops its motor within one 50 µs tick of the debounced signal, with no deceleration, then the motor reverses at 400 steps/s until the switch opens plus 200 clearance steps.

1. Each input is debounced by an integrating filter: 1 ms of agreement to change state.
2. On trip: step output stops in the ISR; the running command ends (`EVT LIMIT`), and that motor's queue is flushed (`EVT DROP` per entry).
3. Back-off runs opposite to the tripped direction with the interlock suppressed.
4. Once the switch opens, 200 more steps are taken, then `EVT CLEAR` with the final position; the motor returns to IDLE.
5. If the switch is still closed after 4,000 back-off steps, the motor enters FAULT (`LIMIT_STUCK`) and accepts no motion until RESET.

- While a switch is closed, commands toward it are refused (`EVT FAIL … LIMIT`); commands away from it are allowed.
- A switch found closed at boot has no known direction, so all motion is refused until the host sends `RELEASE <m> <dir>`.
- Overshoot before stopping is about 1 ms of travel (3 steps at 3,000 steps/s in simulation). Mount switches with that much mechanical margin.

## Safety: e-stop, watchdogs, faults

Software stops are a convenience, not a safety function: a real machine needs a hardwired E-stop that cuts motor power independently of the Arduino.

| Mechanism       | Trigger                                           | Effect                                     | Recovery                         |
| --------------- | ------------------------------------------------- | ------------------------------------------ | -------------------------------- |
| Real-time halt  | `!` byte on serial                                | All motors stop instantly, queues flushed  | None needed                      |
| Software E-stop | `ESTOP` command, or A0 input if `ESTOP_ENABLED 1` | All stop, drivers disabled, motion refused | `RESET` (input must be released) |
| Host watchdog   | No serial byte for `WD` ms (off by default)       | All motors halt, `EVT WDOG`                | Any byte re-arms it              |
| MCU watchdog    | Firmware hang > 250 ms                            | Board resets; drivers boot disabled        | Automatic                        |
| Limit FAULT     | Switch stays closed during back-off               | That motor frozen                          | `RESET <m>`                      |

- Drivers are enabled automatically at the first move. They stay enabled (holding torque) until `EN 0`, E-stop, or idle power-down: once every motor has been idle for its own `IDLE` time (default `DEFAULT_IDLE_MS`, 0 = hold), the shared EN pin goes high and `EVT DRVOFF 0 0` is sent.
- Opening the USB port resets the Uno (DTR). The host must wait \~2 s for `EVT BOOT` and treat all positions as unknown.

## Configuration constants and limitations

All tunables are `#define`s at the top of the sketch; change them and re-flash.

| Constant                                          | Default          | Meaning                                                |
| ------------------------------------------------- | ---------------- | ------------------------------------------------------ |
| `TICK_HZ`                                         | 20000            | Step ISR rate (Hz)                                     |
| `MAX_SPEED`                                       | 10000            | Ceiling, steps/s (≤ TICK_HZ/2)                         |
| `MIN_SPEED`                                       | 50               | Ramp start/stop speed, steps/s                         |
| `DEFAULT_SPEED` / `DEFAULT_ACCEL`                 | 1000 / 4000      | Boot defaults per motor                                |
| `BACKOFF_SPEED` / `BACKOFF_CLEAR` / `BACKOFF_MAX` | 400 / 200 / 4000 | Limit back-off speed, clearance steps, stuck threshold |
| `LIMIT_FILTER`                                    | 5                | Debounce, in 5 kHz scans (5 = 1 ms)                    |
| `LIMIT_ACTIVE_LOW`                                | 1                | 1 = NO switch to GND; 0 = NC switch                    |
| `ESTOP_ENABLED` / `ESTOP_ACTIVE_LOW`              | 0 / 0            | Hardware E-stop on A0                                  |
| `QLEN`                                            | 8                | Command queue depth per motor                          |
| `DEFAULT_IDLE_MS`                                 | 0                | Boot idle power-down time per motor, ms (0 = hold)     |
| `DIR_INVERT[]`                                    | all false        | Per-motor direction flip                               |

Limitations:

- Step timing is quantized to 50 µs, so pulses jitter up to one tick; fine for A4988/DRV8825, audible at some speeds.
- Worst-case ISR cost is 243 CPU cycles per tick (\~30% CPU with all four motors at 10,000 steps/s).
- No coordinated motion: motors started by separate commands begin a few ms apart.
- Positions are not persisted and there is no homing routine yet.
