"""stepper4_host.py - Raspberry Pi / Python client for the stepper4 Arduino firmware.

    pip install pyserial

    from stepper4_host import Stepper4
    s = Stepper4("/dev/ttyACM0")
    t1 = s.move(1, 2000, speed=3000)      # returns tag immediately
    t2 = s.move(2, -500)                  # M2 runs while M1 is moving
    s.wait(t1); s.wait(t2)                # block until each finishes
    s.stop(1)                             # cancel M1 (decelerate + flush queue)
"""
import threading
import time
import serial

# Events that end a tagged command (LIMIT ends the move; its later CLEAR carries the same tag)
TERMINAL = {"DONE", "CANCEL", "DROP", "FAIL", "FAULT", "LIMIT", "CLEAR"}


class Stepper4Error(Exception):
    pass


class Stepper4:
    def __init__(self, port="/dev/ttyACM0", baud=115200, on_event=None, watchdog_ms=0):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2.0)                   # Uno resets when the port opens
        self.ser.reset_input_buffer()
        self.on_event = on_event          # callback(list_of_fields) for every EVT line
        self._lock = threading.Lock()
        self._tag = 0
        self._replies = {}                # tag -> (ok, err, extra_lines)
        self._reply_cv = threading.Condition()
        self._done = {}                   # tag -> terminal event fields
        self._done_cv = threading.Condition()
        self._pending_lines = []
        self._run = True
        threading.Thread(target=self._reader, daemon=True).start()
        if watchdog_ms:
            self.cmd(f"WD {watchdog_ms}")
            threading.Thread(target=self._heartbeat, args=(watchdog_ms / 3000,), daemon=True).start()

    # ---------------------------------------------------------- low level
    def _next_tag(self):
        self._tag = self._tag % 65535 + 1
        return self._tag

    def cmd(self, text, timeout=1.0):
        """Send one command, wait for OK/ERR. Returns (tag, extra response lines)."""
        with self._lock:
            tag = self._next_tag()
            self.ser.write(f"#{tag} {text}\n".encode())
        with self._reply_cv:
            if not self._reply_cv.wait_for(lambda: tag in self._replies, timeout):
                raise Stepper4Error(f"timeout: {text}")
            ok, err, extra = self._replies.pop(tag)
        if not ok:
            raise Stepper4Error(f"{text}: {err}")
        return tag, extra

    def _reader(self):
        buf = b""
        while self._run:
            buf += self.ser.read(256)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if line:
                    self._handle(line)

    def _handle(self, line):
        f = line.split()
        if f[0] in ("OK", "ERR"):
            tag = int(f[1][1:]) if len(f) > 1 and f[1].startswith("#") else 0
            err = f[-1] if f[0] == "ERR" else None
            with self._reply_cv:
                self._replies[tag] = (f[0] == "OK", err, self._pending_lines)
                self._pending_lines = []
                self._reply_cv.notify_all()
        elif f[0] == "EVT":
            if len(f) >= 4 and f[1] in TERMINAL:
                with self._done_cv:
                    self._done.setdefault(int(f[3]), f[1:])   # first terminal event wins
                    while len(self._done) > 1000:              # forget tags nobody waited on
                        self._done.pop(next(iter(self._done)))
                    self._done_cv.notify_all()
            if self.on_event:
                self.on_event(f[1:])
        else:                             # STA / INF / CFG lines precede their OK
            self._pending_lines.append(f)

    def _heartbeat(self, period):
        while self._run:
            try:
                self.cmd("PING")
            except Stepper4Error:
                pass
            time.sleep(period)

    # ---------------------------------------------------------- motion
    def move(self, motor, steps, speed=None):
        return self.cmd(f"MOVE {motor} {steps}" + (f" {speed}" if speed else ""))[0]

    def forward(self, motor, steps, speed=None):
        return self.move(motor, abs(steps), speed)

    def backward(self, motor, steps, speed=None):
        return self.move(motor, -abs(steps), speed)

    def goto(self, motor, position, speed=None):
        return self.cmd(f"GOTO {motor} {position}" + (f" {speed}" if speed else ""))[0]

    def jog(self, motor, direction, speed=None):
        return self.cmd(f"JOG {motor} {'+' if direction >= 0 else '-'}" + (f" {speed}" if speed else ""))[0]

    def dwell(self, motor, ms):
        return self.cmd(f"WAIT {motor} {ms}")[0]

    def stop(self, motor=0):  self.cmd(f"STOP {motor}")
    def skip(self, motor):    self.cmd(f"SKIP {motor}")
    def halt(self, motor=0):  self.cmd(f"HALT {motor}")
    def clear(self, motor=0): self.cmd(f"CLEAR {motor}")
    def speed(self, motor, sps):   self.cmd(f"SPEED {motor} {sps}")
    def accel(self, motor, spss):  self.cmd(f"ACCEL {motor} {spss}")
    def zero(self, motor, pos=0):  self.cmd(f"ZERO {motor} {pos}")
    def release(self, motor, direction): self.cmd(f"RELEASE {motor} {'+' if direction >= 0 else '-'}")
    def reset(self, motor=0):  self.cmd(f"RESET {motor}")
    def enable(self, on=True): self.cmd(f"EN {1 if on else 0}")

    def estop(self):
        self.ser.write(b"!")              # real-time byte: acts before any queued line
        self.cmd("ESTOP")

    def status(self, motor=0):
        _, lines = self.cmd(f"STATUS {motor}")
        keys = ("state", "pos", "rate", "queued", "limit", "tag")
        out = {}
        for f in lines:
            if f[0] == "STA":
                d = dict(zip(keys, f[2:]))
                for k in ("pos", "rate", "queued", "limit", "tag"):
                    d[k] = int(d[k])
                out[int(f[1])] = d
        return out

    def wait(self, tag, timeout=None):
        """Block until the command with this tag ends. Returns the terminal event fields,
        e.g. ['DONE', '1', '17']. Raises on FAIL / FAULT / CANCEL / DROP."""
        with self._done_cv:
            if not self._done_cv.wait_for(lambda: tag in self._done, timeout):
                raise Stepper4Error(f"tag {tag}: timeout")
            ev = self._done.pop(tag)
        if ev[0] != "DONE":
            raise Stepper4Error(f"tag {tag}: {' '.join(ev)}")
        return ev

    def close(self):
        self._run = False
        self.ser.close()


if __name__ == "__main__":
    import sys
    s = Stepper4(sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0",
                 on_event=lambda e: print("EVT", *e))
    a = s.move(1, 2000, 3000)
    b = s.move(2, -800)
    time.sleep(0.3)
    print(s.status())
    s.stop(2)                             # cancel M2 mid-move
    print(s.wait(a))
    s.close()
