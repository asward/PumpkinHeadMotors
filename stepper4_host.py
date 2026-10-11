"""stepper4_host.py - Raspberry Pi / Python client for the stepper4 Arduino firmware.

    pip install pyserial

    from stepper4_host import Stepper4
    s = Stepper4("/dev/ttyACM0")
    t1 = s.move(1, 2000, speed=3000)      # returns tag immediately
    t2 = s.move(2, -500)                  # M2 runs while M1 is moving
    s.wait(t1); s.wait(t2)                # block until each finishes
    s.stop(1)                             # cancel M1 (decelerate + flush queue)

Console (the default when run as a script): type raw protocol commands, or pipe them in
from another program; replies and events are written to stdout, one per line.

    python stepper4_host.py /dev/ttyUSB0 --speed 2000 --accel 1:8000 --idle 1000
    other_program | python stepper4_host.py /dev/ttyUSB0 --idle 1000
"""
import argparse
import sys
import threading
import time
import serial

# Events that end a tagged command (LIMIT ends the move; its later CLEAR carries the same tag)
TERMINAL = {"DONE", "CANCEL", "DROP", "FAIL", "FAULT", "LIMIT", "CLEAR"}


class Stepper4Error(Exception):
    pass


class Stepper4:
    def __init__(self, port="/dev/ttyUSB0", baud=115200, on_event=None, watchdog_ms=0):
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
    def idle(self, motor, ms):     self.cmd(f"IDLE {motor} {ms}")   # driver power-down delay, 0 = hold
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


CONSOLE_HELP = """\
Type any protocol command (e.g. MOVE 1 2000 3000, STATUS, INFO). Tags are added
automatically and shown in the reply; EVT lines print as they arrive.
  !            halt everything now (real-time byte)
  wait <tag>   block until that command ends (Ctrl-C to give up)
  help         this text
  quit / exit  leave (Ctrl-D also works)"""

_out_lock = threading.Lock()
_interactive = sys.stdin.isatty()


def _out(text):
    """Write one line to stdout. On a terminal, redraw the prompt and anything typed so far."""
    with _out_lock:
        if _interactive:
            try:
                import readline
                buf = readline.get_line_buffer()
            except ImportError:
                buf = ""
            sys.stdout.write("\r\033[K" + text + "\ns4> " + buf)
            sys.stdout.flush()
        else:
            print(text, flush=True)


def run_line(s, line):
    """Run one console line, printing its reply. Returns False on ERR or bad input."""
    word = line.split()[0].lower()
    try:
        if word == "help":
            print(CONSOLE_HELP)
        elif line == "!":
            s.ser.write(b"!")
        elif word == "wait":
            print(" ".join(s.wait(int(line.split()[1]))), flush=True)
        else:
            if line.startswith("#"):          # drop a user tag; cmd() adds its own
                line = line.split(None, 1)[1] if " " in line else ""
            tag, extra = s.cmd(line)
            with _out_lock:
                for f in extra:
                    print(" ".join(f))
                print(f"OK #{tag}", flush=True)
        return True
    except Stepper4Error as e:
        print("ERR", e, flush=True)
    except (IndexError, ValueError):
        print("ERR usage: wait <tag>", flush=True)
    return False


def console(s):
    prompt = ""
    if _interactive:
        try:
            import readline  # noqa: F401  (line editing + history for input())
        except ImportError:
            pass
        print(CONSOLE_HELP)
        prompt = "s4> "
    while True:
        try:
            line = input(prompt).strip()
        except EOFError:                      # Ctrl-D, or the feeding program closed the pipe
            if _interactive:
                print()
            return
        except KeyboardInterrupt:
            print()
            return
        if not line:
            continue
        if line.lower() in ("quit", "exit"):
            return
        try:
            run_line(s, line)
        except KeyboardInterrupt:             # Ctrl-C during "wait" only abandons the wait
            print()


def demo(s):
    a = s.move(1, 2000, 3000)
    s.move(2, -800)
    time.sleep(0.3)
    print(s.status())
    s.stop(2)                             # cancel M2 mid-move
    print(s.wait(a))


def _per_motor(verb):
    """argparse type for VALUE (all motors) or MOTOR:VALUE -> 'VERB m value'."""
    def parse(v):
        m, _, val = v.rpartition(":")
        try:
            return f"{verb} {int(m or 0)} {int(val)}"
        except ValueError:
            raise argparse.ArgumentTypeError(f"expected VALUE or MOTOR:VALUE, got {v!r}")
    return parse


if __name__ == "__main__":
    ap = argparse.ArgumentParser(
        description="stepper4 console. Startup options run in the order given, then the console "
                    "reads commands from stdin (a terminal or a pipe from another program).")
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--watchdog", type=int, default=0, metavar="MS", help="enable host watchdog")
    ap.add_argument("--speed", action="append", dest="startup", type=_per_motor("SPEED"),
                    metavar="[M:]SPS", help="default speed, steps/s (all motors, or motor M)")
    ap.add_argument("--accel", action="append", dest="startup", type=_per_motor("ACCEL"),
                    metavar="[M:]SPSS", help="acceleration, steps/s^2")
    ap.add_argument("--idle", action="append", dest="startup", type=_per_motor("IDLE"),
                    metavar="[M:]MS", help="idle time before drivers power down, 0 = hold")
    ap.add_argument("-c", "--cmd", action="append", dest="startup", metavar="CMD",
                    help='any protocol command, e.g. -c "ZERO 1"')
    ap.add_argument("--demo", action="store_true", help="run the scripted two-motor demo instead")
    args = ap.parse_args()

    s = Stepper4(args.port, args.baud, watchdog_ms=args.watchdog,
                 on_event=lambda e: _out("EVT " + " ".join(e)))
    try:
        for line in args.startup or []:
            if not run_line(s, line):
                sys.exit(f"startup command failed: {line}")
        demo(s) if args.demo else console(s)
    finally:
        s.close()
