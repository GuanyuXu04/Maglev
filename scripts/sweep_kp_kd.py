#!/usr/bin/env python3
"""Sweep Kp x Kd over the serial link and record a 0.30 mm step response for each.

Firmware (arduino/controller_top/controller_top.ino) is NOT modified. Two things
about it drive the design here:

  * The bare 's' command is *relative* and *hard-coded*: it does
    g_ref_mm += STEP_AMP_MM with STEP_AMP_MM = 0.20 mm. Sending 's' twice walks
    the reference to +0.40 mm -- there is no "step back" command and nothing
    resets it. It also cannot produce the 0.30 mm step we want.
  * "R <mm>" sets the reference *absolutely*, and echoes "R=30.30" from inside
    the same handle() call that assigns g_ref_mm.

So a step here is "R 30.30" and the baseline restore is "R 30.00": exactly
0.30 mm, repeatable an unlimited number of times, and the "R=" echo is the t=0
marker (same role "===== STEP =====" plays for 's'). STEP_AMP_MM is unused.

Per (Kp, Kd) combination:
    1. wait for the user to press Enter
    2. send R (baseline) / KP / KD, verify the echoes
    3. watch 2 s of telemetry, print gap mean+-std and pwm, ask "stable? [Y/n]"
    4. on N -> log it as unstable, next combination (no step attempted)
       on Y -> record PRE_N rows, fire the step, record POST_N rows, save CSV,
               then restore the baseline reference

Output: scripts/sweep_kp_kd/step_kp<..>_kd<..>.csv (one per stable combination)
        scripts/sweep_kp_kd/summary.csv            (every combination, appended
                                                    as it happens, so a crash or
                                                    Ctrl+C loses nothing)
Re-running skips combinations already present in summary.csv, so the sweep can
be done over several sittings. 133 combinations x ~25 s is a long sitting.

Run (from the venv next to this file):
    scripts/.venv/Scripts/python.exe scripts/sweep_kp_kd.py
"""
import collections
import csv
import os
import statistics
import threading
import time

import serial

# ---- hard-coded config ----
PORT = "COM6"
BAUD = 115200
TIMEOUT = 1              # s; short so the reader thread notices the stop flag

KP_LIST = [12.0, 13.0, 14.0, 14.5, 15.0, 15.3, 15.5, 15.8, 15.9,
           16.0, 16.1, 16.2, 16.3, 16.5, 16.8, 17.0, 17.5, 18.0, 19.0]
KD_LIST = [0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45]

REF_BASE = 30.0          # baseline gap (mm); keep inside the [22,49] Hall band
STEP_SIZE = 0.30         # mm; step target is REF_BASE + STEP_SIZE
PRE_N = 200              # telemetry rows before the step (~4 s @ 50 Hz)
POST_N = 1000            # telemetry rows after the step  (~20 s @ 50 Hz)

SETTLE_S = 2.0           # telemetry watched before the stable? prompt
LOST_PWM_ROWS = 15       # consecutive pwm==0 rows that mean "magnet gone"

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sweep_kp_kd")
SUMMARY = os.path.join(OUT_DIR, "summary.csv")


def parse(line):
    """'gap:30.0,ref:32.0,pwm:149' -> (gap, ref, pwm), or None if not telemetry."""
    parts = line.split(",")
    if len(parts) != 3:
        return None
    fields = {}
    for p in parts:
        k, sep, v = p.partition(":")
        if not sep:
            return None
        fields[k.strip()] = v.strip()
    try:
        return float(fields["gap"]), float(fields["ref"]), int(fields["pwm"])
    except (KeyError, ValueError):
        return None      # startup banner, command echo, garbled line


class Trial(object):
    """Per-combination recording state, owned by the reader thread."""

    def __init__(self, step_to):
        self.step_to = step_to
        self.pre = []
        self.post = []
        self.t_send = None      # when we wrote the step command
        self.t_step = None      # when the firmware echoed "R=" == t0
        self.zero_run = 0
        self.lost = False
        self.done = False


class Reader(threading.Thread):
    """Drains the port continuously and runs a trial when one is armed.

    Draining even while idle matters: the OS buffer would otherwise hold seconds
    of stale telemetry from while the user was fiddling with the rig, and the
    first "pre-step" row of a trial would be minutes old.

    The step command is written from *this* thread, the instant the PRE_N'th row
    lands. Letting the main thread poll for that instead would add up to one poll
    period of extra hover and make the pre-step window length sloppy.
    """

    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.lock = threading.Lock()
        self.recent = collections.deque(maxlen=400)   # (t, gap, ref, pwm)
        self.trial = None
        self.stop = False
        self.echoes = collections.deque(maxlen=64)    # non-telemetry lines

    def run(self):
        while not self.stop:
            try:
                raw = self.ser.readline().decode("ascii", "ignore").strip()
            except serial.SerialException:
                break
            if not raw:
                continue
            now = time.time()
            row = parse(raw)

            if row is None:
                with self.lock:
                    self.echoes.append(raw)
                    tr = self.trial
                    # "R=" is the step marker, but only once we have actually
                    # sent the step -- the baseline restore echoes "R=" too.
                    if tr and tr.t_send is not None and tr.t_step is None \
                            and raw.startswith("R="):
                        tr.t_step = now
                continue

            fire = False
            with self.lock:
                self.recent.append((now,) + row)
                tr = self.trial
                if tr is None or tr.done:
                    continue

                if row[2] == 0:                 # pwm==0 -> firmware sees no target
                    tr.zero_run += 1
                    if tr.zero_run >= LOST_PWM_ROWS:
                        tr.lost = True
                        tr.done = True
                        continue
                else:
                    tr.zero_run = 0

                if tr.t_send is None:
                    tr.pre.append((now,) + row)
                    fire = len(tr.pre) >= PRE_N
                elif tr.t_step is None:
                    # command is out but the echo has not come back; the
                    # reference has not moved yet, so these are still pre-step.
                    tr.pre.append((now,) + row)
                else:
                    tr.post.append((now,) + row)
                    if len(tr.post) >= POST_N:
                        tr.done = True

            if fire:
                cmd = "R %.2f\n" % tr.step_to
                self.ser.write(cmd.encode("ascii"))
                self.ser.flush()
                with self.lock:
                    tr.t_send = time.time()

    # ---- main-thread API ----
    def start_trial(self, step_to):
        tr = Trial(step_to)
        with self.lock:
            self.trial = tr
        return tr

    def end_trial(self):
        with self.lock:
            self.trial = None

    def snapshot(self, tr):
        with self.lock:
            return (len(tr.pre), len(tr.post), tr.t_send, tr.t_step,
                    tr.lost, tr.done)

    def window(self, seconds):
        """Telemetry rows from the last `seconds`."""
        cutoff = time.time() - seconds
        with self.lock:
            return [r for r in self.recent if r[0] >= cutoff]

    def drain_echoes(self):
        with self.lock:
            out = list(self.echoes)
            self.echoes.clear()
        return out


def send(ser, rd, cmd, expect, timeout=1.5):
    """Send one line and wait for the firmware's echo. Returns the echo or None.

    Every setter in handle() prints a confirmation, so a missing echo means the
    line never landed -- worth knowing before we attribute a bad step response
    to the gains.
    """
    rd.drain_echoes()
    ser.write((cmd + "\n").encode("ascii"))
    ser.flush()
    deadline = time.time() + timeout
    while time.time() < deadline:
        for e in rd.drain_echoes():
            if e.startswith(expect):
                return e
        time.sleep(0.02)
    return None


def hard_reset(ser):
    """Pulse DTR to reset the board (the Uno/Nano auto-reset cap on RESET).

    This is the software answer to "can we reset the hardware?": yes, but the
    firmware comes back with its compiled-in defaults (Kp=16, Kd=0.3, R=30,
    bias=149) and the coil de-energizes, so the float drops. It is an escape
    hatch for a wedged board, not part of the normal sweep -- 'R 30.00' restores
    the reference without dropping anything.
    """
    ser.setDTR(False)
    time.sleep(0.1)
    ser.setDTR(True)
    time.sleep(2.0)
    ser.reset_input_buffer()


def load_done():
    """(kp, kd) pairs already in summary.csv, so a sweep can be resumed."""
    done = set()
    if not os.path.exists(SUMMARY):
        return done
    with open(SUMMARY, newline="") as f:
        for r in csv.DictReader(f):
            try:
                done.add((round(float(r["kp"]), 3), round(float(r["kd"]), 3)))
            except (KeyError, ValueError, TypeError):
                continue
    return done


def log_summary(kp, kd, stable, n_pre, n_post, note, fname):
    new = not os.path.exists(SUMMARY)
    with open(SUMMARY, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["kp", "kd", "stable", "n_pre", "n_post", "note", "file"])
        w.writerow(["%.2f" % kp, "%.2f" % kd, stable, n_pre, n_post, note, fname])


def save_trial(kp, kd, tr):
    """Write one trial to CSV. Called after end_trial(), so tr is ours alone."""
    fname = "step_kp%.1f_kd%.2f.csv" % (kp, kd)
    path = os.path.join(OUT_DIR, fname)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "phase", "gap_mm", "ref_mm", "pwm"])
        for phase, rows in (("pre", tr.pre), ("post", tr.post)):
            for t, gap, ref, pwm in rows:
                w.writerow(["%.3f" % (t - tr.t_step), phase,
                            "%.1f" % gap, "%.1f" % ref, pwm])
    return fname, path


def run_one(ser, rd, kp, kd):
    """One combination. Returns True to continue the sweep, False to quit."""
    step_to = REF_BASE + STEP_SIZE
    print("\n" + "=" * 62)
    print("  Kp = %.2f   Kd = %.2f" % (kp, kd))
    print("=" * 62)
    ans = input("  Enter = apply gains | s = skip | r = board reset | q = quit : ")
    ans = ans.strip().lower()
    if ans == "q":
        return False
    if ans == "s":
        log_summary(kp, kd, "skip", 0, 0, "user skipped", "")
        return True
    if ans == "r":
        print("  resetting board (the float will drop) ...")
        hard_reset(ser)

    # Baseline first: a previous trial may have left the reference at the step
    # value, and the gains should be changed with the reference already home.
    for cmd, expect in (("R %.2f" % REF_BASE, "R="),
                        ("KP %.3f" % kp, "Kp="),
                        ("KD %.3f" % kd, "Kd=")):
        echo = send(ser, rd, cmd, expect)
        if echo is None:
            print("  !! no echo for '%s' -- board not responding?" % cmd)
            log_summary(kp, kd, "error", 0, 0, "no echo for %s" % cmd, "")
            return True
        print("  -> %s" % echo)

    print("  watching %.0f s of telemetry ..." % SETTLE_S)
    time.sleep(SETTLE_S)
    w = rd.window(SETTLE_S)
    if len(w) < 10:
        print("  !! only %d telemetry rows -- is the board streaming?" % len(w))
    else:
        gaps = [r[1] for r in w]
        pwms = [r[3] for r in w]
        print("  gap %.2f +- %.2f mm   pwm %.0f +- %.0f   ref %.1f mm"
              % (statistics.mean(gaps), statistics.pstdev(gaps),
                 statistics.mean(pwms), statistics.pstdev(pwms), w[-1][2]))

    ans = input("  Stable levitation? [Y/n] ").strip().lower()
    if ans.startswith("n"):
        log_summary(kp, kd, "N", 0, 0, "user: not stable", "")
        print("  logged as unstable, no step taken")
        return True

    # ---- step response ----
    print("  recording: %d pre rows -> step to %.2f mm -> %d post rows"
          % (PRE_N, step_to, POST_N))
    tr = rd.start_trial(step_to)
    try:
        while True:
            n_pre, n_post, t_send, t_step, lost, done = rd.snapshot(tr)
            if lost:
                print("\r  !! levitation lost (pwm=0) -- aborting this combination   ")
                break
            if done:
                print("\r  post-step %d/%d  done                    " % (n_post, POST_N))
                break
            if t_send is None:
                state = "pre-step  %d/%d" % (n_pre, PRE_N)
            elif t_step is None:
                state = "step sent, waiting for R= echo"
                if time.time() - t_send > 5:
                    print("\n  !! no R= echo within 5 s -- did the command land?")
                    break
            else:
                state = "post-step %d/%d" % (n_post, POST_N)
            print("\r  %s          " % state, end="", flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n  interrupted -- saving what we have")
    finally:
        rd.end_trial()

    # Always put the reference back, whatever happened above.
    send(ser, rd, "R %.2f" % REF_BASE, "R=")

    if tr.t_step is None:
        print("  no step marker, nothing saved")
        log_summary(kp, kd, "Y", len(tr.pre), 0, "step marker missing", "")
        return True

    fname, path = save_trial(kp, kd, tr)
    note = "lost levitation" if tr.lost else ""
    if len(tr.post) < POST_N:
        note = (note + "; " if note else "") + "short: %d/%d post rows" % (len(tr.post), POST_N)
    log_summary(kp, kd, "Y", len(tr.pre), len(tr.post), note, fname)
    print("  saved %d pre + %d post rows -> %s" % (len(tr.pre), len(tr.post), fname))

    # The reference is what actually moved; check it did move by STEP_SIZE.
    # ref prints at 1 decimal, so 0.30 reads back as 0.3 -- 0.15 mm of slack
    # absorbs the rounding at both ends.
    if tr.pre and tr.post:
        observed = tr.post[-1][2] - tr.pre[-1][2]
        if abs(observed - STEP_SIZE) > 0.15:
            print("  !! WARNING: ref moved %+.1f mm, expected %+.2f mm"
                  % (observed, STEP_SIZE))
        else:
            print("  ref check ok: %.1f -> %.1f mm" % (tr.pre[-1][2], tr.post[-1][2]))
    return True


def main():
    if not os.path.isdir(OUT_DIR):
        os.makedirs(OUT_DIR)

    done = load_done()
    combos = [(kp, kd) for kp in KP_LIST for kd in KD_LIST]
    todo = [c for c in combos if (round(c[0], 3), round(c[1], 3)) not in done]

    print("Kp x Kd sweep: %d combinations, %d already in summary.csv, %d to go"
          % (len(combos), len(combos) - len(todo), len(todo)))
    print("Step: R %.2f -> R %.2f (%.2f mm). Firmware STEP_AMP_MM / 's' unused."
          % (REF_BASE, REF_BASE + STEP_SIZE, STEP_SIZE))
    if not todo:
        print("nothing to do -- delete summary.csv to start over")
        return

    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    time.sleep(2)                  # the board resets when the port opens
    ser.reset_input_buffer()

    rd = Reader(ser)
    rd.start()
    print("%s @ %d  ->  %s" % (PORT, BAUD, OUT_DIR))

    try:
        for i, (kp, kd) in enumerate(todo, 1):
            print("\n[%d/%d]" % (i, len(todo)), end="")
            if not rd.is_alive():
                print("\nreader thread died -- serial port gone?")
                break
            if not run_one(ser, rd, kp, kd):
                print("\nquit at Kp=%.2f Kd=%.2f" % (kp, kd))
                break
    except KeyboardInterrupt:
        print("\naborted")
    finally:
        rd.end_trial()
        try:
            send(ser, rd, "R %.2f" % REF_BASE, "R=", timeout=0.5)
        except serial.SerialException:
            pass
        rd.stop = True
        rd.join(timeout=2)
        ser.close()
    print("summary -> %s" % SUMMARY)


if __name__ == "__main__":
    main()
