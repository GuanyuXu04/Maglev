#!/usr/bin/env python3
"""Record one step response from controller_top.ino into data_step_<size>.csv.

The firmware is untouched: sending a bare 's' bumps g_ref_mm by STEP_AMP_MM and
echoes "===== STEP =====" in the same loop pass that changes the reference, so
that marker line -- not the moment we write the byte -- is t = 0 here.

Telemetry is the stock 50 Hz stream:  gap:30.0,ref:32.0,pwm:149
so PRE_N=200 rows is ~4 s of hover and POST_N=1000 rows is ~20 s of response.

Sequence, once you press Enter: record PRE_N rows -> send 's' automatically ->
record POST_N rows -> save. Nothing is logged before the keypress, so you decide
when the levitation has settled and the pre-step window is exactly PRE_N rows.

Manual sweep procedure (0.25 -> 2.00 mm in 0.25 steps):
    1. edit STEP_AMP_MM in arduino/controller_top/controller_top.ino, upload
    2. edit STEP_SIZE below to the same value
    3. run this script, wait for stable hover, press Enter
The script cross-checks the two by watching 'ref' in the telemetry and shouts if
they disagree, since keeping them in sync by hand is the easy thing to get wrong.

Run (from the venv next to this file):
    scripts/.venv/Scripts/python.exe scripts/collect_step.py
"""
import csv
import os
import threading
import time

import serial

# ---- hard-coded config ----
PORT = "COM6"
BAUD = 115200
TIMEOUT = 1             # s; short so the reader thread can notice the stop flag
STEP_SIZE = 0.20        # mm -- MUST equal STEP_AMP_MM in controller_top.ino
PRE_N = 200             # telemetry rows recorded before the step (~4 s)
POST_N = 1000           # telemetry rows recorded after the step  (~20 s)

STEP_MARKER = "===== STEP ====="


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


class Reader(threading.Thread):
    """Drains the port and runs the whole sequence once armed.

    It fires the 's' itself the instant the 200th pre-step row lands, rather than
    letting the main thread poll for that: polling would add up to one poll period
    of extra hover before the step and make the pre-window length sloppy.
    Draining continuously (even while idle) keeps the first armed row fresh --
    there is never a backlog of stale lines sitting in the OS buffer.
    """

    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.lock = threading.Lock()
        self.armed = False
        self.pre = []
        self.post = []
        self.t_send = None      # when we wrote the 's'
        self.t_step = None      # when the firmware echoed the marker == t0
        self.stop = False

    def run(self):
        while not self.stop:
            try:
                raw = self.ser.readline().decode("ascii", "ignore").strip()
            except serial.SerialException:
                break
            if not raw:
                continue
            now = time.time()
            with self.lock:
                if not self.armed:
                    continue            # idle: read and discard, keep the port drained
            if STEP_MARKER in raw:
                with self.lock:
                    if self.t_step is None:
                        self.t_step = now
                continue
            row = parse(raw)
            if row is None:
                continue
            with self.lock:
                if self.t_send is None:
                    self.pre.append((now,) + row)
                    fire = len(self.pre) >= PRE_N
                elif self.t_step is None:
                    # 's' is out but the marker has not come back yet; these rows
                    # are still pre-step as far as the reference is concerned.
                    self.pre.append((now,) + row)
                    fire = False
                else:
                    self.post.append((now,) + row)
                    fire = False
                    if len(self.post) >= POST_N:
                        self.stop = True
            if fire:
                self.ser.write(b"s")
                self.ser.flush()
                with self.lock:
                    self.t_send = time.time()

    def arm(self):
        with self.lock:
            self.armed = True

    def counts(self):
        with self.lock:
            return len(self.pre), len(self.post), self.t_send, self.t_step


def main():
    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            f"data_step_{STEP_SIZE:.2f}.csv")
    if os.path.exists(out_path):
        if input(f"{os.path.basename(out_path)} exists. Overwrite? [y/N] ").strip().lower() != "y":
            print("aborted")
            return

    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    time.sleep(2)                 # board resets when the port opens
    ser.reset_input_buffer()

    rd = Reader(ser)
    rd.start()
    print(f"{PORT} @ {BAUD}  step={STEP_SIZE:.2f} mm  ->  {out_path}")
    print(f"Wait for stable hover, then press Enter to start: "
          f"{PRE_N} rows pre-step, then 's' fires automatically, "
          f"then {POST_N} rows post-step. (Ctrl+C aborts.)")
    try:
        input()
    except KeyboardInterrupt:
        rd.stop = True
        ser.close()
        print("\naborted")
        return
    if not rd.is_alive():
        print("reader died before arming -- is the board streaming?")
        ser.close()
        return
    rd.arm()

    try:
        while rd.is_alive():
            n_pre, n_post, t_send, t_step = rd.counts()
            if t_send is None:
                state = f"pre-step {n_pre}/{PRE_N}"
            elif t_step is None:
                state = "'s' sent, waiting for STEP marker"
                if time.time() - t_send > 5:
                    print("\nno STEP marker within 5 s -- did the 's' reach the board?")
                    break
            else:
                state = f"post-step {n_post}/{POST_N}"
            print(f"\r  {state}                    ", end="", flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nstopped early -- writing what we have")
    rd.stop = True
    rd.join(timeout=2)
    ser.close()

    with rd.lock:
        pre, post, t_step = list(rd.pre), list(rd.post), rd.t_step
    if t_step is None:
        print("no step recorded, nothing saved")
        return

    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "phase", "gap_mm", "ref_mm", "pwm"])
        for phase, rows in (("pre", pre), ("post", post)):
            for t, gap, ref, pwm in rows:
                w.writerow([f"{t - t_step:.3f}", phase, f"{gap:.1f}", f"{ref:.1f}", pwm])

    print(f"\nSaved {len(pre)} pre + {len(post)} post rows to {out_path}")

    # The one hand-sync error this experiment is exposed to: firmware STEP_AMP_MM
    # edited but STEP_SIZE here (or vice versa) left behind. 'ref' tells the truth.
    if pre and post:
        observed = post[-1][2] - pre[-1][2]
        # ref is printed at 1 decimal, so a 0.25 mm step reads back as 0.2 or 0.3;
        # 0.15 mm of slack absorbs the rounding on both endpoints.
        if abs(observed - STEP_SIZE) > 0.15:
            print(f"  !! WARNING: telemetry ref moved {observed:+.1f} mm but "
                  f"STEP_SIZE says {STEP_SIZE:.2f} mm.")
            print("  !! The firmware's STEP_AMP_MM and this script disagree -- "
                  "the file name is wrong. Fix and re-run.")
        else:
            print(f"  ref check ok: {pre[-1][2]:.1f} -> {post[-1][2]:.1f} mm")
    if len(post) < POST_N:
        print(f"  note: only {len(post)}/{POST_N} post-step rows captured")


if __name__ == "__main__":
    main()
