#!/usr/bin/env python3
"""Collect the calibrate_coil.ino serial stream into a timestamped CSV.

Arduino line format:  pwm, hall_mV, adc16   (duty sweeps 0..255, then wraps to 0)
Output CSV columns:    t_s, pwm, hall_mV, adc16
Output file:           run_<timestamp>_coil.csv   (next to this script)

Run (from the venv created next to this file):
    scripts/.venv/Scripts/python.exe scripts/collect_coil.py
Stop with Ctrl+C (or set SWEEPS below to auto-stop after N full 0..255 sweeps).
The file is flushed after every row so nothing is lost.
"""
import csv
import os
import time
from datetime import datetime

import serial

# ---- hard-coded config ----
PORT = "COM6"
BAUD = 115200
TIMEOUT = 3600          # seconds: effectively "wait forever" for data, never errors
SWEEPS = 0              # 0 = run until Ctrl+C; N = stop after N full 0..255 sweeps


def main():
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), f"run_{ts}_coil.csv")

    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    time.sleep(2)                 # board resets when the port opens
    ser.reset_input_buffer()

    n = 0
    sweeps = 0
    prev_pwm = None
    t0 = time.time()
    print(f"Logging {PORT} @ {BAUD} -> {out_path}   (Ctrl+C to stop)")
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "pwm", "hall_mV", "adc16"])
        try:
            while True:
                raw = ser.readline().decode("ascii", "ignore").strip()
                if not raw:
                    continue      # no data yet -> keep waiting, no error
                parts = [p.strip() for p in raw.split(",")]
                if len(parts) != 3:
                    continue
                try:
                    pwm, mv, adc = int(parts[0]), int(parts[1]), int(parts[2])
                except ValueError:
                    continue      # skip startup / garbled lines
                w.writerow([f"{time.time() - t0:.3f}", pwm, mv, adc])
                f.flush()
                n += 1

                # A wrap from a high duty back to 0 marks one completed sweep.
                if pwm == 0 and prev_pwm not in (None, 0):
                    sweeps += 1
                    print(f"\r{n} rows   sweeps done: {sweeps}                         ",
                          end="", flush=True)
                    if SWEEPS and sweeps >= SWEEPS:
                        break
                prev_pwm = pwm
                if n % 20 == 0:
                    print(f"\r{n} rows   last: pwm={pwm}  hall={mv}mV  adc16={adc}      ",
                          end="", flush=True)
        except KeyboardInterrupt:
            pass
    ser.close()
    print(f"\nSaved {n} rows ({sweeps} full sweeps) to {out_path}")


if __name__ == "__main__":
    main()
