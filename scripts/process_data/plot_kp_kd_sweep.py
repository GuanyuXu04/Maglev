#!/usr/bin/env python3
"""Same processing as plot_step_sweep.py, applied to the Kp x Kd sweep instead.

scripts/sweep_kp_kd holds one step_kp<Kp>_kd<Kd>.csv per *stable* (Kp, Kd)
combination the sweep recorded (unstable/skipped combinations have no file --
see summary.csv). Every file uses the same fixed 0.30 mm step (see
sweep_kp_kd.py), so here Kp/Kd are the swept variable instead of step size.

Per combination, one PNG is produced (saved next to the CSVs in DATA_DIR):
  step_kp<Kp>_kd<Kd>.png    filtered gap, recentered to equilibrium = 0

There is no "all traces on one figure" plot here (33 combinations would be an
unreadable tangle) -- instead, steady_state_error_heatmap.png shows settled
center minus the commanded 0.30 mm step as a 2D Kp x Kd heatmap, one cell per
combination. Cells are left blank for combinations with no CSV (unstable/
skipped per summary.csv) AND for combinations diagnose() flags as bad raw
data (see diagnose()'s docstring) -- their individual step-response plot still
gets generated, with the flag noted in its title, but the number is excluded
from the heatmap rather than silently baked in.

Run (from the venv next to this file):
    scripts/.venv/Scripts/python.exe scripts/process_data/plot_kp_kd_sweep.py
"""
import csv
import glob
import os
import re

import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter, filtfilt

DATA_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         "sweep_kp_kd")

STEP_SIZE = 0.30   # mm; fixed for every combination in this sweep

CUTOFF_HZ = 1.0    # low-pass corner; telemetry runs at ~50 Hz
ORDER = 3

SS_CALC_END = 15.0       # steady-state estimate uses only t <= this (see plot_step_sweep.py)
NORM_SETTLE_S = 3.0

FILT_COLOR = "tab:blue"
REF_COLOR = "tab:red"

FIG_SIZE = (8, 6)        # 4:3 width:height for every plot in this script

DEV_XLIM = (-5, 15)
DEV_YLIM = (-0.2, 1.2)

TITLE_FS = 18
LABEL_FS = 16
TICK_FS = 15
LEGEND_FS = 14

FNAME_RE = re.compile(r"step_kp([\d.]+)_kd([\d.]+)\.csv$")


def find_combos():
    combos = []
    for path in sorted(glob.glob(os.path.join(DATA_DIR, "step_kp*_kd*.csv"))):
        m = FNAME_RE.search(os.path.basename(path))
        if m:
            combos.append((float(m.group(1)), float(m.group(2)), path))
    return combos


def load(path):
    t, gap, ref, pwm = [], [], [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            t.append(float(row["t_s"]))
            gap.append(float(row["gap_mm"]))
            ref.append(float(row["ref_mm"]))
            pwm.append(int(row["pwm"]))
    return np.array(t), np.array(gap), np.array(ref), np.array(pwm)


def diagnose(t, gap, pwm, window_end=SS_CALC_END):
    """Flag two raw-data problems seen in this sweep (not filtering artifacts):

    - 'stuck-pre': the Hall reading holds one exact value for >=10 consecutive
      pre-step samples (sensor glitch), which drags the pre_level baseline off
      -- e.g. Kp=15.0/Kd=0.20 sits at a flat 25.2 mm for ~2.5 s of its 4 s
      pre-window instead of the ~27.5 mm every other run shows.
    - 'diverged': gap pins at the sensor's 45 mm ceiling or pwm hits 0 within
      the settled_center() tail window itself, so the "steady-state" estimate
      would be measuring a lost-levitation glitch, not a settled value --
      e.g. Kp=15.0/Kd=0.45 loses levitation outright; Kp=15.5/Kd=0.40 has
      brief pwm=0 blips that happen to land inside the 12-15 s tail window.
    """
    pre = gap[t < 0]
    max_run, run = 1, 1
    for i in range(1, len(pre)):
        run = run + 1 if pre[i] == pre[i - 1] else 1
        max_run = max(max_run, run)
    stuck_pre = max_run >= 10

    tail_mask = (t > window_end - NORM_SETTLE_S) & (t <= window_end)
    diverged = bool(np.any(gap[tail_mask] >= 44.9) or np.any(pwm[tail_mask] == 0))

    reasons = []
    if stuck_pre:
        reasons.append("stuck-pre baseline")
    if diverged:
        reasons.append("diverged/saturated in tail window")
    return reasons


def lowpass(t, x, cutoff_hz=CUTOFF_HZ, order=ORDER):
    fs = 1.0 / np.median(np.diff(t))
    b, a = butter(order, cutoff_hz / (fs / 2.0), btype="low")
    return filtfilt(b, a, x)


def settled_center(t, dev, window_end=SS_CALC_END):
    mask = (t > window_end - NORM_SETTLE_S) & (t <= window_end)
    return dev[mask].mean()


def plot_deviation(kp, kd, t, dev, flags=()):
    ref_ideal = np.where(t < 0, 0.0, STEP_SIZE)

    fig, ax = plt.subplots(figsize=FIG_SIZE)
    ax.plot(t, dev, color=FILT_COLOR, linewidth=1.6, label="gap (filtered, %.0f Hz LP)" % CUTOFF_HZ)
    ax.plot(t, ref_ideal, color=REF_COLOR, linewidth=1, linestyle="--", label="ref (idealized target)")
    ax.axhline(0, color="gray", linewidth=0.8, linestyle=":")
    ax.axvline(0, color="gray", linewidth=0.8, linestyle=":")

    ax.set_xlim(*DEV_XLIM)
    ax.set_ylim(*DEV_YLIM)

    ax.set_xlabel("time [s]", fontsize=LABEL_FS)
    ax.set_ylabel("air gap (y - y_ref) [mm]", fontsize=LABEL_FS)
    title = "Step response: Kp=%.2f, Kd=%.2f, step = %.2f mm" % (kp, kd, STEP_SIZE)
    if flags:
        title += "\n[flagged: %s]" % ", ".join(flags)
    ax.set_title(title, fontsize=TITLE_FS)
    ax.tick_params(axis="both", labelsize=TICK_FS)
    ax.legend(loc="best", fontsize=LEGEND_FS)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_ss_error_heatmap(results):
    kps = sorted({kp for kp, kd, _ in results})
    kds = sorted({kd for kp, kd, _ in results})
    grid = np.full((len(kps), len(kds)), np.nan)
    for kp, kd, err in results:
        grid[kps.index(kp), kds.index(kd)] = err

    fig, ax = plt.subplots(figsize=FIG_SIZE)
    masked = np.ma.masked_invalid(grid)
    cmap = plt.get_cmap("viridis").copy()
    cmap.set_bad(color="0.85")
    im = ax.imshow(masked, cmap=cmap, aspect="auto", origin="lower")

    ax.set_xticks(range(len(kds)))
    ax.set_xticklabels(["%.2f" % kd for kd in kds])
    ax.set_yticks(range(len(kps)))
    ax.set_yticklabels(["%.1f" % kp for kp in kps])

    for i in range(len(kps)):
        for j in range(len(kds)):
            if not np.isnan(grid[i, j]):
                ax.text(j, i, "%.2f" % grid[i, j], ha="center", va="center",
                        fontsize=9, color="white" if grid[i, j] > np.nanmean(grid) else "black")

    cbar = fig.colorbar(im, ax=ax)
    cbar.set_label("steady-state error [mm]", fontsize=LABEL_FS)
    cbar.ax.tick_params(labelsize=TICK_FS)

    ax.set_xlabel("Kd", fontsize=LABEL_FS)
    ax.set_ylabel("Kp", fontsize=LABEL_FS)
    ax.set_title("Steady-state error: Kp x Kd (step = %.2f mm)" % STEP_SIZE, fontsize=TITLE_FS)
    ax.tick_params(axis="both", labelsize=TICK_FS)
    fig.tight_layout()
    return fig


def main():
    combos = find_combos()
    print("found %d stable (Kp, Kd) combinations" % len(combos))

    results = []
    excluded = []
    for kp, kd, path in combos:
        t, gap, ref, pwm = load(path)
        gap_f = lowpass(t, gap)

        pre_level = gap_f[t < 0].mean()
        dev = gap_f - pre_level

        flags = diagnose(t, gap, pwm)

        fig = plot_deviation(kp, kd, t, dev, flags)
        out_path = os.path.join(DATA_DIR, "step_kp%.1f_kd%.2f.png" % (kp, kd))
        fig.savefig(out_path, dpi=150, bbox_inches="tight")
        plt.close(fig)

        center = settled_center(t, dev)
        ss_error = center - STEP_SIZE
        tag = " [EXCLUDED: %s]" % ", ".join(flags) if flags else ""
        print("Kp=%.2f Kd=%.2f: settled center %.3f mm, steady-state error %.3f mm%s"
              % (kp, kd, center, ss_error, tag))

        if flags:
            excluded.append((kp, kd, flags))
        else:
            results.append((kp, kd, ss_error))

    if excluded:
        print("\n%d combination(s) excluded from the heatmap:" % len(excluded))
        for kp, kd, flags in excluded:
            print("  Kp=%.2f Kd=%.2f: %s" % (kp, kd, ", ".join(flags)))

    fig = plot_ss_error_heatmap(results)
    out_path = os.path.join(DATA_DIR, "steady_state_error_heatmap.png")
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("saved %s" % out_path)


if __name__ == "__main__":
    main()
