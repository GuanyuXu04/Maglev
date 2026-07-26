#!/usr/bin/env python3
"""Plot gap_mm vs t_s for selected data_step_*.csv files in sweep_step_kp16_kd0.3.

The raw Hall gap reading is noisy, so each trace is filtered with a zero-phase
Butterworth low-pass (scipy.signal.filtfilt -- no phase lag, so the filtered
curve stays aligned with the step marker at t=0) before use. Only the filtered
trace is plotted (no raw overlay).

Only the step sizes in STEP_SIZES are handled (0.15/0.20/0.30/0.40/0.60 mm),
per request -- 0.25, 0.35 and 0.80 mm are skipped even though the CSVs exist.

The 0.60 mm run loses levitation partway through (the Hall reading pins at the
sensor's max value once the magnet is gone). That data is NOT cropped before
plotting -- the full filtered trace is drawn -- it just runs off the top of
the fixed y-range (DEV_YLIM) once it saturates, which is the intended way to
show "it diverged" without a manual x cutoff. For the steady-state-error
*calculation* only, SS_WINDOW_END still bounds the settled-value estimate to
the brief pre-divergence window (0.60 mm never reaches an actual steady
state, so this is the best available proxy).

Each trace is recentered so its own pre-step average -- physically the 30 mm
equilibrium -- sits at 0 (mm deviation from equilibrium). All plots (except
the steady-state-error summary) share an x-range of -5..15 s.

Four kinds of PNG are produced, all in DATA_DIR:
  data_step_<x>.png             filtered gap, recentered to equilibrium = 0
  data_step_<x>_normalized.png  filtered gap rescaled 0 (pre-step) -> 1
                                 (settled); all five share the same x/y
                                 limits so rise time / overshoot compare
                                 directly across step sizes
  step_comparison.png           all five recentered traces on one figure
  steady_state_error.png        settled-center minus commanded step, vs step size

Run (from the venv next to this file):
    scripts/.venv/Scripts/python.exe scripts/process_data/plot_step_sweep.py
"""
import csv
import os

import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter, filtfilt

DATA_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         "sweep_step_kp16_kd0.3")

STEP_SIZES = ["0.15", "0.20", "0.30", "0.40", "0.60"]

# Steady-state estimate uses only t <= SS_CALC_END: the 0.30 mm run visibly
# drifts/grows past t=15s (still stable, just not settled), so anything later
# skews the "settled center" estimate. 0.60 mm gets its own, earlier window
# since it diverges (loses levitation) well before 15 s.
SS_CALC_END = 15.0
SS_WINDOW_END = {"0.60": 6.0}   # step_size -> last t_s still pre-divergence

CUTOFF_HZ = 1.0    # low-pass corner; telemetry runs at ~50 Hz
ORDER = 3

SS_ERROR_STEPS = ["0.15", "0.20", "0.30", "0.40"]   # 0.60 excluded: never reaches steady state

FILT_COLOR = "tab:blue"
REF_COLOR = "tab:red"
COMBINED_COLORS = ["tab:blue", "tab:orange", "tab:green", "tab:purple", "tab:brown"]

FIG_SIZE = (8, 6)        # 4:3 width:height for every plot in this script

DEV_XLIM = (-5, 15)      # shared x-range for the recentered/deviation plots
DEV_YLIM = (-0.2, 1.2)   # shared y-range for the recentered/deviation plots

TITLE_FS = 18
LABEL_FS = 16
TICK_FS = 15
LEGEND_FS = 14

NORM_SETTLE_S = 3.0      # window at the tail used as the "settled" level
NORM_XLIM = (-5, 15)
NORM_YLIM = (-0.5, 1.8)


def load(path):
    t, gap, ref = [], [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            t.append(float(row["t_s"]))
            gap.append(float(row["gap_mm"]))
            ref.append(float(row["ref_mm"]))
    return np.array(t), np.array(gap), np.array(ref)


def lowpass(t, x, cutoff_hz=CUTOFF_HZ, order=ORDER):
    fs = 1.0 / np.median(np.diff(t))
    b, a = butter(order, cutoff_hz / (fs / 2.0), btype="low")
    return filtfilt(b, a, x)


def settled_center(step, t, dev):
    """Mean deviation over the last NORM_SETTLE_S seconds of the usable window.

    The usable window ends at SS_CALC_END (15 s) for the steps that stay
    stable -- e.g. 0.30 mm keeps oscillating with growing amplitude past 15 s,
    which is real drift, not settling, so including it would bias the
    estimate. 0.60 mm gets its own, earlier window from SS_WINDOW_END since it
    diverges well before 15 s.
    """
    window_end = SS_WINDOW_END.get(step, SS_CALC_END)
    mask = (t > window_end - NORM_SETTLE_S) & (t <= window_end)
    return dev[mask].mean()


def plot_deviation(step, t, dev):
    # Idealized ref: 0 before the step, exactly the commanded step size after --
    # drawn this way (instead of the real ref_mm - pre_level trace) so the
    # target line reads cleanly as "0 -> step" against the noisy measured
    # equilibrium offset. The blue (measured) trace is untouched.
    ref_ideal = np.where(t < 0, 0.0, float(step))

    fig, ax = plt.subplots(figsize=FIG_SIZE)
    ax.plot(t, dev, color=FILT_COLOR, linewidth=1.6, label="gap (filtered, %.0f Hz LP)" % CUTOFF_HZ)
    ax.plot(t, ref_ideal, color=REF_COLOR, linewidth=1, linestyle="--", label="ref (idealized target)")
    ax.axhline(0, color="gray", linewidth=0.8, linestyle=":")
    ax.axvline(0, color="gray", linewidth=0.8, linestyle=":")

    ax.set_xlim(*DEV_XLIM)
    ax.set_ylim(*DEV_YLIM)

    ax.set_xlabel("time [s]", fontsize=LABEL_FS)
    ax.set_ylabel("air gap (y - y_ref) [mm]", fontsize=LABEL_FS)
    ax.set_title("Step response: Kp=16, Kd=0.3, step = %s mm" % step, fontsize=TITLE_FS)
    ax.tick_params(axis="both", labelsize=TICK_FS)
    ax.legend(loc="best", fontsize=LEGEND_FS)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_normalized(step, t, gap_f):
    pre_level = gap_f[t < 0].mean()
    post_level = pre_level + settled_center(step, t, gap_f - pre_level)
    norm = (gap_f - pre_level) / (post_level - pre_level)

    fig, ax = plt.subplots(figsize=FIG_SIZE)
    ax.plot(t, norm, color=FILT_COLOR, linewidth=1.6, label="gap (normalized, filtered)")
    ax.axhline(0, color="gray", linewidth=0.8, linestyle=":")
    ax.axhline(1, color="gray", linewidth=0.8, linestyle=":")
    ax.axvline(0, color="gray", linewidth=0.8, linestyle=":")
    ax.set_xlim(*NORM_XLIM)
    ax.set_ylim(*NORM_YLIM)
    ax.set_xlabel("time [s]  (t=0 at step)")
    ax.set_ylabel("normalized gap  (0 = pre-step, 1 = settled)")
    ax.set_title("Normalized step response: Kp=16, Kd=0.3, step = %s mm" % step)
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_combined(results):
    fig, ax = plt.subplots(figsize=FIG_SIZE)
    for (step, t, dev), color in zip(results, COMBINED_COLORS):
        ax.plot(t, dev, color=color, linewidth=1.4, label="step = %s mm" % step)
    ax.axhline(0, color="gray", linewidth=0.8, linestyle=":")
    ax.axvline(0, color="gray", linewidth=0.8, linestyle=":")

    ax.set_xlim(*DEV_XLIM)
    ax.set_ylim(*DEV_YLIM)

    ax.set_xlabel("time [s]", fontsize=LABEL_FS)
    ax.set_ylabel("air gap (y - y_ref) [mm]", fontsize=LABEL_FS)
    ax.set_title("Step response comparison: Kp=16, Kd=0.3", fontsize=TITLE_FS)
    ax.tick_params(axis="both", labelsize=TICK_FS)
    ax.legend(loc="best", fontsize=LEGEND_FS)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def plot_steady_state_error(errors):
    steps = [float(s) for s, _ in errors]
    ss = [e for _, e in errors]

    fig, ax = plt.subplots(figsize=FIG_SIZE)
    ax.plot(steps, ss, "o-", color=FILT_COLOR, linewidth=1.6, markersize=7)
    ax.axhline(0, color="gray", linewidth=0.8, linestyle=":")

    ax.set_xlabel("step size [mm]", fontsize=LABEL_FS)
    ax.set_ylabel("steady-state error [mm]", fontsize=LABEL_FS)
    ax.set_title("Steady-state error vs step size: Kp=16, Kd=0.3", fontsize=TITLE_FS)
    ax.tick_params(axis="both", labelsize=TICK_FS)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def main():
    combined = []
    errors = []
    for step in STEP_SIZES:
        path = os.path.join(DATA_DIR, "data_step_%s.csv" % step)
        if not os.path.exists(path):
            print("missing %s, skipping" % path)
            continue

        t, gap, ref = load(path)
        gap_f = lowpass(t, gap)

        pre_level = gap_f[t < 0].mean()
        dev = gap_f - pre_level

        fig = plot_deviation(step, t, dev)
        out_path = os.path.join(DATA_DIR, "data_step_%s.png" % step)
        fig.savefig(out_path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print("saved %s" % out_path)

        fig = plot_normalized(step, t, gap_f)
        out_path = os.path.join(DATA_DIR, "data_step_%s_normalized.png" % step)
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        print("saved %s" % out_path)

        combined.append((step, t, dev))

        center = settled_center(step, t, dev)
        ss_error = center - float(step)
        errors.append((step, ss_error))
        print("step %s mm: settled center %.3f mm, steady-state error %.3f mm"
              % (step, center, ss_error))

    fig = plot_combined(combined)
    out_path = os.path.join(DATA_DIR, "step_comparison.png")
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("saved %s" % out_path)

    ss_plot_errors = [(s, e) for s, e in errors if s in SS_ERROR_STEPS]
    fig = plot_steady_state_error(ss_plot_errors)
    out_path = os.path.join(DATA_DIR, "steady_state_error.png")
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("saved %s" % out_path)


if __name__ == "__main__":
    main()
