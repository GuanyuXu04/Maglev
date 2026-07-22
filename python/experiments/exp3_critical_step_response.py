"""Critical-damping step-amplitude experiment.

The gains are computed once from the ideal linearized plant at zeta=1.  The
nonlinear simulation then sweeps positive step amplitudes in millimetres.
It scans 10-22 mm in uniform 0.05 mm increments.  Three figures are produced:

1. measured settling time versus step amplitude, with the exact ideal-linear
   settling time as a dashed baseline and unstable cases shown as a shaded region;
2. representative normalized trajectories at 12, 19, and 22 mm;
3. nonlinear and ideal-linear steady-state tracking errors.

Run:  cd python && PYTHONPATH=. python experiments/exp3_critical_step_response.py
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import PchipInterpolator

from maglev_sim import linearize, metrics, params, plant
from maglev_sim.reference_controller import ControllerParams, PDController


RESULTS_DIR = Path(__file__).resolve().parents[1] / "results"

ZETA = 1.0
OMEGA_MULT = 1.35
STEP_AMPLITUDES_MM = np.round(np.arange(10.0, 22.0 + 1e-9, 0.05), 2)
PLOT_STEP_MIN_MM = 14.0
PLOT_STEP_MAX_MM = 22.0
REPRESENTATIVE_STEPS_MM = (12.0, 19.0, 22.0)
REPRESENTATIVE_Y_LIM = (-0.15, 2.8)
TITLE_FONTSIZE = 18
LABEL_FONTSIZE = 16
TICK_FONTSIZE = 15
LEGEND_FONTSIZE = 14
STEP_TIME_S = 0.05
POST_STEP_DURATION_S = 2.0
SETTLING_BAND = 0.05


@dataclass
class Case:
    step_mm: float
    t: np.ndarray
    y: np.ndarray
    r: np.ndarray
    u: np.ndarray
    settling_time_s: float
    overshoot: float
    final_y_mm: float
    saturated: bool
    unstable: bool
    failure_reason: str


def design() -> tuple[float, float, float]:
    """Return omega_n, kP, and kD from the ideal linearized model."""
    omega_n = OMEGA_MULT * linearize.open_loop_pole()
    kP, kD = linearize.kp_kd_from_zeta_omega(ZETA, omega_n)
    return omega_n, kP, kD


def ideal_settling_time() -> float:
    """Exact continuous-time 5% settling time for a critical step response."""
    omega_n, _, _ = design()
    # For zeta=1, the normalized residual is
    # q(t) = (1 + omega_n*t) * exp(-omega_n*t).  Solve q(ts)=0.05.
    x = 5.0
    for _ in range(20):
        exp_neg_x = math.exp(-x)
        residual = (1.0 + x) * exp_neg_x - SETTLING_BAND
        derivative = -x * exp_neg_x
        x -= residual / derivative
    return x / omega_n


def ideal_steady_state_error_mm(step_mm: float) -> float:
    """Magnitude of ideal-linear steady-state tracking error in millimetres."""
    coeffs = linearize.linear_coeffs()
    _, kP, _ = design()
    dc_gain = coeffs.c_prime * kP / (coeffs.c_prime * kP - coeffs.b)
    return abs(dc_gain - 1.0) * step_mm


def _style_axes(ax: plt.Axes) -> None:
    ax.tick_params(axis="both", labelsize=TICK_FONTSIZE)


def _failure_reason(y: np.ndarray, target: float, step: float) -> str:
    if not np.all(np.isfinite(y)):
        return "non-finite state"

    rail_tol = 1e-9
    if np.any(y <= params.LIMITS.y_min + rail_tol):
        return "upper mechanical stop"
    if np.any(y >= params.LIMITS.y_max - rail_tol):
        return "fell to lower mechanical stop"

    # A non-decaying oscillation may be visible before a rail is reached.
    half = len(y) // 2
    scale = max(abs(step), 0.001)
    amp_first = float(np.max(np.abs(y[:half] - target)))
    amp_second = float(np.max(np.abs(y[half:] - target)))
    if amp_second > 1.10 * amp_first and amp_second > 0.10 * scale:
        return "growing oscillation"
    return ""


def run_case(step_mm: float) -> Case:
    op, plant_params, loop = params.OP, params.PLANT, params.LOOP
    _, kP, kD = design()
    step = step_mm / 1000.0
    target = op.y0 + step

    def reference(t: float) -> float:
        return op.y0 if t < STEP_TIME_S else target

    controller = PDController(ControllerParams.from_design(kP, kD))
    result = plant.simulate_closed_loop(
        controller,
        plant_params,
        op,
        reference,
        STEP_TIME_S + POST_STEP_DURATION_S,
        loop.dt,
    )

    post = result.t >= STEP_TIME_S
    post_t = result.t[post] - STEP_TIME_S
    post_y = result.y[post]
    reason = _failure_reason(post_y, target, step)
    unstable = bool(reason)
    saturated = bool(
        np.any(np.abs(result.u[post]) >= params.ACTUATOR.supply_voltage - 1e-6)
    )

    if unstable:
        Mp = ts = final_y = float("nan")
    else:
        Mp, ts, final_y = metrics.step_response_metrics(
            post_t, post_y, settle_band=SETTLING_BAND
        )

    return Case(
        step_mm=float(step_mm),
        t=result.t - STEP_TIME_S,
        y=result.y,
        r=result.r,
        u=result.u,
        settling_time_s=float(ts),
        overshoot=float(Mp),
        final_y_mm=float(final_y * 1000.0),
        saturated=saturated,
        unstable=unstable,
        failure_reason=reason,
    )


def _first_unstable(cases: list[Case]) -> Case:
    failed = [case for case in cases if case.unstable]
    if not failed:
        raise RuntimeError("No unstable amplitude found in the coarse sweep")
    return min(failed, key=lambda case: case.step_mm)


def sweep() -> tuple[list[Case], float]:
    """Run the requested uniform sweep at 0.05 mm spacing."""
    cases = [run_case(float(step)) for step in STEP_AMPLITUDES_MM]
    boundary = _first_unstable(cases).step_mm
    return cases, boundary


def choose_representatives(cases: list[Case]) -> tuple[Case, Case, Case]:
    by_step = {round(case.step_mm, 2): case for case in cases}
    requested = REPRESENTATIVE_STEPS_MM
    missing = [step for step in requested if step not in by_step]
    if missing:
        raise RuntimeError(f"Representative amplitudes missing from sweep: {missing}")
    return tuple(by_step[step] for step in requested)


def write_csv(cases: list[Case], ideal_ts: float) -> Path:
    out = RESULTS_DIR / "exp3_critical_step_sweep.csv"
    with out.open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "step_mm", "kP", "kD", "ideal_settling_time_s",
                "measured_settling_time_s", "settling_error_pct", "overshoot",
                "final_y_mm", "ideal_steady_state_error_mm",
                "nonlinear_steady_state_error_mm", "saturated", "unstable", "failure_reason",
            ],
        )
        writer.writeheader()
        _, kP, kD = design()
        for case in cases:
            error = (
                100.0 * (case.settling_time_s / ideal_ts - 1.0)
                if not case.unstable else float("nan")
            )
            nonlinear_sse = (
                abs(case.final_y_mm - (params.OP.y0 * 1000.0 + case.step_mm))
                if not case.unstable else float("nan")
            )
            writer.writerow(
                {
                    "step_mm": case.step_mm,
                    "kP": kP,
                    "kD": kD,
                    "ideal_settling_time_s": ideal_ts,
                    "measured_settling_time_s": case.settling_time_s,
                    "settling_error_pct": error,
                    "overshoot": case.overshoot,
                    "final_y_mm": case.final_y_mm,
                    "saturated": case.saturated,
                    "ideal_steady_state_error_mm": ideal_steady_state_error_mm(case.step_mm),
                    "nonlinear_steady_state_error_mm": nonlinear_sse,
                    "unstable": case.unstable,
                    "failure_reason": case.failure_reason,
                }
            )
    return out


def plot_settling(cases: list[Case], ideal_ts: float, boundary: float) -> Path:
    stable = [case for case in cases if not case.unstable]
    stable_x = np.array([case.step_mm for case in stable])
    stable_y = 1000.0 * np.array([case.settling_time_s for case in stable])
    baseline_ms = 1000.0 * ideal_ts

    smooth_x = np.linspace(float(stable_x[0]), float(stable_x[-1]), 800)
    smooth_y = PchipInterpolator(stable_x, stable_y)(smooth_x)

    fig, ax = plt.subplots(figsize=(8.0, 6.0))
    ax.plot(smooth_x, smooth_y, color="tab:blue", lw=2.0,
            label="Nonlinear simulation")
    ax.axhline(baseline_ms, color="black", ls="--", lw=1.6,
               label=f"Ideal reduced-order linear plant: {baseline_ms:.1f} ms")
    if boundary < PLOT_STEP_MAX_MM:
        ax.axvspan(boundary, PLOT_STEP_MAX_MM, color="red", alpha=0.10,
                   label="Unstable: no finite settling time")
        ax.axvline(boundary, color="red", ls=":", lw=1.2)

    ax.set_xlabel("Step input amplitude (mm)", fontsize=LABEL_FONTSIZE)
    ax.set_ylabel("5% settling time about final value (ms)", fontsize=LABEL_FONTSIZE)
    ax.set_title("settling time versus step amplitude", fontsize=TITLE_FONTSIZE)
    ax.grid(True, alpha=0.25)
    ax.set_xlim(PLOT_STEP_MIN_MM, PLOT_STEP_MAX_MM)
    ax.set_ylim(160.0, 210.0)
    _style_axes(ax)
    ax.legend(loc="lower left", fontsize=LEGEND_FONTSIZE)
    fig.tight_layout()

    out = RESULTS_DIR / "exp3_critical_step_settling.png"
    fig.savefig(out, dpi=180)
    plt.close(fig)
    return out


def plot_representatives(representatives: tuple[Case, Case, Case], ideal_ts: float) -> Path:
    labels = [r"$\Delta r = 12$ mm", r"$\Delta r = 19$ mm", r"$\Delta r = 22$ mm"]
    colors = ["tab:blue", "tab:orange", "tab:green"]
    fig, ax = plt.subplots(figsize=(8.0, 6.0))

    max_end_time_ms = 0.0
    for case, label, color in zip(representatives, labels, colors):
        # Stop a failed trace shortly after contact with a mechanical rail so
        # the plot shows the loss of levitation without a long flat tail.
        end = len(case.t)
        if case.unstable:
            hit = np.flatnonzero(
                (case.y <= params.LIMITS.y_min + 1e-9)
                | (case.y >= params.LIMITS.y_max - 1e-9)
            )
            if len(hit):
                end = min(len(case.t), int(hit[0]) + 60)

        step = case.step_mm / 1000.0
        y_norm = (case.y[:end] - params.OP.y0) / step
        time_ms = 1000.0 * case.t[:end]
        max_end_time_ms = max(max_end_time_ms, float(time_ms[-1]))
        ax.plot(time_ms, y_norm, color=color, lw=1.9, label=label)

    ax.plot([-50.0, 0.0, 0.0, max_end_time_ms], [0.0, 0.0, 1.0, 1.0],
            "r--", lw=1.7, label="Normalized reference")
    ax.axvline(0.0, color="0.5", ls=":", lw=1.0)
    ax.set_xlabel("Time (ms)", fontsize=LABEL_FONTSIZE)
    ax.set_ylabel("Normalized gap change (y-y0)/step", fontsize=LABEL_FONTSIZE)
    ax.set_xlim(0.0, 1000.0)
    ax.set_ylim(*REPRESENTATIVE_Y_LIM)
    ax.set_title("Representative normalized step responses", fontsize=TITLE_FONTSIZE)
    ax.grid(True, alpha=0.25)
    _style_axes(ax)
    ax.legend(fontsize=LEGEND_FONTSIZE, loc="best")

    fig.tight_layout()
    out = RESULTS_DIR / "exp3_representative_responses.png"
    fig.savefig(out, dpi=180)
    plt.close(fig)
    return out


def plot_steady_state_error(cases: list[Case], boundary: float) -> Path:
    stable = [case for case in cases if not case.unstable]
    all_steps = np.array([case.step_mm for case in cases])
    ideal_error = np.array([
        ideal_steady_state_error_mm(case.step_mm) for case in cases
    ])
    nonlinear_error = np.array([
        abs(case.final_y_mm - (params.OP.y0 * 1000.0 + case.step_mm))
        for case in stable
    ])

    fig, ax = plt.subplots(figsize=(8.0, 6.0))
    ax.plot(all_steps, ideal_error, "k--", lw=1.7,
            label="Ideal linear plant prediction")
    ax.plot([case.step_mm for case in stable], nonlinear_error,
            color="tab:blue", lw=1.8, label="Nonlinear simulation")
    if boundary < PLOT_STEP_MAX_MM:
        ax.axvspan(boundary, PLOT_STEP_MAX_MM, color="red", alpha=0.10,
                   label="Unstable: no steady state")
        ax.axvline(boundary, color="red", ls=":", lw=1.2)
    ax.set_xlabel("Step input amplitude (mm)", fontsize=LABEL_FONTSIZE)
    ax.set_ylabel("Steady-state error magnitude |y_ss - r| (mm)", fontsize=LABEL_FONTSIZE)
    ax.set_title("Steady-state tracking error versus step amplitude", fontsize=TITLE_FONTSIZE)
    ax.set_xlim(PLOT_STEP_MIN_MM, PLOT_STEP_MAX_MM)
    ax.grid(True, alpha=0.25)
    _style_axes(ax)
    ax.legend(loc="upper left", fontsize=LEGEND_FONTSIZE)
    fig.tight_layout()

    out = RESULTS_DIR / "exp3_steady_state_error.png"
    fig.savefig(out, dpi=180)
    plt.close(fig)
    return out


def main() -> None:
    RESULTS_DIR.mkdir(exist_ok=True)
    omega_n, kP, kD = design()
    ideal_ts = ideal_settling_time()
    cases, boundary = sweep()
    representatives = choose_representatives(cases)

    csv_path = write_csv(cases, ideal_ts)
    settling_path = plot_settling(cases, ideal_ts, boundary)
    response_path = plot_representatives(representatives, ideal_ts)
    steady_error_path = plot_steady_state_error(cases, boundary)

    stable = [case for case in cases if not case.unstable]
    errors = [100.0 * (case.settling_time_s / ideal_ts - 1.0) for case in stable]
    print(f"critical design: omega_n={omega_n:.6g} rad/s, kP={kP:.6g}, kD={kD:.6g}")
    print(f"ideal linear 5% settling time: {ideal_ts * 1000.0:.1f} ms")
    print(f"refined first unstable amplitude: {boundary:.2f} mm")
    print(f"stable settling-time error range: {min(errors):+.1f}% to {max(errors):+.1f}%")
    print("representatives: " + ", ".join(
        f"{case.step_mm:.2f} mm" for case in representatives
    ))
    print(f"wrote {csv_path}")
    print(f"wrote {settling_path}")
    print(f"wrote {response_path}")
    print(f"wrote {steady_error_path}")


if __name__ == "__main__":
    main()
