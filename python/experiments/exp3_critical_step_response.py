"""Critical-damping step-amplitude experiment.

The gains are computed once from the ideal linearized plant at zeta=1.  The
nonlinear simulation then sweeps positive step amplitudes in millimetres.
It scans 15-25 mm in uniform 0.05 mm increments.  Three figures are produced:

1. measured settling time versus step amplitude, with the exact ideal-linear
   settling time as a dashed baseline and unstable cases marked by crosses;
2. representative trajectories at 17, 21, and 25 mm;
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
STEP_AMPLITUDES_MM = np.round(np.arange(15.0, 25.0 + 1e-9, 0.05), 2)
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
    """Run the requested uniform 15-25 mm sweep at 0.05 mm spacing."""
    cases = [run_case(float(step)) for step in STEP_AMPLITUDES_MM]
    boundary = _first_unstable(cases).step_mm
    return cases, boundary


def choose_representatives(cases: list[Case]) -> tuple[Case, Case, Case]:
    by_step = {round(case.step_mm, 2): case for case in cases}
    requested = (17.0, 21.0, 25.0)
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

    smooth_x = np.linspace(float(stable_x[0]), float(stable_x[-1]), 800)
    smooth_y = PchipInterpolator(stable_x, stable_y)(smooth_x)

    fig, ax = plt.subplots(figsize=(8.2, 4.8))
    ax.plot(smooth_x, smooth_y, color="tab:blue", lw=2.0,
            label="Nonlinear simulation")
    ax.axhline(1000.0 * ideal_ts, color="black", ls="--", lw=1.6,
               label=f"Ideal reduced-order linear plant: {1000.0 * ideal_ts:.1f} ms")
    ax.axvspan(boundary, 25.0, color="red", alpha=0.10,
               label="Unstable: no finite settling time")
    ax.axvline(boundary, color="red", ls=":", lw=1.2)
    ax.annotate(
        f"first unstable case: {boundary:.2f} mm",
        xy=(boundary, 198.0), xytext=(boundary + 0.12, 198.0),
        fontsize=9, color="darkred", ha="left", va="top",
    )

    ax.set_xlabel("Step input amplitude (mm)")
    ax.set_ylabel("5% settling time about final value (ms)")
    ax.set_title("Critical-damping gains: settling time versus step amplitude")
    ax.grid(True, alpha=0.25)
    ax.set_xlim(15.0, 25.0)
    ax.set_ylim(160.0, 200.0)
    ax.legend(loc="lower left", fontsize=8.5)
    fig.tight_layout()

    out = RESULTS_DIR / "exp3_critical_step_settling.png"
    fig.savefig(out, dpi=180)
    plt.close(fig)
    return out


def plot_representatives(representatives: tuple[Case, Case, Case], ideal_ts: float) -> Path:
    labels = ["Stable response", "Nonlinear response", "Divergent response"]
    fig, axes = plt.subplots(1, 3, figsize=(14.0, 4.3))

    for ax, case, label in zip(axes, representatives, labels):
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

        ax.plot(1000.0 * case.t[:end], 1000.0 * case.r[:end], "r--", lw=1.7,
                label="Reference")
        ax.plot(1000.0 * case.t[:end], 1000.0 * case.y[:end], color="tab:blue",
                lw=1.8, label="Actual position")
        ax.axvline(0.0, color="0.5", ls=":", lw=1.0)
        ax.set_xlabel("Time from step (ms)")
        ax.set_ylabel("Gap y (mm)")
        if case.unstable:
            subtitle = f"{label}\nstep={case.step_mm:.2f} mm; {case.failure_reason}"
        else:
            subtitle = (
                f"{label}\nstep={case.step_mm:.2f} mm, "
                f"ts={1000.0 * case.settling_time_s:.0f} ms "
                f"(ideal {1000.0 * ideal_ts:.0f} ms)"
            )
        ax.set_title(subtitle, fontsize=10)
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=8, loc="best")

    fig.suptitle("Representative responses using linear-model critical-damping gains")
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

    fig, ax = plt.subplots(figsize=(8.2, 4.8))
    ax.plot(all_steps, ideal_error, "k--", lw=1.7,
            label="Ideal linear plant prediction")
    ax.plot([case.step_mm for case in stable], nonlinear_error,
            color="tab:blue", lw=1.8, label="Nonlinear simulation")
    ax.axvspan(boundary, 25.0, color="red", alpha=0.10,
               label="Unstable: no steady state")
    ax.axvline(boundary, color="red", ls=":", lw=1.2)
    ax.set_xlabel("Step input amplitude (mm)")
    ax.set_ylabel("Steady-state error magnitude |y_ss - r| (mm)")
    ax.set_title("Steady-state tracking error versus step amplitude")
    ax.set_xlim(15.0, 25.0)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper left", fontsize=8.5)
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
