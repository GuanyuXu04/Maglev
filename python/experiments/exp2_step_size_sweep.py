"""Experiment 2: step-size sweep at the demo critical-damping design point --
how large a reference step still stays inside the region where the
linearized model (README 1.2, 1.4) is a good description of the real
(nonlinear) plant?

Gains are held fixed at the committed demo design point (zeta=1,
omega_n=1.35*sqrt(b) -- also the .ino's default kP, kD): "the theoretically
critical-damped case" the assignment asks about. An LTI model's *normalized*
step response doesn't depend on step amplitude at all, so theory predicts a
perfectly flat Mp=0 and ts=3/(zeta*omega_n) regardless of step size. Step
size is the only independent variable in this experiment (contrast with
experiment 1, which held step size small and fixed and swept gains) -- any
growth in overshoot or deviation in settling time as the step grows here is
attributable to the plant's nonlinearity (F=K*i/y^2 deviating from its
linear approximation as y strays from y0), not to the L/filter effects
experiment 1 already isolated at small step.

Both signs of step are swept: moving the gap further from the coil (larger
y, weaker/more linear force-vs-gap sensitivity) vs closer to the coil
(smaller y, force blows up as 1/y^2 -- expected to break down faster).

Run:  cd python && PYTHONPATH=. python experiments/exp2_step_size_sweep.py
Outputs: results/exp2_step_size_sweep.csv, results/exp2_step_size.png
"""

from pathlib import Path
import csv

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from maglev_sim import params, linearize, plant, metrics
from maglev_sim.reference_controller import ControllerParams, PDController

RESULTS_DIR = Path(__file__).resolve().parents[1] / "results"

ZETA = 1.0
OMEGA_MULT = 1.35  # matches the .ino's committed demo gains (kP=1806.4, kD=39.0)
STEP_FRACTIONS = [0.01, 0.02, 0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80]
# ts_rel tolerance is looser than it looks like it should need to be: the
# closed loop here has a lightly-damped residual mode (see PARAMETERS.md,
# the L/tau-caused -15rad/s pole) that sits right at the edge of the +-5%
# settling band even deep in the linear regime, so the *discrete* crossing
# time is itself sensitive to step size by ~15-20% between neighboring small
# steps with no real nonlinearity involved (confirmed by inspecting the
# y(t) traces directly). 0.30 avoids flagging that metric-edge noise while
# still catching the genuine, much larger deviations found near the
# divergence boundary.
TS_REL_TOLERANCE = 0.30   # relative ts deviation from the small-step baseline
MP_ABS_TOLERANCE = 0.10   # absolute extra overshoot (fraction) beyond the small-step baseline


def _is_diverging(y: np.ndarray, y0: float, target: float, step: float) -> bool:
    """Physical failure (non-positive/NaN gap) or a growing-not-decaying
    oscillation -- see exp1_gain_sweep._is_diverging for the same idea;
    here the amplitude reference scale is the step itself (which varies
    across this sweep), not a fixed fraction of y0.
    """
    if not np.all(np.isfinite(y)):
        return True
    if np.any(y <= 0.1 * y0):
        return True  # gap collapsed toward/through the coil -- unphysical
    half = len(y) // 2
    ref_scale = max(abs(step), 0.01 * y0)
    first_amp = np.max(np.abs(y[:half] - target))
    second_amp = np.max(np.abs(y[half:] - target))
    return second_amp > 1.1 * first_amp and second_amp > 0.1 * ref_scale


def run_case(step_fraction: float) -> dict:
    op, plant_params, loop = params.OP, params.PLANT, params.LOOP
    omega_n = OMEGA_MULT * linearize.open_loop_pole()
    kP, kD = linearize.kp_kd_from_zeta_omega(ZETA, omega_n)
    Mp_formula, ts_formula = linearize.theoretical_response(ZETA, omega_n)

    step = step_fraction * op.y0
    target = op.y0 + step
    r_fn = lambda t: target
    decay_rate = ZETA * omega_n
    t_end = max(20.0 / decay_rate, 0.3)

    controller = PDController(ControllerParams.from_design(kP, kD))
    result = plant.simulate_closed_loop(controller, plant_params, op, r_fn, t_end, loop.dt)

    diverged = _is_diverging(result.y, op.y0, target, step)
    saturated = bool(np.any(np.abs(result.u) >= params.ACTUATOR.supply_voltage - 1e-6))

    if diverged:
        Mp_sim, ts_sim, y_final = float("nan"), float("nan"), float("nan")
    else:
        Mp_sim, ts_sim, y_final = metrics.step_response_metrics(result.t, result.y)

    return dict(
        step_fraction=step_fraction, step_mm=step * 1000, target_mm=target * 1000,
        kP=kP, kD=kD, Mp_formula=Mp_formula, ts_formula=ts_formula,
        Mp_sim=Mp_sim, ts_sim=ts_sim,
        y_final_mm=y_final * 1000 if not diverged else float("nan"),
        diverged=diverged, saturated=saturated,
    )


def _find_boundary(rows: list[dict], sign: int) -> float | None:
    """First |step_fraction| (with the given sign) at which the case
    diverges, or deviates from the *smallest-step-of-that-sign* baseline by
    more than the tolerances above.

    The baseline is empirical (the smallest step actually simulated),
    not the closed-form theory or even the idealized-linear-ODE prediction,
    because both of those already differ from this fixed-gain system's
    small-step behavior by amounts characterized separately in experiment 1
    (the README ts formula is itself only accurate for zeta<1, and this
    repo's committed tau/L aren't negligible either). Comparing against the
    smallest step's *own simulated* response cancels those constant offsets
    and isolates the thing this experiment actually varies: step size.
    """
    signed = sorted([r for r in rows if np.sign(r["step_fraction"]) == sign],
                     key=lambda r: abs(r["step_fraction"]))
    if not signed or signed[0]["diverged"]:
        return abs(signed[0]["step_fraction"]) if signed else None
    baseline_Mp, baseline_ts = signed[0]["Mp_sim"], signed[0]["ts_sim"]
    for r in signed[1:]:
        if r["diverged"]:
            return abs(r["step_fraction"])
        mp_bad = abs(r["Mp_sim"] - baseline_Mp) > MP_ABS_TOLERANCE
        ts_bad = abs(r["ts_sim"] / baseline_ts - 1.0) > TS_REL_TOLERANCE
        if mp_bad or ts_bad:
            return abs(r["step_fraction"])
    return None


def main() -> None:
    RESULTS_DIR.mkdir(exist_ok=True)
    fractions = sorted(set(STEP_FRACTIONS) | {-f for f in STEP_FRACTIONS})
    rows = [run_case(f) for f in fractions]

    csv_path = RESULTS_DIR / "exp2_step_size_sweep.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {csv_path} ({len(rows)} points)")

    boundary_pos = _find_boundary(rows, +1)
    boundary_neg = _find_boundary(rows, -1)
    print(f"\nSafe linear region (gains fixed at zeta={ZETA}, omega_n={OMEGA_MULT:.2f}x sqrt(b), "
          f"vs the smallest-step baseline: Mp within {MP_ABS_TOLERANCE:.2f} abs, "
          f"ts within {TS_REL_TOLERANCE*100:.0f}% rel):")
    print(f"  increasing-gap (r>y0) side breaks down at step >= "
          f"{boundary_pos*100:.1f}% of y0" if boundary_pos else "  increasing-gap side: stayed within tolerance for all steps tested")
    print(f"  decreasing-gap (r<y0) side breaks down at step >= "
          f"{boundary_neg*100:.1f}% of y0" if boundary_neg else "  decreasing-gap side: stayed within tolerance for all steps tested")

    diverged = [r for r in rows if r["diverged"]]
    if diverged:
        print(f"\n{len(diverged)} point(s) diverged outright (physical failure or growing oscillation):")
        for r in sorted(diverged, key=lambda r: r["step_fraction"]):
            print(f"  step={r['step_fraction']*100:+.0f}% of y0 (target={r['target_mm']:.3f}mm)")

    _plot(rows, boundary_pos, boundary_neg)


def _plot(rows: list[dict], boundary_pos: float | None, boundary_neg: float | None) -> None:
    rows = sorted(rows, key=lambda r: r["step_fraction"])
    fracs = np.array([r["step_fraction"] for r in rows]) * 100
    ok = [not r["diverged"] for r in rows]
    smallest_pos = min((r for r in rows if r["step_fraction"] > 0), key=lambda r: r["step_fraction"])
    smallest_neg = min((r for r in rows if r["step_fraction"] < 0), key=lambda r: abs(r["step_fraction"]))

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))

    axes[0].axhline(rows[0]["Mp_formula"], color="k", ls="--", lw=1.2, label="closed-form formula (Mp=0; known-approximate)")
    axes[0].axhline(smallest_pos["Mp_sim"], color="C1", ls=":", lw=1.2, label="small-step baseline (+side)")
    axes[0].axhline(smallest_neg["Mp_sim"], color="C2", ls=":", lw=1.2, label="small-step baseline (-side)")
    axes[0].plot(fracs[ok], [r["Mp_sim"] for r, keep in zip(rows, ok) if keep], "o-", color="C0", label="sim")
    if any(not k for k in ok):
        axes[0].plot(fracs[[not k for k in ok]], [0] * sum(not k for k in ok), "rx", ms=10, mew=2, label="diverged")
    axes[0].set_xlabel("step, % of y0 (negative = toward coil)")
    axes[0].set_ylabel("overshoot Mp (fraction)")
    axes[0].set_title("Overshoot vs step size")
    axes[0].legend(fontsize=6.5)

    axes[1].axhline(rows[0]["ts_formula"] * 1000, color="k", ls="--", lw=1.2, label="closed-form formula (known-approximate)")
    axes[1].axhline(smallest_pos["ts_sim"] * 1000, color="C1", ls=":", lw=1.2, label="small-step baseline (+side)")
    axes[1].axhline(smallest_neg["ts_sim"] * 1000, color="C2", ls=":", lw=1.2, label="small-step baseline (-side)")
    axes[1].plot(fracs[ok], [r["ts_sim"] * 1000 for r, keep in zip(rows, ok) if keep], "o-", color="C0", label="sim")
    axes[1].set_xlabel("step, % of y0 (negative = toward coil)")
    axes[1].set_ylabel("settling time ts (ms)")
    axes[1].set_title("Settling time vs step size")
    axes[1].legend(fontsize=6.5)

    for ax in axes:
        if boundary_pos:
            ax.axvline(boundary_pos * 100, color="red", ls="-.", lw=1, alpha=0.6)
        if boundary_neg:
            ax.axvline(-boundary_neg * 100, color="red", ls="-.", lw=1, alpha=0.6)

    fig.suptitle(f"Step-size sweep at zeta={ZETA}, omega_n={OMEGA_MULT:.2f}x sqrt(b) "
                 "(fixed 'critically damped' design)")
    fig.tight_layout()
    out = RESULTS_DIR / "exp2_step_size.png"
    fig.savefig(out, dpi=150)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
