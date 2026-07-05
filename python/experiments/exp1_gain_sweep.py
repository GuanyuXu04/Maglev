"""Experiment 1: Kp/Kd sweep -- overshoot and settling time of a small step
response, README 1.4 theory vs the exact idealized linear ODE vs the actual
nonlinear simulation (which is what the .ino's algorithm would produce).

Swept as (zeta, omega_n) pairs (more physically interpretable than a raw
Kp/Kd grid) and converted to Kp, Kd via linearize.kp_kd_from_zeta_omega --
every point is still, concretely, a (Kp, Kd) pair; they're just chosen this
way rather than off an arbitrary grid.

Run:  cd python && PYTHONPATH=. python experiments/exp1_gain_sweep.py
Outputs: results/exp1_gain_sweep.csv, results/exp1_overshoot.png,
         results/exp1_settling_time.png
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

ZETAS = [0.3, 0.5, 0.7, 1.0, 1.3, 1.6, 2.0]
OMEGA_MULTS = [0.75, 1.0, 1.35, 1.75, 2.25]  # x sqrt(b)
STEP_FRACTION = 0.02  # of y0 -- "small step" per README's scope; see exp2
                       # for how large a step actually stays linear.
DIVERGE_THRESHOLD_FRAC = 0.5  # of y0: excursion beyond this counts as "left
                               # the safe linear neighborhood," matching the
                               # bound exp2 uses to define linear validity.


def _decay_rate(zeta: float, omega_n: float) -> float:
    if zeta <= 1.0:
        return zeta * omega_n
    return zeta * omega_n - omega_n * np.sqrt(zeta ** 2 - 1)


def _is_diverging(y: np.ndarray, y0: float, threshold_frac: float) -> bool:
    """True if the trace ever leaves the safe neighborhood, OR if oscillation
    amplitude is growing (not decaying) between the first and second half of
    the window. The amplitude-growth check matters because some gain
    combinations here are genuinely, slowly unstable once the neglected coil
    inductance is included (linear theory says any kP, kD > 0 with kD > 0 and
    c'*kP>b is stable -- that's only true for the *reduced* 2nd-order model);
    a short/nominal-decay-rate window can end before slow growth becomes
    large enough to trip a simple excursion threshold. See PARAMETERS.md.
    """
    if not np.all(np.isfinite(y)):
        return True
    if np.max(np.abs(y - y0)) > threshold_frac * y0:
        return True
    half = len(y) // 2
    first_amp = np.max(np.abs(y[:half] - y0))
    second_amp = np.max(np.abs(y[half:] - y0))
    return second_amp > 1.1 * first_amp and second_amp > 0.02 * y0


def run_case(zeta: float, mult: float) -> dict:
    op, plant_params, loop = params.OP, params.PLANT, params.LOOP
    omega_n = mult * linearize.open_loop_pole()
    kP, kD = linearize.kp_kd_from_zeta_omega(zeta, omega_n)
    Mp_formula, ts_formula = linearize.theoretical_response(zeta, omega_n)

    step = STEP_FRACTION * op.y0
    r_fn = lambda t: op.y0 + step
    # A generous, fixed-floor window: slowly-growing instabilities need time
    # to reveal their trend, which the nominal (possibly-wrong) decay rate
    # can't be trusted to predict.
    t_end = max(30.0 / _decay_rate(zeta, omega_n), 0.5)

    controller = PDController(ControllerParams.from_design(kP, kD))
    result = plant.simulate_closed_loop(controller, plant_params, op, r_fn, t_end, loop.dt)

    finite = bool(np.all(np.isfinite(result.y)))
    excursion = float(np.max(np.abs(result.y - op.y0))) if finite else float("inf")
    diverged = _is_diverging(result.y, op.y0, DIVERGE_THRESHOLD_FRAC)

    if diverged:
        Mp_sim, ts_sim, y_final = float("nan"), float("nan"), float("nan")
    else:
        Mp_sim, ts_sim, y_final = metrics.step_response_metrics(result.t, result.y)

    t_id, y_id = linearize.simulate_ideal_linear_response(zeta, omega_n, step, t_end, loop.dt)
    Mp_ideal, ts_ideal, _ = metrics.step_response_metrics(t_id, y_id)

    return dict(
        zeta=zeta, mult=mult, omega_n=omega_n, kP=kP, kD=kD,
        Mp_formula=Mp_formula, ts_formula=ts_formula,
        Mp_ideal=Mp_ideal, ts_ideal=ts_ideal,
        Mp_sim=Mp_sim, ts_sim=ts_sim,
        y_final_mm=y_final * 1000 if not diverged else float("nan"),
        excursion_mm=excursion * 1000 if finite else float("inf"),
        diverged=diverged,
    )


def main() -> None:
    RESULTS_DIR.mkdir(exist_ok=True)
    rows = [run_case(z, m) for z in ZETAS for m in OMEGA_MULTS]

    csv_path = RESULTS_DIR / "exp1_gain_sweep.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {csv_path}  ({len(rows)} points)")

    diverged_rows = [r for r in rows if r["diverged"]]
    if diverged_rows:
        print(f"\n{len(diverged_rows)} point(s) DIVERGED (excursion beyond "
              f"{DIVERGE_THRESHOLD_FRAC*100:.0f}% of y0) even at a "
              f"{STEP_FRACTION*100:.0f}%-of-y0 step -- linear theory says c'*kP>b, "
              "kD>0 is always stable, but the real system (with the neglected "
              "coil inductance/filter lag) is not, at these gains:")
        for r in diverged_rows:
            print(f"  zeta={r['zeta']:.2f}, omega_n={r['mult']:.2f}x sqrt(b) "
                  f"(kP={r['kP']:.1f}, kD={r['kD']:.2f}): excursion={r['excursion_mm']:.2f}mm")

    _plot(rows)


def _plot(rows: list[dict]) -> None:
    colors = plt.get_cmap("viridis")(np.linspace(0.15, 0.9, len(OMEGA_MULTS)))

    fig, ax = plt.subplots(figsize=(6.5, 4.5))
    zetas_ref = np.array(ZETAS)
    ax.plot(zetas_ref, [linearize.theoretical_overshoot(z) for z in zetas_ref],
            "k--", lw=1.5, label="README formula (function of zeta only)")
    for mult, color in zip(OMEGA_MULTS, colors):
        subset = [r for r in rows if r["mult"] == mult]
        ok = [r for r in subset if not r["diverged"]]
        bad = [r for r in subset if r["diverged"]]
        ax.plot([r["zeta"] for r in ok], [r["Mp_sim"] for r in ok], "o-",
                color=color, label=f"sim, omega_n={mult:.2f}x sqrt(b)")
        if bad:
            ax.plot([r["zeta"] for r in bad], [0] * len(bad), "rx", ms=10, mew=2,
                    label="_nolegend_")
    ax.set_xlabel("design zeta")
    ax.set_ylabel("overshoot Mp (fraction)")
    ax.set_title(f"Overshoot vs zeta ({STEP_FRACTION*100:.0f}%-of-y0 step); "
                 "red X = diverged, no valid Mp")
    ax.legend(fontsize=7, loc="upper right")
    fig.tight_layout()
    fig.savefig(RESULTS_DIR / "exp1_overshoot.png", dpi=150)

    fig, ax = plt.subplots(figsize=(6.5, 4.5))
    y_floor = min(r["ts_formula"] for r in rows) * 1000 * 0.7
    for mult, color in zip(OMEGA_MULTS, colors):
        subset = [r for r in rows if r["mult"] == mult]
        ok = [r for r in subset if not r["diverged"]]
        bad = [r for r in subset if r["diverged"]]
        ax.plot([r["zeta"] for r in subset], [r["ts_formula"] * 1000 for r in subset],
                "--", color=color, alpha=0.5, label=f"formula, omega_n={mult:.2f}x")
        ax.plot([r["zeta"] for r in ok], [r["ts_sim"] * 1000 for r in ok], "o-",
                color=color, label=f"sim, omega_n={mult:.2f}x")
        if bad:
            ax.plot([r["zeta"] for r in bad], [y_floor] * len(bad), "rx", ms=10, mew=2,
                    label="_nolegend_")
    ax.set_xlabel("design zeta")
    ax.set_ylabel("settling time ts (ms)")
    ax.set_yscale("log")
    ax.set_title(f"Settling time vs zeta ({STEP_FRACTION*100:.0f}%-of-y0 step); "
                 "red X = diverged, no valid ts")
    ax.legend(fontsize=6, loc="upper right", ncol=2)
    fig.tight_layout()
    fig.savefig(RESULTS_DIR / "exp1_settling_time.png", dpi=150)

    print(f"wrote {RESULTS_DIR / 'exp1_overshoot.png'}")
    print(f"wrote {RESULTS_DIR / 'exp1_settling_time.png'}")


if __name__ == "__main__":
    main()
