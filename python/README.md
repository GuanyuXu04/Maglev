# `maglev_sim` -- Python simulation & verification

Simulates the **nonlinear** maglev plant from README.md section 1.1
(`F=K*i/y^2`, no linearization) and closes the loop with a Python port of
the exact discrete control algorithm the Arduino sketch implements, so the
firmware's algorithm can be verified and tuned before -- or without --
touching real hardware. See `../PARAMETERS.md` for where every physical
constant comes from; nothing here is arbitrary.

## Setup

```bash
conda env create -f environment.yml   # or: pip install -r requirements.txt
conda activate maglev
```

## Layout

- `maglev_sim/params.py` -- single source of truth for every physical
  constant and design choice (mass, R, L, y0, i0, dt, tau, actuator limits).
  The Arduino `.ino`'s constants block hardcodes the same numbers; a test
  checks they can't silently drift apart.
- `maglev_sim/plant.py` -- the nonlinear ODE (`F=K*i/y^2`, full electrical
  dynamics) with an RK4 integrator, sub-stepped for accuracy against the
  coil's fast electrical time constant.
- `maglev_sim/linearize.py` -- README 1.2/1.4's linearized-model formulas
  (`b`, `c'`, `kP`/`kD` <-> `zeta`/`omega_n` conversions, `Mp`/`ts`
  closed-form formulas), plus `simulate_ideal_linear_response()`, an exact
  simulation of the idealized 2nd-order closed loop -- a more rigorous
  ground truth than the closed-form formulas, which are themselves only
  approximations for `zeta>=1` (see PARAMETERS.md).
- `maglev_sim/reference_controller.py` -- a line-for-line Python mirror of
  the `.ino`'s `computeControl()`: same Tustin-discretized derivative
  filter, same (sign-corrected, see PARAMETERS.md) PD law, same
  feedforward and saturation. This is the "software-in-the-loop" (SIL) half
  of verification -- fast, no hardware needed, exercises the *algorithm*.
- `maglev_sim/hil_serial.py` -- the "hardware-in-the-loop" (HIL) half: talks
  to a **real** Arduino running the actual compiled `.ino` over serial,
  using its `SIM`/`Y` commands to inject privileged, exact position samples
  in place of the real VL53L0X, and integrates the nonlinear plant using the
  board's own returned control command. Requires a board; see the module
  docstring for usage.
- `maglev_sim/metrics.py` -- overshoot/settling-time extraction from a
  step-response trace, using README 1.4's convention (relative to the
  response's own final value, not the reference).
- `experiments/exp1_gain_sweep.py` -- **Experiment 1**: sweeps
  `(zeta, omega_n)` -> `(kP, kD)` over a grid, applies a small, fixed step,
  and compares simulated overshoot/settling time against the closed-form
  formulas. Flags gain combinations that are dynamically unstable once the
  (linear-theory-neglected) coil inductance is included, even though the
  reduced-order theory calls them trivially stable.
- `experiments/exp2_step_size_sweep.py` -- **Experiment 2**: holds gains
  fixed at the "critically damped" demo design point and sweeps step size
  (both directions -- toward and away from the coil) to find how far you
  can step before the response measurably departs from its own
  small-step behavior, and where it fails outright (actuator saturation /
  runaway).

Both experiments write a CSV and PNG plot(s) to `results/` (gitignored).

## Running

```bash
cd python
PYTHONPATH=. python experiments/exp1_gain_sweep.py
PYTHONPATH=. python experiments/exp2_step_size_sweep.py
PYTHONPATH=. pytest        # or just `pytest` if pytest.ini's pythonpath=. is picked up
```

(`PYTHONPATH=.` is only needed because this package isn't `pip install -e`'d;
pytest works without it since `pytest.ini` sets `pythonpath = .`.)

## Verification philosophy

Two independent things are checked, deliberately kept separate:

1. **Does the discrete algorithm implement the intended theory?**
   (`tests/test_maglev_sim.py`'s
   `test_reference_controller_matches_ideal_linear_model_in_the_fast_filter_limit`)
   -- checked against `linearize.simulate_ideal_linear_response`, in a
   regime where the two things real theory neglects (coil inductance,
   filter lag) are made deliberately negligible, isolating "is the code
   right" from "is the idealized theory a good description of the real,
   deliberately-non-ideal design point."
2. **Does the real (committed, non-ideal) design point behave sanely?**
   (`test_closed_loop_stays_bounded_and_qualitatively_correct`) -- loose
   bounds at the actual placeholder `tau`/`L`, documenting the expected
   (and, per Experiment 1, sometimes large) gap from idealized theory
   rather than asserting a tight match that wouldn't hold.
