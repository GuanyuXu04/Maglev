# Project Memory: Maglev Simulation

This repository is for an ME4950J fixed-point magnetic levitation controller.
The simulation work lives under `python/` and exists to verify the controller
logic before using the physical rig.

## What The Simulator Is For

- The plant is an attractive-type magnetic levitation system: an electromagnet
  pulls a permanent magnet upward while gravity pulls it downward.
- The controller target is one fixed operating point, not large-range tracking.
  Reference steps should stay small, usually a few millimeters, so the local
  linear model and actuator headroom remain meaningful.
- The Python simulator is the software-in-the-loop reference for the firmware:
  it runs the same PD control algorithm against a nonlinear plant model.

## Main Files

- `python/maglev_sim/params.py` is the simulation parameter source of truth.
  Check `PARAMETERS.md` before changing constants.
- `python/maglev_sim/plant.py` is the nonlinear plant model. Its state is
  `[y, y_dot, i]`, where `y` is the coil-to-magnet gap in meters. It includes
  coil force, gravity, coil electrical dynamics, a permanent-magnet/core
  attraction term, and travel limits for ground/ceiling stops.
- `python/maglev_sim/linearize.py` contains the reduced linear model and the
  gain/design formulas used for PD tuning.
- `python/maglev_sim/reference_controller.py` mirrors the controller math.
- `python/maglev_sim/arduino_port.py` structurally ports the Arduino firmware
  loop, serial commands, and sensor/actuator seams for real-time simulation.
- `python/run_console.py` is the interactive real-time simulation entry point.
- `python/maglev_sim/hil_serial.py` is for hardware-in-the-loop testing with a
  real Arduino running the firmware while Python integrates the plant.
- `python/tests/test_maglev_sim.py` documents and regression-tests the intended
  simulation behavior.

## Model And Control Notes

- Open loop is unstable around the hover point, so do not command open-loop
  voltage/current steps as if the plant were self-stabilizing.
- The controller is derivative-on-measurement PD with a filtered derivative and
  equilibrium feedforward voltage `u0`.
- The implemented control-law sign is the sign-corrected version described in
  `PARAMETERS.md` and the firmware comments. Do not "fix" it to the literal
  older README equation without re-deriving the closed-loop dynamics.
- The reduced linear model is useful for designing `kP` and `kD`, but the
  nonlinear simulation is the practical truth check because it includes effects
  the linear model intentionally neglects.
- The 1 kHz control-loop tick is deliberate. Earlier slower-loop assumptions
  did not resolve the coil electrical dynamics well enough.
- A realistic roughly 30 Hz time-of-flight sensor rate is not enough for this
  design. The current simulation story assumes a larger 50 mm operating gap so
  a roughly 60 Hz sensor update rate has usable stability margin.

## Current Caveat

The simulation parameters and the Arduino firmware constants may not represent
the same physical calibration state. Firmware comments mention newer measured
force/gain assumptions, while `python/maglev_sim/params.py` still carries the
simulation plant model used by the tests and experiments. If reconciling them,
update the plant/linearization model and documentation together; do not simply
copy constants from one side to the other.

## How To Run

From a fresh clone:

```bash
cd python
python -m pip install -r requirements.txt
python run_console.py
python -m pytest
```

Optional experiments:

```bash
cd python
python experiments/exp1_gain_sweep.py
python experiments/exp2_step_size_sweep.py
python experiments/exp3_critical_step_response.py
```

Experiment outputs go to `python/results/`, which is gitignored. Some processed
hardware sweep plots and CSVs also live under `scripts/`.

## Good Teammate Habits

- Read `PARAMETERS.md` before touching numerical constants or gains.
- Keep units explicit: simulation uses SI units internally; serial commands
  often use millimeters for human-facing setpoints.
- Re-run `python -m pytest` after simulation or firmware-control changes.
- Treat failed parameter-consistency tests as a signal to inspect the model
  state, not as permission to blindly synchronize numbers.
- Keep machine-specific paths, board ports, local environment names, and private
  calibration files out of committed docs.
