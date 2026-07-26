---
name: maglev-data-collection-pipeline
description: Maglev rig hardware (Arduino + Hall sensor + coil PWM), the serial-command step-response data collection scripts, and the filter/recenter/steady-state-error processing pipeline built for their output
metadata:
  type: project
---

Hardware: an Arduino (ATmega328P, running `arduino/controller_top/controller_top.ino`) closes the control loop locally — reads a Hall-effect sensor on an analog input for gap sensing, drives the electromagnet coil via a PWM output (Timer1, ~31 kHz so the actuator adds negligible lag), and runs the PD control law onboard. See [[maglev-hall-sensing-constraints]] for sensor calibration/accuracy limits and [[maglev-coil-correction-positive-feedback]] for the coil-correction feedback hazard.

Serial command interface (115200 baud): `KP <val>` / `KD <val>` set gains and echo `Kp=...`/`Kd=...`; `R <mm>` sets the reference absolutely and echoes `R=<mm>` — that echo is used as the t=0 marker for step captures. The bare `s` command is a legacy hard-coded relative +0.20 mm step, unused by the current collection scripts. Telemetry streams continuously as `gap:<mm>,ref:<mm>,pwm:<val>` lines at roughly 50 Hz.

Data collection scripts (`scripts/collect_step.py`, `scripts/sweep_kp_kd.py`, and the script behind `scripts/sweep_step_kp16_kd0.3/`) all follow the same pattern: send `R`/`KP`/`KD`, watch a short settle window and ask for a live stability go/no-go, then record a fixed number of pre-step rows, fire the reference step, record a fixed number of post-step rows, and save one CSV per trial with columns `t_s` (seconds relative to the `R=` echo), `phase` (`pre`/`post`), `gap_mm`, `ref_mm`, `pwm`. A `summary.csv` alongside the per-trial files records which (Kp, Kd) or step-size combinations were stable/skipped/unstable — a raw per-trial CSV only exists for combinations that passed the live check.

Post-processing pipeline (`scripts/process_data/plot_step_sweep.py` and `plot_kp_kd_sweep.py`), arrived at iteratively:
- Raw `gap_mm` is noisy; apply a zero-phase Butterworth low-pass (`scipy.signal.filtfilt`, order 3) so no phase lag is introduced at the step marker. The cutoff needed to go lower than expected to look genuinely settled rather than jittery — ended at 1 Hz (started at 4 Hz).
- Each trace is recentered so its own pre-step average is treated as the equilibrium (0 mm deviation), because the raw Hall reading carries a calibration offset from the nominal commanded reference (e.g. pre-step readings sit around 27.5 mm even though the commanded reference is 30 mm) — see [[maglev-hall-sensing-constraints]].
- "Steady-state error" = mean of the filtered deviation over a bounded tail window, minus the commanded step size. The window must be bounded well before the end of the recording (e.g. t ≤ 15 s of a ~20 s post-step recording); some runs show real amplitude growth/drift late in the recording that biases a full-recording tail average upward.

Two raw-data failure modes turned up only by looking at plotted results, not by assuming filtering would handle everything — worth checking for in any future sweep from this rig:
- **Loss of levitation**: `pwm` pins at 0 and `gap_mm` pins at the Hall sensor's ~45 mm reading ceiling. This is real instability, not noise — exclude it from steady-state/settling analysis rather than averaging it in.
- **Stuck sensor reading**: the Hall channel occasionally holds one exact value for many (>10) consecutive samples, most dangerously during the pre-step window, silently dragging the equilibrium/baseline estimate off by a few mm. Detected by checking for long runs of bit-identical consecutive `gap_mm` values.

**Why:** each of these was a concrete, non-obvious problem discovered by looking at plotted results, not derivable just from reading the scripts — future analysis of this rig's data should reuse this pipeline rather than re-deriving it from scratch.
**How to apply:** when processing new sweeps from this rig, apply filter → recenter → bounded steady-state window → check for the two failure modes, in that order, before trusting any steady-state number. Also treat single-trial (no-repeat) gain-sweep comparisons as noisy: only broad regional patterns across a heatmap/sweep are trustworthy, not fine-grained cell-to-cell ranking.
