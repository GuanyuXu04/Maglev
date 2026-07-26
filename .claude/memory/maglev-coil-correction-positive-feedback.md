---
name: maglev-coil-correction-positive-feedback
description: The dB_coil subtraction in controller_top.ino forms a >1-gain positive feedback loop during transients — root cause of the 0↔255 PWM chatter
metadata:
  type: project
---

In `arduino/controller_top/controller_top.ino`, `hall_pm = hall_op - dB_coil(g_duty)` applies the coil-field correction **algebraically and instantaneously**, but the coil current lags by its electrical time constant L/R. During a duty step the correction is already full-size while the real field has not moved, so the gap estimate is inflated → PD reads "gap too big" → raises duty further. Single-step loop gain:

    G = Kp · 1.03 · d(gap)/d(hall)

With Kp≈16 and the cubic map's slope, G ≈ 1.01 at gap 30 mm, 1.57 at 33 mm, 2.30 at 40 mm. G>1 with one-step delay gives alternating-sample divergence — which matches the observed log exactly (gap oscillating 25.7↔33.5 mm, pwm alternating 0↔255).

Fix is to low-pass the duty fed into `dB_coil()` with τ ≈ L/R so the correction's dynamics match the physical field, not to reduce Kp.

A second, independent defect in the same signal path: the fitted cubic `gapFromHall` has a spurious near-zero-slope plateau at hall≈2163 mV (gap≈25 mm, ~0.004 mm/mV vs 0.18 mm/mV at 2490) — 45× sensitivity variation. This is a polynomial artifact and is backwards physically (small gap should be MORE sensitive), so the controller goes nearly blind near 25 mm. Replace with a monotone model or a lookup table.

**Why:** these two are the direct causes of the instability; they were diagnosed analytically from the fit coefficients and confirmed against logged data, so they should not be re-derived from scratch.
**How to apply:** when levitation misbehaves in the gap-inversion controller, suspect these before touching Kp/Kd/BIAS. See [[maglev-hall-sensing-constraints]] for the calibration numbers.
