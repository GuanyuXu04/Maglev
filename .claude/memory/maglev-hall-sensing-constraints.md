---
name: maglev-hall-sensing-constraints
description: "ME495 Maglev project — why Hall replaced the ToF sensor, and the two calibration curves plus their known failure modes"
metadata:
  type: project
---

ME495 Maglev, zero budget for new sensors.

The VL53L0X ToF front end was **abandoned for closed-loop control**: 20 ms integration + 25 ms period gave ~35–45 ms loop delay against an open-loop unstable pole p≈20–28 rad/s (τ_plant≈35–50 ms), i.e. **p·τ ≈ 0.9 — right at the fundamental stabilizability limit** for a RHP pole plus pure delay (Skogestad: need p·T < 1). Symptom was a relay limit cycle with PWM slamming 0↔255; lowering Kp changed nothing, which is the diagnostic that it is a bandwidth wall, not a tuning problem. This p·τ argument is the intended headline result for the poster/report even if levitation never stabilizes.

Replaced by one **SS49EUA analog Hall** sensor, read with the ATmega328P ADC at /16 prescaler (~13 µs/conversion, ~77 kSPS) → effective delay ~1 ms, p·τ ≈ 0.03. PWM moved to Timer1 at ~31 kHz so the actuator adds no lag.

Two calibration curves were fitted (raw data in `scripts/run_*_PM.csv` and `run_*_coil.csv`; both share null ≈ 2507–2510 mV so they superpose):
- `dB_coil(u) ≈ -1.03·u` mV — coil self-field at the Hall, essentially linear, −262 mV at u=255.
- `gap_mm = f(hall_pm)` cubic, valid only for hall_pm ∈ [2015, 2490] mV ↔ gap ∈ [22, 49] mm.

Air gap ground truth came from `air_gap = 5 + (ToF_max − ToF)` with ToF_max=418 (user-supplied datum: coil touching the magnet = 5 mm gap).

**Critical geometry fact:** magnet approaching AND coil current increasing both push the Hall reading DOWN. The coil field therefore mimics the magnet approaching, which is why the `dB_coil` subtraction is both mandatory and dangerously touchy — see [[maglev-coil-correction-positive-feedback]].

Measurement quality is limited by the magnet's lateral/tilt freedom, not by the sensors: unconstrained by hand gave ±90 mV (≈±21 mm) scatter; adding an axial rod (1-DOF constraint) collapsed it to ±4.7 mm, with the residual coming from the cardboard under the magnet tilting. Any further tightening must come from better mechanical constraint, not from filtering.

**Why:** ranked fallbacks matter under a tight schedule and no budget — Hall gap-inversion control (`controller_top.ino`), then raw-Hall-voltage regulation (`controller_hall.ino`, immune to both inversion bugs), then the quantified-failure writeup.
**How to apply:** never recommend re-adding ToF to the control loop; never propose buying a sensor; check any gap setpoint is inside [22, 49] mm before suggesting it.
