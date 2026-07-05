# Where every number in this repo comes from

## Erratum found during simulation-based verification: a sign in README §1.3

README §1.3 writes the control law as `u(t) = kP*(r-y) - kD*y_dot_filtered`.
Implemented literally, **this is unconditionally unstable** for this plant,
not just fragile — closing the loop against the nonlinear simulation with
this exact formula saturates the actuator and drives the gap through zero
(the magnet "crashes" through the coil) within a few sample periods, for
any step size including tiny ones.

Reason: §1.2 derives `P(s) = -c'/(s^2-b)` — negative, because more coil
current *pulls the magnet closer* (shrinks the gap), an unavoidable
consequence of `F = K*i/y^2` in §1.1. Substituting the literal §1.3 law into
the linearized ODE gives a closed-loop characteristic equation of
`s^2 - c'*kD*s - (c'*kP+b) = 0`, which has no stable pole placement for any
kP, kD > 0 (Routh-Hurwitz needs both non-leading coefficients positive; here
both are forced negative). Flipping the sign of the *entire* PD output —

```
u(t) = -kP*(r(t) - y(t)) + kD*y_dot_filtered(t)
```

— reproduces §1.4's own stated characteristic equation
`s^2 + c'*kD*s + (c'*kP-b) = 0` exactly, which makes the stability
condition and the `omega_n`/`zeta`/`Mp`/`ts` formulas in §1.4 internally
consistent with the plant model in §1.1-1.2. This was confirmed three
independent ways: substituting directly into the linearized ODE by hand,
block-diagram algebra on `1 - P(s)C(s) = 0`, and symbolically with `sympy`
(all three agree); the physical reading also matches — if the gap is too
large, the fix is to *increase* current, not decrease it.

This is what `python/maglev_sim/reference_controller.py` and the `.ino`
implement (both carry the same derivation in a comment). **README.md itself
is left untouched** since it may be shared/assignment material — this note
exists so the discrepancy isn't silently papered over. If you believe §1.3
is correct as written and something else here is wrong, the place to check
first is the actuator/H-bridge current-direction convention: the fix above
is also equivalent to keeping §1.3's literal formula but wiring the
LMD18200's DIR sign so that a *positive* PD output corresponds to
*decreasing* coil current — i.e. the sign has to live somewhere, either in
the control law or in the actuator convention, and this repo puts it in the
control law since README §1.1 already fixes the actuator/force sign via
`F = K*i/y^2`.

## Where every other number in this repo comes from

This project needs numeric values for `m, g, K, R, L, y0, i0, tau, dt, kP, kD`
before anything can be simulated or run on hardware. The honest situation is:
**none of these are known yet for the real rig**, and most of them cannot be
guessed — they have to be measured or derived from one calibration
experiment. This document is the single place that explains, parameter by
parameter, which bucket it falls into and exactly how to obtain it. Every
number that appears in [`python/maglev_sim/params.py`](python/maglev_sim/params.py)
and in the constants block of
[`arduino/maglev_controller/maglev_controller.ino`](arduino/maglev_controller/maglev_controller.ino)
is a placeholder drawn from this document, never an independent guess.

There are four buckets:

| Bucket | Parameters | How you get the number |
|---|---|---|
| A. Direct measurement | `m`, `R`, `L` | Scale, multimeter, LR step-response test |
| B. Design choice (yours to pick, within hardware limits) | `y0`, `i0`, `dt`, `tau`, supply voltage | Pick, don't measure — but pick inside real constraints |
| C. Derived from A+B via one calibration run | `K` | Algebra from a single hovering measurement, **not** a separate force rig |
| D. Derived from A+B+C via the closed-loop formulas in the README | `kP`, `kD` | Plug `zeta`, `omega_n` into §1.4 of the README — this is literally what Experiment 1 sweeps |

The nominal numbers actually committed in the code are a **self-consistent
fictional data sheet** — they satisfy the equilibrium equation and the
"electrical pole is much faster than mechanical" assumption, so the
simulation and the theory formulas agree with each other, and so the sweep
experiments below produce sane-looking plots. They are placeholders only.
**Do not wire up hardware and expect these numbers to mean anything until
you've replaced them following the recipe below.**

## Bucket A — measure these directly, no theory needed

- **`m` (levitated mass, kg).** Put the permanent magnet (plus whatever
  bracket/vane holds it) on a kitchen or lab scale. There is nothing to
  derive here.
- **`R` (coil DC resistance, Ω).** Multimeter, leads on the coil, power off.
- **`L` (coil inductance, H).** Either an LCR meter, or if you don't have
  one: drive the coil (magnet removed, or clamped away from the coil so
  there's no motion) through a known series resistor from a step voltage,
  and scope the current. The current rises as `i(t) = (V/Rtotal)(1 -
  exp(-t/tau_e))` with `tau_e = L/Rtotal`; read `tau_e` off the scope (time
  to reach 63% of final value) and solve `L = tau_e * Rtotal`.

  **Why this matters beyond just having a number:** the README's linearized
  plant (§1.2) throws away the coil's electrical dynamics on the argument
  that `R/L` (the electrical pole) is much faster than `sqrt(b)` (the
  mechanical instability rate). You must check `R/L >> sqrt(b)` with your
  *measured* R and L once you have them — if it's not true, the
  second-order model in §1.4 (and everything this repo computes from it) is
  invalid and you'd need to control the third-order plant instead. The
  nominal values below give `R/L ≈ 400 rad/s` vs `sqrt(b) ≈ 44 rad/s`, a
  ~9x margin — comfortable, but arbitrary until you check it against reality.

## Bucket B — design choices, not measurements

These aren't properties of the hardware; they're operating points and
sample-time choices you get to pick, subject to real constraints:

- **`y0` (equilibrium gap).** Pick a gap that (a) the VL53L0X can read
  reliably and (b) gives you enough clearance for a step without the magnet
  hitting the coil or drifting out of range. **Caveat worth flagging early:**
  the VL53L0X's datasheet-rated accuracy degrades noticeably below ~30-50mm,
  even though it will return *some* number down to near 0mm; a lot of small
  maglev builds run gaps of 5-15mm anyway and just live with more sensor
  noise at that range. We nominally pick `y0 = 10mm`, but **you should
  empirically check the VL53L0X's noise/repeatability at your intended gap
  before trusting this choice** — if it's too noisy, increasing `y0`
  directly reduces `b = 2g/y0` (an easier, slower open-loop instability) at
  the cost of a floppier-looking demo.
- **`i0` (equilibrium current).** Also a choice, constrained by the
  LMD18200's 3A continuous rating and your supply voltage. We pick `i0 =
  0.4A`, deliberately well under the 3A limit, leaving headroom for the ΔI
  the controller commands during a step.
- **Supply voltage.** We assume a 12V bench supply. Combined with `R = 8Ω`
  this caps current at 1.5A even in full saturation — i.e. the voltage
  headroom, not the LMD18200's current limit, is what would clip control
  effort here. That's a deliberate, safer failure mode than relying on the
  driver's current limiting.
- **`dt` (control loop period) and `tau` (derivative filter time constant).**
  See "Sample-rate reality check" below — these interact with the VL53L0X's
  achievable update rate and can't be chosen independently of it.

## Bucket C — `K`, the one you cannot guess (and don't need a force rig for)

`K` (the lumped `F = K*i/y^2` magnetic constant) depends on the coil's turns,
core material and geometry. It is *not* something you can look up or
reasonably estimate a priori, and building a separate force-vs-current rig
to measure it directly is more work than necessary. Instead, use the
equilibrium condition itself as the calibration:

```
m*g = K*i0/y0^2   =>   K = m*g*y0^2 / i0
```

Procedure once you have `m` (bucket A) and have picked `y0` (bucket B):

1. Start with deliberately conservative, hand-picked gains (small `kP`, `kD
   > 0`) — you don't need to know `K` to get a first stabilizing controller,
   you just need *some* positive `c'*kP > b`. In practice people bring the
   magnet up manually near `y0` while slowly increasing `kP` from zero until
   it catches and holds, exactly like tuning any unknown-plant PD loop by
   hand.
2. Once it hovers stably at `y0`, read off the steady-state coil current
   `i0` (either a current-sense reading from the LMD18200's current-sense
   pin, or infer it from the commanded steady PWM duty and supply voltage
   via `i0 ≈ u0/R`, since at DC `L*di/dt = 0`).
3. Compute `K = m*g*y0^2/i0`.
4. Recompute `b, c, c'` from the real `m, R, y0, i0, K`, and redesign `kP,
   kD` from the target `zeta, omega_n` using the §1.4 formulas (inverted —
   see `linearize.kp_kd_from_zeta_omega` in the Python package) instead of
   the hand-picked bootstrap gains.

This is why the architecture keeps a hard separation between "bootstrap
gains used only to get a first hover" and "the swept `kP, kD` used for the
actual experiments" — the calibration in step 1-3 is expected to use
something crude, and only after it produces `K` do the theoretical formulas
become trustworthy enough to design against.

## Bucket D — `kP`, `kD`

Once `b, c'` are known (from A+B+C), any target `(zeta, omega_n)` maps to
`(kP, kD)` via the closed-loop characteristic equation in README §1.4:

```
kP = (omega_n^2 + b) / c'
kD = 2*zeta*omega_n / c'
```

This is exactly what Experiment 1 sweeps (over a grid of `zeta, omega_n`,
reported alongside the resulting `kP, kD`), and what Experiment 2 holds
fixed at `zeta = 1` while sweeping step size.

## The nominal placeholder numbers actually in the code

Computed to be internally consistent (equilibrium holds, `R/L >> sqrt(b)`,
actuator headroom is sane):

| Symbol | Value | Bucket |
|---|---|---|
| `g` | 9.81 m/s² | constant |
| `m` | 0.020 kg | A (placeholder — replace with your scale reading) |
| `y0` | 0.010 m | B |
| `i0` | 0.400 A | B |
| `R` | 8.0 Ω | A (placeholder — replace with multimeter reading) |
| `L` | 0.020 H | A (placeholder — replace with LR test) |
| `K` | 4.905e-5 N·m²/A | C, derived: `m*g*y0^2/i0` |
| `u0` | 3.2 V | derived: `i0*R` |
| `b` | 1962 s⁻² (`sqrt(b) ≈ 44.3 rad/s`, 7.05 Hz) | derived: `2g/y0` |
| `c'` | 3.0656 | derived: `(g/i0)/R` |
| `dt` | 1 ms (1 kHz loop tick) | B, forced fast by the electrical-pole-aliasing finding below |
| `tau` | 10 ms | B |
| demo `kP, kD` | 1806.4, 39.0 (`zeta=1`, `omega_n=1.35*sqrt(b)≈59.8 rad/s`) | D |

`python/maglev_sim/params.py` is the single source of truth for these
numbers; `linearize.py` derives `b, c, c'` from them programmatically (never
hand-rounded), and the Arduino constants block is checked against
`params.py` by `python/tests/test_maglev_sim.py` so the two can't silently
drift apart.

## A second finding: electrical-pole aliasing forced `dt` down to 1ms

The first `dt` chosen for this repo was 5ms (200Hz), picked to be "fast
compared to the sensor." Closing the loop in `python/maglev_sim` at that `dt`
with the demo gains produced a **growing oscillation** in the simulated step
response — not noise, not a plotting artifact. Chasing it down (see the
git-free scratch analysis behind this note, reproducible by dropping `dt`
back to 0.005 in `params.py` and rerunning `pytest`) turned up a real
discrete-control effect, independently confirmed three ways (direct
forward-simulation of the exact discrete recursion, a hand-derived discrete
state-space model, and the continuous-time 4th-order eigenvalues below all
agreeing once the arithmetic was untangled):

- The *continuous-time* closed loop (plant + coil electrical dynamics +
  derivative filter, all as designed) is stable but only lightly damped: one
  pole pair sits at `-15 ± 109j` (`zeta≈0.14`, ringing at ~17Hz) alongside two
  well-damped real poles at `-27` and `-443`. Nothing pathological.
- But `-443 rad/s` is a 14ms period, and at `dt=5ms` that pole is sampled
  barely twice per cycle — nowhere near enough. The *discretized* closed loop
  built from the exact recursion the Arduino's `computeControl()` uses
  genuinely has an eigenvalue outside the unit circle at `dt=5ms` (confirmed
  by direct iteration: response amplitude visibly grows call-to-call, not
  just "looks noisy"). At `dt=1ms` the same recursion is comfortably stable
  and matches the continuous prediction.

**Takeaway kept in the code:** `dt = 1ms` (1kHz `loop()` tick) is now the
committed value, specifically because it's fast enough to resolve the coil's
electrical pole given the placeholder `R, L`. This is unrelated to (and
doesn't fix) the separate sensor-rate limitation below — it's about how often
`computeControl()` itself needs to run, which costs the AVR essentially
nothing (see the compile output: <20% flash, <20% RAM), independent of how
often a *new* sensor sample is available.

**Why this matters for you, not just for this repo's placeholders:** if your
measured `R, L` give a much faster or much slower electrical pole than the
nominal 2.5ms, redo this check (bump `dt` down in `params.py`, rerun
`pytest`, watch for growing-amplitude step responses) before trusting any
gains computed from the §1.4 formulas — the formulas themselves don't know
about the sampling rate at all, and a design that's fine in continuous time
can still be unstable once discretized too coarsely.

## Sample-rate reality check (the sensor is a separate, harder limit)

The VL53L0X's default continuous-ranging timing budget is ~33ms (~30Hz), and
its documented minimum via `setMeasurementTimingBudget()` is ~20ms (~50Hz) —
faster than that trades away accuracy/range. That's the real ceiling on how
often you get a *new* position sample, regardless of how fast `loop()` spins
(1kHz here, per the finding above).

This is a materially harder constraint than the electrical-pole issue: a
20-33ms effective feedback update is *coarser* than the unstable 5ms case
just diagnosed, not finer. Two things follow, both worth confirming on real
hardware before trusting the demo gains:

1. The Arduino sketch's sensor stub is modeled as "poll and get a
   *new-data-ready* flag," not "always returns a fresh value" — matching how
   the real VL53L0X continuous-ranging API actually behaves. `loop()`
   re-outputs the last computed `u` on ticks where no new sample arrived,
   and only recomputes the filtered derivative/error when a new sample
   lands, using the *actual measured* inter-sample time (not an assumed
   constant), since the sensor's cadence isn't perfectly periodic. But if
   the *effective* feedback rate ends up being the sensor's ~30-50Hz rather
   than the 1kHz tick, the electrical-pole-aliasing check above needs
   re-running at *that* dt, not 1ms — and given 20-33ms is worse than the
   5ms that already failed, it may well fail again with these placeholder
   R, L.
2. The standard real-hardware fix for this class of problem, if it
   materializes with your measured R, L, is a **cascaded control
   structure**: a fast inner loop regulating coil current directly (using
   the LMD18200's current-sense pin, sampled fast, independent of the slow
   position sensor) tracking a setpoint from a slow outer position loop
   running at the VL53L0X's native rate. That decouples "must resolve the
   electrical pole" from "limited by the position sensor." This repo
   implements the single-loop (position PD directly commanding coil voltage)
   design from README 1.3 as specified; adding a cascaded current loop would
   be a natural next step if bench testing shows the electrical dynamics are
   a real problem at your sensor's achieved rate.

If your real sensor rate is on the slow end (~30Hz) and a cascaded loop is
out of scope, dropping to a lower `omega_n` multiplier (e.g. `1.0x sqrt(b)`
instead of `1.35x`) is the first, cheapest thing to try — Experiment 1's
sweep grid deliberately includes this lower-bandwidth region so you can see
how predicted-vs-actual match degrades as `omega_n` climbs relative to
whatever sample rate you actually have.

`tau = 2*dt` (with the new `dt=1ms`, so `tau=10ms`) is a compromise rather
than a clean design: the "don't distort the closed loop" guideline wants the
filter corner (`1/tau`) at least 5-10x above `omega_n`, and `10ms` vs
`omega_n≈60rad/s` (`1/tau=100rad/s`, only ~1.7x) falls short of that.
Treat `tau` as a knob to increase first if the D-term looks noisy on
hardware, and re-check for added lag/overshoot if you do.
