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
  nominal values below give `R/L ≈ 400 rad/s` vs `sqrt(b) ≈ 19.8 rad/s`, a
  ~20x margin — comfortable, but arbitrary until you check it against reality.

## Bucket B — design choices, not measurements

These aren't properties of the hardware; they're operating points and
sample-time choices you get to pick, subject to real constraints:

- **`y0` (equilibrium gap).** Pick a gap that (a) the VL53L0X can read
  reliably and (b) gives you enough clearance for a step without the magnet
  hitting the coil or drifting out of range. This repo picks **`y0 = 50mm`**
  — not the original, tighter first guess of 10mm — and the reason is now
  the *dominant* one, ahead of sensor noise: see "Why a 30Hz sensor cannot
  stabilize this plant" below. In short, `b = 2g/y0` sets the mechanical
  open-loop instability's own time constant, and at 10mm that instability
  is faster than *any* sensor rate achievable without expensive hardware —
  a hard sampling-rate requirement, not something gain tuning can fix. 50mm
  is the smallest gap (checked by direct discrete-stability simulation) at
  which a 60Hz sensor stabilizes this plant with real margin. A pleasant
  side effect: 50mm is also comfortably at/above the range where VL53L0X
  accuracy is typically rated good (datasheet accuracy degrades noticeably
  below ~30-50mm), whereas the original 10mm was deep in the noisy-reading
  region anyway. **Still empirically check your own sensor's noise at
  whatever gap you use** — this repo's conclusion is about the *sampling
  rate* limit, not a claim that 50mm is noise-free on your specific unit.
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

(This is the coil-only balance. This repo's plant also carries a
permanent-magnet-on-core term `K_pm/y^4`, so the committed `K` is derived from
the two-term balance `m*g = K*i0/y0^2 + K_pm/y0^4` instead — see
"Permanent-magnet core-magnetization force" above. The procedure below is
unchanged; only the algebra that turns the calibration into `K` gains the
`− K_pm/y0^4` term.)

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

## Permanent-magnet core-magnetization force (an effect beyond README §1.1)

README §1.1's plant keeps only the coil's `F = K*i/y^2` pull. A real
attractive-type rig has a second, always-upward force the README omits: the
levitated permanent magnet magnetizes the electromagnet's iron core, and is
then attracted to the magnetization it induced. This "suck-in" force points
up the whole time (it does not reverse with coil-current sign) and falls off
faster with distance than the coil term; this repo models it as

```
F_pm(y) = K_pm / y^4          (always upward, K_pm >= 0)
```

so the nonlinear plant (`maglev_sim/plant.py`) is now
`m*y_ddot = m*g - K*i/y^2 - K_pm/y^4`.

- **`K_pm` (bucket B/C — a design-choice magnitude, then folded into the
  calibration).** There is no first-principles value for `K_pm` any more than
  for `K`; on real hardware it would fall out of the same hover calibration.
  Here it is a placeholder chosen so the core term supplies **5% of the weight
  at the operating point**: `K_pm = 0.05 * m*g * y0^4 = 6.13125e-8 N·m⁴`.
- **Why `K` changed from 1.22625e-3 to 1.1649375e-3.** The hover force balance
  is now shared between the two upward terms,
  `m*g = K*i0/y0^2 + K_pm/y0^4`, so the coil's share is `m*g − K_pm/y0^4` and
  `K = (m*g − K_pm/y0^4)*y0^2/i0` (`params.K_from_equilibrium(..., K_pm)`).
  With the 5% split, `K` drops to 95% of its coil-only value. `(y0, i0, u0)`
  is therefore still an *exact* equilibrium of the full nonlinear plant —
  `run_console.py` still starts at rest at `y0`, and
  `check_equilibrium_consistency()` now checks this two-term balance. `MAG_K`
  in the `.ino` was updated to match (it is a plant constant the control law
  never reads), and a companion `MAG_K_PM` constant was added there for the
  same params-sync bookkeeping.
- **The controller was NOT redesigned for it.** The linearized design model
  (README §1.2 / `linearize.py`) still uses `b = 2g/y0` and the same demo
  `kP, kD`. Including `F_pm` would steepen the true open-loop instability to
  `b_true = 2(g + K_pm/(m*y0^4))/y0` (~5% larger here), so `F_pm` is a
  deliberately-unmodeled effect the existing controller must simply be robust
  to — exactly like the neglected electrical pole and derivative-filter lag.
  Its influence is confined to transients and off-`y0` excursions (the two
  upward terms have different `y`-profiles), not to the resting hover point.

## Ground and ceiling travel limits

The simulated plant (`maglev_sim/plant.py`) confines `y` to a physical rail:
a ground below and the electromagnet's face above, via
`apply_travel_limits()`/`params.TravelLimits`. This was added after an early
version of the real-time console let a diverging response reach `y` = tens
of meters — physically meaningless, since the real magnet would have hit
something within a few tens of centimeters.

- **`y_max = 0.450 m`** ("ground") — Bucket A: this is a measurement of
  *your* rig (how far the magnet can fall before hitting the table/base),
  not a derived quantity. Replace it with whatever your build actually
  measures.
- **`y_min = 0.0005 m`** ("ceiling", the electromagnet's face) — Bucket B, a
  small numerical/design floor, *not* a measurement. `F = K*i/y^2` is
  undefined at `y=0`; some small minimum gap is also physically realistic
  (a mechanical spacer or the magnet's own casing would prevent literal
  contact with the coil). 0.5mm is an arbitrary but reasonable placeholder —
  tighten or loosen it once you know your rig's actual minimum clearance.

Modeled as a simple inelastic stop: hitting a boundary clamps position and
zeros only the velocity component driving further into it, so the magnet
can immediately leave again if the net force reverses (e.g. the controller
pulls hard enough to overcome gravity right at that position) — see
`test_magnet_leaves_the_ground_once_it_can`.

**In practice, the ground is effectively one-way with this repo's
placeholder `K`**: lifting off from `y_max` needs
`i = m*g*y_max^2/K ≈ 32A` (`F ~ 1/y^2` makes the electromagnet ~81x weaker
at 450mm than at `y0=50mm`) — about 10.8x the ~3A `CURRENT_LIMIT_A` rating,
still well out of reach. This matches real intuition: an attractive-type
maglev that loses its object can't call it back from tens of centimeters
away; someone has to place it back near `y0` by hand. Don't read the
ground/ceiling as "a soft safety margin the controller can recover from" —
reaching either one with realistic actuator limits is, practically,
terminal for that run. (This margin is *less* extreme than it was at the
original `y0=10mm` placeholder, where it was ~270x/810A — `K` scales with
`y0^2` through the equilibrium calibration, so a larger `y0` also means a
proportionally stronger magnet/coil pairing was needed to hover there in
the first place.)

## The nominal placeholder numbers actually in the code

Computed to be internally consistent (equilibrium holds, `R/L >> sqrt(b)`,
actuator headroom is sane):

| Symbol | Value | Bucket |
|---|---|---|
| `g` | 9.81 m/s² | constant |
| `m` | 0.020 kg | A (placeholder — replace with your scale reading) |
| `y0` | 0.050 m | B — see "Why a 30Hz sensor cannot stabilize this plant" |
| `i0` | 0.400 A | B |
| `R` | 8.0 Ω | A (placeholder — replace with multimeter reading) |
| `L` | 0.020 H | A (placeholder — replace with LR test) |
| `K` | 1.1649375e-3 N·m²/A | C, derived: `(m*g − K_pm/y0^4)*y0^2/i0` |
| `K_pm` | 6.13125e-8 N·m⁴ | B/C, design: PM-on-core pull, 5% of hover force (see above) |
| `u0` | 3.2 V | derived: `i0*R` |
| `b` | 392.4 s⁻² (`sqrt(b) ≈ 19.81 rad/s`, 3.15 Hz) | derived: `2g/y0` |
| `c'` | 3.0656 | derived: `(g/i0)/R` |
| `dt` | 1 ms (1 kHz loop tick) | B, see the electrical-pole-aliasing finding below |
| `tau` | 10 ms | B |
| demo `kP, kD` | 361.28, 17.4465 (`zeta=1`, `omega_n=1.35*sqrt(b)≈26.7 rad/s`) | D |
| `y_max` ("ground") | 450 mm | A — measure your rig |
| `y_min` ("ceiling") | 0.5 mm | B — numerical/design floor |

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

**Update after moving `y0` to 50mm** (see "Why a 30Hz sensor cannot
stabilize this plant" below): the specific eigenvalues quoted above were
computed at the original `y0=10mm` demo gains (`kP=1806.4, kD=39.0`). The
current demo gains (`kP=361.28, kD=17.45`, gentler because `omega_n` scales
with the now-smaller `sqrt(b)`) couple much less strongly into the
electrical state — re-checked directly, the closed loop is now stable even
at `dt=5ms`, not just `dt=1ms`. `dt=1ms` is kept anyway: it costs the AVR
nothing and leaves comfortable margin rather than designing exactly to the
edge of what's needed.

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

**Correction, superseded by the finding below:** an earlier version of this
section suggested dropping to a lower `omega_n` multiplier as "the first,
cheapest thing to try" if your sensor is slow. That's wrong for a sensor as
slow as ~30Hz specifically — see "Why a 30Hz sensor cannot stabilize this
plant" below, which shows the problem at 30Hz isn't gain-dependent at all.
Experiment 1's sweep grid still correctly shows predicted-vs-actual match
degrading as `omega_n` climbs *at the 1kHz rate it's run at*; it just isn't
the relevant knob once your effective sample period is comparable to or
slower than the plant's own open-loop instability time constant.

## Why a 30Hz sensor cannot stabilize this plant (found by building the real-time console)

`python/run_console.py` (see its module docstring) runs the ported firmware
in real time against a *rate-limited* simulated sensor, to match what you'd
actually see with a VL53L0X wired up. The first time it was run with a
realistic ~30Hz sensor rate, a plain 2% step diverged explosively — not a
slow drift, `y` reaching tens of meters within a couple of seconds. Two
distinct problems were found chasing this down, in order:

1. **A real bug, now fixed**: `computeControl()`'s `dt` argument was being
   computed as time-since-the-last-1kHz-tick, not time-since-the-last-
   *actual call to computeControl()*. Those are the same when the sensor is
   always fresh (true throughout `python/maglev_sim` everywhere else in this
   repo, and in experiments 1-2, and in the pytest suite — which is why this
   was never caught before), but when the sensor is slower than the tick
   (the normal case, see above), they differ by 20-30x. The derivative
   filter's Tustin coefficients are only valid for the interval the
   `(y - y_prev)` difference actually spans; using the wrong (much smaller)
   `dt` made the filter overestimate velocity by roughly that same 20-30x
   (confirmed directly: a true 0.05 m/s signal was read back as
   0.16-0.43 m/s). Fixed in both `maglev_controller.ino`
   (`g_lastControlMicros`, tracked separately from the tick gate
   `g_lastTickMicros`) and `arduino_port.ArduinoFirmware.loop_tick()`
   (`_time_since_last_control`, accumulated across ticks with no new
   sample). `test_firmware_closed_loop_matches_plant_reference_controller`
   and the always-fresh-sensor test in
   `test_realtime_console_stable_at_fast_sensor_rate` guard this.

2. **A hard, non-negotiable limit, not a bug**: even after that fix, the
   demo gains still diverge at a realistic 30Hz sensor rate — and so does
   *every* gain tried, from the demo point down to essentially zero
   feedback. Direct discrete forward-simulation (the same method used to
   find the 1ms requirement above, re-run at `dt=33ms`) shows the closed
   loop growing by a factor of >800,000 over one second of simulated time,
   with growth still present at `omega_n` multipliers as low as 0.005.
   The reason: the plant's *open-loop* instability time constant is
   `1/sqrt(b) ≈ 22.6ms` **at the original `y0=10mm`** — already faster than
   one 33ms sample period. A controller that only finds out where the
   magnet is once every 33ms cannot out-run an instability that doubles
   roughly every 22.6ms; this is true regardless of how the feedback gain
   is tuned. Direct forward-simulation puts the actual stability boundary
   for `y0=10mm` at roughly `dt <= 3-5ms` (>=200-300Hz) — comfortably
   explaining why the committed 1ms/1kHz tick (chosen earlier for the
   *electrical*-pole reason above) also happened to be fast enough for
   *this* reason too, at that `y0`.

**What this means for real hardware:** if your measured `y0` and coil `R,L`
come out anywhere near the *original* 10mm placeholder, a VL53L0X-class
~30Hz sensor cannot close this loop by itself, full stop — not "needs
better tuning." Three real fixes, in rough order of how much they change
the design (and see "Resolution" immediately below for which one this repo
actually took, and the numbers behind it):
- **Increase `y0`.** `b = 2g/y0` shrinks as `y0` grows, slowing the
  open-loop instability — this is the fix this repo uses.
- **Cascaded control**: a fast inner loop regulating coil current (via the
  LMD18200's current-sense pin, sampled fast, independent of the slow
  position sensor) with a slow outer position loop running at the VL53L0X's
  native rate. Standard fix for exactly this class of problem; not
  implemented in this repo's single-loop design (see below for why it turned
  out not to be the more practical choice here).
- **A faster position sensor** (analog Hall-effect, laser triangulation)
  with sub-millisecond latency, replacing the VL53L0X.

### Resolution: `y0` moved to 50mm, sensor rate to 60Hz

The first estimate above ("`y0` on the order of tens of cm") was a rough
scaling argument, made without actually running the numbers — and it was
wrong enough to be misleading. Redone properly (same direct discrete
forward-simulation method, this time scanning `y0` at a fixed sensor `dt`
instead of scanning `dt` at a fixed `y0`), with a 60Hz sensor rate (chosen
as "the fastest a VL53L0X-class sensor can plausibly go without moving to
more expensive hardware," per an explicit project constraint):

| `y0` | single-loop, 60Hz sensor | cascaded control (idealized), 60Hz sensor |
|---|---|---|
| 10mm (original) | unstable | unstable |
| 30mm | unstable | stable |
| 40mm | stable, but **zero margin** (50Hz already fails catastrophically) | stable |
| **50mm** | **stable, margin down to ~45Hz** | stable with more margin |

Two things fell out of actually computing this instead of guessing:

1. **Cascaded control does not fix this by itself.** Even with a *perfect,
   instantaneous* inner current loop (removing the electrical dynamics from
   consideration entirely), the *mechanical* open-loop instability alone —
   which depends only on `y0`, not on `R`/`L`/control architecture — is
   still too fast for a 60Hz sensor below `y0≈15-20mm`. Cascading buys a
   smaller minimum gap (~16mm vs ~30-40mm) but was not necessary once a
   modest `y0` increase was on the table anyway, so this repo did not add
   the extra firmware complexity of an inner current loop.
2. **The naive threshold has zero safety margin.** `y0=40mm` is technically
   stable at exactly 60Hz, but 50Hz *already* fails catastrophically there
   (no gradual degradation) — too fragile against a real sensor's actual
   achieved rate wandering a bit below its nominal spec. `y0=50mm` stays
   stable down to ~45Hz, a real margin, for a small additional gap increase.

`y0=50mm` was therefore adopted as the new default throughout this repo
(`params.py`, the `.ino`, `arduino_port.py`'s demo gains, `run_console.py`'s
default `--sensor-hz 60`), with `K` and `kP, kD` recomputed from it via the
same bucket-C/D procedures above — nothing about *how* those values are
derived changed, only the `y0` input to that derivation.

**Try it yourself**: `python run_console.py` (default: `y0=50mm`, 60Hz
sensor, stable) vs. `python run_console.py --sensor-hz 30` (reproduces the
original finding interactively — still unstable at 30Hz even at the new
`y0`) vs. `--sensor-hz 1000` (idealized fast sensor, matching experiments
1-2's assumption).

`tau = 10ms` is a compromise rather than a clean design: the "don't distort
the closed loop" guideline wants the filter corner (`1/tau`) at least
5-10x above `omega_n`, and `1/tau=100rad/s` vs the current
`omega_n≈26.7rad/s` (~3.75x) still falls a bit short of that, though it's a
notably better margin than the ~1.7x this ratio had at the original
`y0=10mm`/`omega_n≈60rad/s` design point, simply as a side effect of
`omega_n` scaling down with the smaller `sqrt(b)` at `y0=50mm`. Treat `tau`
as a knob to increase first if the D-term looks noisy on hardware, and
re-check for added lag/overshoot if you do.
