# ME4950J Maglev Fixed-Point PD Control

This is a mechanical engineering lab project (ME4950J). The hardware is an
**attractive-type magnetic levitation rig**: a permanent magnet is suspended
below an electromagnet whose coil current is driven bidirectionally by an
H-bridge (LMD18200). Position feedback comes from an laser time-of-flight
sensor (VL53L0X).

The control block diagram is the standard unity-feedback loop:

```
r(t) --> [+ sum -] --> e(t) --> C(s) --> x(t) --> P(s) --> y(t)
              ^                                              |
              |------------------- H(s) <---------------------
                                 (sensor)
```

- `r(t)`: reference/setpoint height (commanded gap).
- `e(t) = r - y`: error.
- `C(s)`: the controller you are implementing (PD, see §1.3).
- `x(t)`: control output = PWM duty cycle / coil voltage command.
- `P(s)`: the physical plant (electromagnet + permanent magnet + gravity).
- `y(t)`: measured position (gap between electromagnet and magnet).
- `H(s)`: sensor; after calibration, treated as unit gain, `H(s) = 1`.

**Scope of this task only**: stabilize the magnet at ONE fixed operating
point `y0`, and demonstrate small reference steps around it. Do **not**
attempt large-range motion, trajectory tracking, or nonlinear control here
Steps must stay small (a few mm) so the linear model below remains valid.

## 1. Physical Model

### 1.1 Nonlinear plant

Let `y` be the absolute gap between electromagnet and magnet, `i` the coil
current, `m` the levitated mass, `g` gravitational acceleration. For a
permanent magnet (fixed moment) under a coil, the attractive force is
approximately linear in current and inverse-square in gap:

```
F(y, i) = K * i / y^2          (K > 0, lumped magnetic constant)
m*y_ddot = m*g - F(y, i)
L*di/dt + R*i = u               (coil electrical dynamics; R, L measured)
```

### 1.2 Linearized plant at equilibrium `(y0, i0)`

At equilibrium: `m*g = K*i0/y0^2`. Define perturbations `y' = y - y0`,
`Δi = i - i0`, `Δu = u - u0`. First-order Taylor expansion gives:

```
y_ddot' - b*y' = -c*Δi           where   b = 2*g/y0 ,   c = g/i0
```

Neglecting the fast electrical pole (`R/L` is much faster than the
mechanical dynamics `sqrt(b)`; verify this with measured R, L before
neglecting it — flag it if it turns out not to hold):

```
P(s) = Y'(s)/ΔU(s) = -c' / (s^2 - b),      c' = c/R
```

This plant has poles at `s = +sqrt(b)` and `s = -sqrt(b)` — one right-half-
plane pole, i.e. **open-loop unstable**. This is why closed-loop feedback is
mandatory and why you must never command an open-loop current/voltage step
(see pitfalls in §5).

### 1.3 Controller: PD, derivative-on-measurement

Use derivative-on-measurement (NOT derivative-of-error) to avoid a
closed-loop zero and "derivative kick" on setpoint changes:

```
u(t) = kP * (r(t) - y(t)) - kD * y_dot_filtered(t)
```

`y_dot_filtered` is the derivative of the measured position, passed through
a single-pole low-pass filter (time constant `tau`, a fixed design constant,
NOT swept as an experimental factor) to avoid amplifying sensor noise:

```
y_dot_filtered(s) = [s / (tau*s + 1)] * Y(s)
```

### 1.4 Closed-loop characteristic equation & performance formulas

```
s^2 + c'*kD*s + (c'*kP - b) = 0

Stability requires:  c'*kP > b   AND   kD > 0

omega_n = sqrt(c'*kP - b)
zeta    = c'*kD / (2*omega_n)
Mp      = exp( -pi*zeta / sqrt(1 - zeta^2) )                (overshoot, fraction)
ts      ≈ 3 / (zeta*omega_n) = 6 / (c'*kD)      (±5% settling time)
```

Both Mp and ts are defined **relative to the final steady-state value of
y(t)**, not relative to r, because a pure PD controller (no integral term)
has nonzero DC steady-state error.

## 2. Implementation

- **Entry point: [`python/run_console.py`](python/run_console.py)** --
  `cd python && PYTHONPATH=. python run_console.py`. Opens a real-time
  window running a structural port of the Arduino firmware
  (`python/maglev_sim/arduino_port.py`) against the true nonlinear plant,
  paced against your actual clock. Type serial commands (`R 55`, `KP
  2000`, ...) into the terminal while it runs and watch the response live.
- [`arduino/maglev_controller/`](arduino/maglev_controller/) -- the
  controller firmware (sensor/actuator access left as stubs).
- [`python/`](python/) -- nonlinear-plant simulation, a Python mirror of the
  firmware's control algorithm for fast verification without hardware, a
  serial hardware-in-the-loop harness for verification *with* hardware, a
  real-time console (above), and the two sweep experiments (Kp/Kd vs.
  overshoot/settling time; step size vs. linear-region validity).
- [`PARAMETERS.md`](PARAMETERS.md) -- where every numeric constant used
  above comes from (measured, chosen, or derived from one calibration
  measurement) -- start here before touching real hardware. It also
  documents what simulation-based verification caught: a sign inconsistency
  between this README's §1.3 and §1.4, a control-loop sample-rate
  requirement driven by the coil's electrical dynamics, and why a
  VL53L0X-realistic ~30Hz sensor cannot stabilize this plant at all via
  direct position-to-voltage PD control at any gain -- which is why the
  equilibrium gap `y0` is 50mm rather than a tighter value: it's the
  smallest gap at which a non-expensive 60Hz sensor works instead.
