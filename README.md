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
