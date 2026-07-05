"""Python mirror of the Arduino control law in
arduino/maglev_controller/maglev_controller.ino's computeControl().

This is the "software-in-the-loop" (SIL) half of verification: it is a
line-for-line port of the exact discrete algorithm the microcontroller runs
(same derivative-filter discretization, same feedforward, same saturation),
so that closing the loop against the nonlinear plant.py here is a genuine
check of the *algorithm*, not a different, easier-to-get-right controller.

tests/test_maglev_sim.py additionally parses the .ino source and compares
its numeric constants against maglev_sim.params, so the two
implementations can't silently diverge on the *numbers* either -- only the
translation from Python to C++ syntax is left unverified by this repo (that
part is exercised by actually compiling the sketch, see arduino/README.md).

Derivative filter discretization
---------------------------------
README 1.3 defines the filtered derivative in continuous time as
    y_dot_filtered(s) = [s / (tau*s + 1)] * Y(s)
This module (and the .ino) discretize that transfer function directly with
a bilinear (Tustin) transform, s -> (2/dt)*(z-1)/(z+1), which gives the
well-known "dirty derivative" recursion:
    y_dot_filt[k] = a*y_dot_filt[k-1] + b*(y[k] - y[k-1])
    a = (2*tau - dt) / (2*tau + dt)
    b = 2 / (2*tau + dt)
This is preferred over naively low-pass-filtering a finite-difference
derivative because it is the *exact* discretization of the README's transfer
function (not an approximation of an approximation), and it needs only one
extra state variable (the previous filtered derivative).

Sign correction relative to README 1.3
---------------------------------------
README 1.3 literally states `u(t) = kP*(r-y) - kD*y_dot_filtered`. Implemented
literally, this is unconditionally unstable: the plant's P(s) = -c'/(s^2-b)
(README 1.2) is negative because more coil current pulls the magnet *closer*
(shrinks the gap), so stabilizing feedback must *increase* current when the
gap is too large -- the literal formula does the opposite. Substituting the
literal law into the linearized ODE gives closed-loop characteristic equation
`s^2 - c'*kD*s - (c'*kP+b) = 0`, which has no stable pole placement for any
kP, kD > 0 (confirmed both by hand and symbolically with sympy). Flipping
the sign of the whole PD output, `u(t) = -kP*(r-y) + kD*y_dot_filtered`,
reproduces README 1.4's stated characteristic equation
`s^2 + c'*kD*s + (c'*kP-b) = 0` exactly, making the stability condition and
the omega_n/zeta/Mp/ts formulas in 1.4 internally consistent with the plant
model in 1.1-1.2. This is what's implemented below; see PARAMETERS.md for
the full derivation. (It was also caught empirically: simulating the literal
sign, even for a 5%-of-y0 step, saturates the actuator and drives the gap
through zero within a few sample periods.)
"""

from dataclasses import dataclass

from .params import PlantParams, OperatingPoint, LoopTiming, ActuatorLimits, PLANT, OP, LOOP, ACTUATOR
from .params import u0_from_equilibrium


@dataclass
class ControllerParams:
    kP: float
    kD: float
    tau: float
    u0: float
    u_min: float
    u_max: float

    @classmethod
    def from_design(cls, kP: float, kD: float,
                     plant: PlantParams = PLANT, op: OperatingPoint = OP,
                     loop: LoopTiming = LOOP, actuator: ActuatorLimits = ACTUATOR) -> "ControllerParams":
        u0 = u0_from_equilibrium(op.i0, plant.R)
        return cls(kP=kP, kD=kD, tau=loop.tau, u0=u0,
                   u_min=-actuator.supply_voltage, u_max=actuator.supply_voltage)


class PDController:
    """Derivative-on-measurement PD with filtered derivative and feedforward.

    u(t) = u0 + kP*(r - y) - kD*y_dot_filtered(t), saturated to actuator limits.

    u0 (the equilibrium coil voltage, i0*R) is a feedforward bias, not part of
    the swept PD gains -- README 1.3's `u(t) = kP*e - kD*y_dot_filt` is written
    in perturbation variables around the (y0, i0, u0) operating point, and this
    class adds u0 back to get the absolute command actually sent to hardware.
    Without it, a pure PD term would need e -> u0/kP just to produce enough
    steady current to fight gravity, which is a much larger steady-state error
    than necessary; see PARAMETERS.md and experiments/exp1 for the effect of
    dropping it (`u0=0`) if you want to see this empirically.
    """

    def __init__(self, params: ControllerParams):
        self.p = params
        self._y_prev: float | None = None
        self._y_dot_filt_prev: float = 0.0

    def reset(self) -> None:
        self._y_prev = None
        self._y_dot_filt_prev = 0.0

    def update(self, dt: float, r: float, y: float) -> float:
        p = self.p
        if self._y_prev is None:
            self._y_prev = y

        a = (2.0 * p.tau - dt) / (2.0 * p.tau + dt)
        b = 2.0 / (2.0 * p.tau + dt)
        y_dot_filt = a * self._y_dot_filt_prev + b * (y - self._y_prev)
        self._y_dot_filt_prev = y_dot_filt
        self._y_prev = y

        e = r - y
        delta_u = -p.kP * e + p.kD * y_dot_filt  # sign-flipped vs README 1.3, see module docstring
        u = p.u0 + delta_u

        if u > p.u_max:
            u = p.u_max
        elif u < p.u_min:
            u = p.u_min
        return u
