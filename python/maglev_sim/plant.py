"""Nonlinear plant model, exactly as given in README.md section 1.1:

    F(y, i) = K * i / y^2
    m*y_ddot = m*g - F(y, i)
    L*di/dt + R*i = u

State vector is [y, y_dot, i]. No linearization here -- this is the ground
truth the Arduino controller (or its Python mirror) is tested against.

`y` is also confined to a physical rail (a ground below, the electromagnet's
face above) via `apply_travel_limits()`/`params.LIMITS` -- see
PARAMETERS.md "Ground and ceiling travel limits". This is what stops a
divergent response from reaching physically meaningless positions (an
earlier version of this repo's real-time console let a diverging response
reach `y` = tens of meters).
"""

from dataclasses import dataclass

import numpy as np

from .params import PlantParams, TravelLimits, PLANT, LIMITS


def dynamics(state: np.ndarray, u: float, plant: PlantParams = PLANT) -> np.ndarray:
    y, y_dot, i = state
    F = plant.K * i / y ** 2
    y_ddot = plant.g - F / plant.m
    di_dt = (u - plant.R * i) / plant.L
    return np.array([y_dot, y_ddot, di_dt])


def rk4_step(state: np.ndarray, u: float, dt: float, plant: PlantParams = PLANT) -> np.ndarray:
    k1 = dynamics(state, u, plant)
    k2 = dynamics(state + 0.5 * dt * k1, u, plant)
    k3 = dynamics(state + 0.5 * dt * k2, u, plant)
    k4 = dynamics(state + dt * k3, u, plant)
    return state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


def apply_travel_limits(state: np.ndarray, limits: TravelLimits = LIMITS) -> np.ndarray:
    """Clamp the magnet to the physical rail (ground below, electromagnet
    face above) -- see PARAMETERS.md "Ground and ceiling travel limits".

    Modeled as a simple inelastic stop: position is clamped to the boundary
    and only the velocity component driving it further past the boundary is
    zeroed, so the magnet can still immediately leave the ground/ceiling
    again once the net force reverses (e.g. the controller pulls hard enough
    to lift it off the ground), rather than sticking permanently.
    """
    y, y_dot, i = state
    if y >= limits.y_max:
        y = limits.y_max
        y_dot = min(y_dot, 0.0)
    elif y <= limits.y_min:
        y = limits.y_min
        y_dot = max(y_dot, 0.0)
    return np.array([y, y_dot, i])


def integrate(state: np.ndarray, u: float, dt: float, plant: PlantParams = PLANT,
              substeps: int = 10, limits: TravelLimits = LIMITS) -> np.ndarray:
    """Advance the plant by dt under a zero-order-held input u.

    Sub-stepped for accuracy: the electrical time constant L/R (2.5ms for the
    nominal placeholders) is only ~2.5x the control-loop dt (1ms, itself
    already forced that fast to avoid discrete-time aliasing against this
    same electrical pole -- see PARAMETERS.md "electrical-pole aliasing") --
    a single RK4 step spanning dt would still under-resolve the current
    transient within that tick.

    The travel-limit clamp is applied after every substep (not just once at
    the end) so a fast fall/rise can't blow through the boundary within a
    single dt before being caught.
    """
    h = dt / substeps
    for _ in range(substeps):
        state = rk4_step(state, u, h, plant)
        state = apply_travel_limits(state, limits)
    return state


@dataclass
class SimResult:
    t: np.ndarray
    y: np.ndarray
    y_dot: np.ndarray
    i: np.ndarray
    u: np.ndarray
    r: np.ndarray


def simulate_closed_loop(controller, plant: PlantParams, op, r_fn, t_end: float, dt: float,
                          substeps: int = 10, initial_state: np.ndarray | None = None) -> SimResult:
    """Closed-loop nonlinear simulation using *true* state as feedback.

    `controller` must expose `.update(dt, r, y_true) -> u` (see
    reference_controller.PDController). Using the true, noise-free y as the
    measurement is the "privileged info" simplification appropriate for a
    pure Python/SIL correctness check -- see hil_serial.py for the version
    that instead talks to real Arduino firmware over serial with the same
    privileged-y-injection idea, but through the actual control code.
    """
    n = int(round(t_end / dt))
    t = np.zeros(n)
    y = np.zeros(n)
    y_dot = np.zeros(n)
    i_arr = np.zeros(n)
    u_arr = np.zeros(n)
    r_arr = np.zeros(n)

    state = np.array([op.y0, 0.0, op.i0]) if initial_state is None else np.array(initial_state)

    for k in range(n):
        tk = k * dt
        r = r_fn(tk)
        u = controller.update(dt, r, state[0])
        t[k], y[k], y_dot[k], i_arr[k], u_arr[k], r_arr[k] = tk, state[0], state[1], state[2], u, r
        state = integrate(state, u, dt, plant, substeps=substeps)

    return SimResult(t=t, y=y, y_dot=y_dot, i=i_arr, u=u_arr, r=r_arr)
