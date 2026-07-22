"""Linearized-model formulas from README.md sections 1.2 and 1.4.

All formulas here are transcribed directly from the README; nothing is
independently derived in this module beyond simple algebraic inversion
(solving the same equations for kP, kD instead of for omega_n, zeta).
"""

from dataclasses import dataclass
import math

import numpy as np

from .params import PlantParams, OperatingPoint, PLANT, OP


@dataclass(frozen=True)
class LinearCoeffs:
    b: float    # 2*g/y0, s^-2 -- open-loop instability rate squared
    c: float    # g/i0
    c_prime: float  # c/R -- README's c'


def linear_coeffs(plant: PlantParams = PLANT, op: OperatingPoint = OP) -> LinearCoeffs:
    # b = 2g/y0 is README 1.2's design-model instability rate. It intentionally
    # does NOT include the permanent-magnet core-magnetization term the
    # nonlinear plant (plant.py) now carries -- that term steepens the true
    # open-loop instability to 2(g + K_pm/(m*y0**4))/y0, a deliberately-
    # unmodeled effect the controller must be robust to, exactly like the
    # neglected electrical pole and derivative-filter lag. See PARAMETERS.md.
    b = 2.0 * plant.g / op.y0
    c = plant.g / op.i0
    c_prime = c / plant.R
    return LinearCoeffs(b=b, c=c, c_prime=c_prime)


def open_loop_pole(plant: PlantParams = PLANT, op: OperatingPoint = OP) -> float:
    """sqrt(b): magnitude of the right-half-plane pole (rad/s)."""
    return math.sqrt(linear_coeffs(plant, op).b)


def check_electrical_pole_fast_enough(plant: PlantParams = PLANT, op: OperatingPoint = OP,
                                       min_ratio: float = 5.0) -> float:
    """Ratio (R/L) / sqrt(b). README says this must be >>1 to neglect the coil's
    electrical dynamics. Returns the ratio; raises if it's below min_ratio.
    """
    ratio = (plant.R / plant.L) / open_loop_pole(plant, op)
    if ratio < min_ratio:
        raise ValueError(
            f"R/L is only {ratio:.2f}x sqrt(b) -- the 'fast electrical pole' "
            "assumption in README 1.2 does not hold with these R, L. The "
            "third-order plant would need to be controlled directly."
        )
    return ratio


def kp_kd_from_zeta_omega(zeta: float, omega_n: float,
                           plant: PlantParams = PLANT, op: OperatingPoint = OP) -> tuple[float, float]:
    """Invert README 1.4's omega_n/zeta formulas to get controller gains."""
    coeffs = linear_coeffs(plant, op)
    kP = (omega_n ** 2 + coeffs.b) / coeffs.c_prime
    kD = 2.0 * zeta * omega_n / coeffs.c_prime
    return kP, kD


def zeta_omega_from_kp_kd(kP: float, kD: float,
                           plant: PlantParams = PLANT, op: OperatingPoint = OP) -> tuple[float, float]:
    """README 1.4 forward direction: gains -> (omega_n, zeta)."""
    coeffs = linear_coeffs(plant, op)
    omega_n_sq = coeffs.c_prime * kP - coeffs.b
    if omega_n_sq <= 0:
        raise ValueError(
            f"c'*kP={coeffs.c_prime * kP:.4f} <= b={coeffs.b:.4f}: unstable, "
            "README 1.4 stability condition c'*kP > b violated."
        )
    omega_n = math.sqrt(omega_n_sq)
    zeta = coeffs.c_prime * kD / (2.0 * omega_n)
    return omega_n, zeta


def theoretical_overshoot(zeta: float) -> float:
    """Mp = exp(-pi*zeta/sqrt(1-zeta^2)), as a fraction. 0 for zeta >= 1."""
    if zeta >= 1.0:
        return 0.0
    return math.exp(-math.pi * zeta / math.sqrt(1.0 - zeta ** 2))


def theoretical_settling_time(zeta: float, omega_n: float,
                               plant: PlantParams = PLANT, op: OperatingPoint = OP) -> float:
    """ts ~= 3/(zeta*omega_n) = 6/(c'*kD), +-5% band, per README 1.4."""
    return 3.0 / (zeta * omega_n)


def theoretical_response(zeta: float, omega_n: float) -> tuple[float, float]:
    """Convenience: (Mp, ts) for a given (zeta, omega_n)."""
    return theoretical_overshoot(zeta), 3.0 / (zeta * omega_n)


def simulate_ideal_linear_response(zeta: float, omega_n: float, step_size: float,
                                    t_end: float, dt: float,
                                    plant: PlantParams = PLANT, op: OperatingPoint = OP
                                    ) -> tuple[np.ndarray, np.ndarray]:
    """Exact response of the *idealized* README 1.4 closed loop:

        y_ddot' + 2*zeta*omega_n*y_dot' + omega_n^2*y' = c'*kP*step_size

    i.e. no coil inductance and no derivative-filter lag -- both are real
    simplifications this repo's controller/plant necessarily include (see
    PARAMETERS.md), so this idealized trajectory is a better ground truth to
    verify the discrete implementation against than the closed-form Mp/ts
    point formulas above, which are themselves approximations (exact only
    for zeta<1; increasingly optimistic for zeta>=1 where the true dominant
    pole is slower than zeta*omega_n -- see PARAMETERS.md). Returned as
    (t, y_absolute) in the same convention as plant.simulate_closed_loop's
    result so metrics.step_response_metrics() applies identically to both.
    """
    coeffs = linear_coeffs(plant, op)
    kP, _ = kp_kd_from_zeta_omega(zeta, omega_n, plant, op)
    forcing = coeffs.c_prime * kP * step_size

    n = int(round(t_end / dt))
    t = dt * np.arange(n)
    y = np.zeros(n)
    yp, ypdot = 0.0, 0.0

    def deriv(a: float, b: float) -> np.ndarray:
        return np.array([b, forcing - 2.0 * zeta * omega_n * b - omega_n ** 2 * a])

    for k in range(n):
        y[k] = op.y0 + yp
        k1 = deriv(yp, ypdot)
        k2 = deriv(yp + 0.5 * dt * k1[0], ypdot + 0.5 * dt * k1[1])
        k3 = deriv(yp + 0.5 * dt * k2[0], ypdot + 0.5 * dt * k2[1])
        k4 = deriv(yp + dt * k3[0], ypdot + dt * k3[1])
        yp += (dt / 6.0) * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        ypdot += (dt / 6.0) * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])

    return t, y
