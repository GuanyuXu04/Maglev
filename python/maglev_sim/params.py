"""Single source of truth for every physical/design parameter.

See ../../PARAMETERS.md for the full discussion of where each number comes
from (direct measurement, design choice, or derived via calibration). Every
number here is a *placeholder* self-consistent with the equilibrium
equation `m*g = K*i0/y0**2` -- not a claim about any real hardware.

The Arduino sketch (arduino/maglev_controller/maglev_controller.ino) hardcodes
the same numeric values in its constants block. `tests/test_maglev_sim.py`
parses that file and checks it against this module so the two cannot
silently drift apart -- if you change a value here, update the .ino too.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class PlantParams:
    """Physical constants. Bucket A (measure) + Bucket C (derive from calibration)."""

    g: float = 9.81          # m/s^2 -- standard gravity, not measured
    m: float = 0.020         # kg -- PLACEHOLDER, replace with a scale reading
    R: float = 8.0           # ohm -- PLACEHOLDER, replace with a multimeter reading
    L: float = 0.020         # H   -- PLACEHOLDER, replace with an LR step-response test
    K: float = 4.905e-5      # N*m^2/A -- derived, see equilibrium() below


@dataclass(frozen=True)
class OperatingPoint:
    """Bucket B design choices: where we linearize around."""

    y0: float = 0.010        # m -- equilibrium gap
    i0: float = 0.400        # A -- equilibrium coil current


@dataclass(frozen=True)
class LoopTiming:
    """Bucket B design choices: sample time and derivative filter."""

    dt: float = 0.001        # s -- control loop tick (1 kHz); see PARAMETERS.md
                             # "electrical-pole aliasing" -- 200 Hz was too slow
                             # to resolve the coil's ~2.5ms electrical time
                             # constant and was discrete-time unstable.
    tau: float = 0.010       # s -- derivative low-pass filter time constant


@dataclass(frozen=True)
class ActuatorLimits:
    """Hardware limits for the LMD18200 H-bridge + supply."""

    supply_voltage: float = 12.0   # V -- assumed bench supply
    pwm_max: int = 255             # 8-bit analogWrite range
    current_limit: float = 3.0     # A -- LMD18200 continuous rating (datasheet)


PLANT = PlantParams()
OP = OperatingPoint()
LOOP = LoopTiming()
ACTUATOR = ActuatorLimits()


def K_from_equilibrium(m: float, g: float, y0: float, i0: float) -> float:
    """K derived from a single hovering calibration point (PARAMETERS.md, bucket C)."""
    return m * g * y0 ** 2 / i0


def u0_from_equilibrium(i0: float, R: float) -> float:
    """Equilibrium coil voltage: at DC, L*di/dt = 0, so u0 = i0*R."""
    return i0 * R


def check_equilibrium_consistency(plant: PlantParams = PLANT, op: OperatingPoint = OP,
                                   rtol: float = 1e-9) -> None:
    """Raise if the committed placeholder numbers don't actually satisfy m*g = K*i0/y0**2.

    This is a guard against editing one placeholder number without the others --
    the whole point of the "self-consistent fictional data sheet" is that it
    stays consistent.
    """
    lhs = plant.m * plant.g
    rhs = plant.K * op.i0 / op.y0 ** 2
    if abs(lhs - rhs) > rtol * max(abs(lhs), abs(rhs), 1e-12):
        raise ValueError(
            f"Equilibrium violated: m*g={lhs:.6e} but K*i0/y0**2={rhs:.6e}. "
            "Either recompute K via K_from_equilibrium(), or fix whichever "
            "placeholder changed."
        )
