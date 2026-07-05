"""Structural port of arduino/maglev_controller/maglev_controller.ino.

This does NOT re-derive the control law -- that would be a third place
(after the .ino and reference_controller.py) for the sign or discretization
to drift out of sync. It reuses `reference_controller.PDController` for
`computeControl()`'s math, and ports everything AROUND it: the same runtime
state, the same serial command vocabulary, the same fixed-rate `loop()`
structure, and the same two hardware seams
(`sensorReadGapMeters`/`actuatorWriteVoltageCommand`). Running an
`ArduinoFirmware` instance with a real serial link and real peripherals
wired to those two seams *is* running the firmware -- this class is not a
simulation-specific redesign of the controller.

Correspondence to the .ino:

| .ino                                | here                                    |
|--------------------------------------|------------------------------------------|
| `g_kP`, `g_kD`                       | `self.kP`, `self.kD`                     |
| `g_setpoint_m`                       | `self.setpoint_m`                        |
| `g_u0_V`                             | `self.u0_V`                              |
| `g_simMode`                          | `self.sim_mode`                          |
| `g_haveSimSample`, `g_simY_m`        | `self._have_sim_sample`, `self._sim_y_m` |
| `g_lastU_V`                          | `self.last_u_V`                          |
| `sensorReadGapMeters()`              | `self.sensor_read_gap_meters()` -- calls the provider from `set_sensor_provider()` |
| `actuatorWriteVoltageCommand()`      | `self.actuator_write_voltage_command()` -- calls the sink from `set_actuator_sink()` |
| `computeControl()`                   | `self._controller.update(...)` (`PDController`) |
| `handleCommand()`                    | `self.handle_command(line)`              |
| `loop()`                             | `self.loop_tick(dt)`                     |

`loop_tick(dt)` takes the caller-measured elapsed time as `dt`, mirroring
the .ino's `micros()`-based measurement of actual elapsed time rather than
assuming a constant -- see `run_console.py`, which is responsible for the
`micros()`-style rate gating the real chip's `loop()` does in hardware.
"""

from __future__ import annotations

from typing import Callable, Optional

from . import params
from .reference_controller import ControllerParams, PDController

# Must match the .ino's g_kP/g_kD initializers exactly (both are computed
# from the same design point; test_ino_default_gains_match_demo_design_point
# checks the .ino side against linearize.py). Recomputed for y0=50mm -- see
# PARAMETERS.md "Why a 30Hz sensor cannot stabilize this plant".
_DEFAULT_KP = 361.28
_DEFAULT_KD = 17.446536945083025


class ArduinoFirmware:
    """One instance == one Arduino running maglev_controller.ino."""

    def __init__(self, plant: params.PlantParams = params.PLANT,
                 op: params.OperatingPoint = params.OP,
                 loop: params.LoopTiming = params.LOOP,
                 actuator: params.ActuatorLimits = params.ACTUATOR):
        self.plant, self.op, self.loop_cfg, self.actuator = plant, op, loop, actuator

        # --- mirrors the .ino's g_* runtime state ---
        self.kP = _DEFAULT_KP
        self.kD = _DEFAULT_KD
        self.setpoint_m = op.y0          # g_setpoint_m = Y0_M
        self.u0_V = params.u0_from_equilibrium(op.i0, plant.R)  # g_u0_V
        self.last_u_V = self.u0_V        # g_lastU_V

        self.sim_mode = False            # g_simMode
        self._have_sim_sample = False    # g_haveSimSample
        self._sim_y_m = op.y0            # g_simY_m

        self._controller = PDController(ControllerParams(
            kP=self.kP, kD=self.kD, tau=loop.tau, u0=self.u0_V,
            u_min=-actuator.supply_voltage, u_max=actuator.supply_voltage,
        ))

        self._sensor_provider: Optional[Callable[[], Optional[float]]] = None
        self._actuator_sink: Optional[Callable[[float], None]] = None
        self._time_since_last_control = 0.0  # accumulates over ticks with no new sample

    # -- hardware seams: mirror the .ino's TODO(hardware) stub bodies -----
    def set_sensor_provider(self, fn: Callable[[], Optional[float]]) -> None:
        """fn() -> gap in meters, or None for "no new sample" (mirrors the
        real sensorReadGapMeters() stub returning false). Real firmware
        would wire this to VL53L0X I2C access; run_console.py wires it to
        a rate-limited read of the simulated plant's true state.
        """
        self._sensor_provider = fn

    def set_actuator_sink(self, fn: Callable[[float], None]) -> None:
        """fn(u_volts) -> None. Real firmware would wire this to the
        LMD18200 PWM/DIR pins; run_console.py wires it to record the
        commanded voltage for the plant integrator to apply.
        """
        self._actuator_sink = fn

    def sensor_read_gap_meters(self) -> Optional[float]:
        if self._sensor_provider is None:
            return None
        return self._sensor_provider()

    def actuator_write_voltage_command(self, u_volts: float) -> None:
        if self._actuator_sink is not None:
            self._actuator_sink(u_volts)

    # -- serial protocol: mirrors handleCommand() -------------------------
    def handle_command(self, line: str) -> Optional[str]:
        """Same command vocabulary as the .ino: KP, KD, R, U0, SIM, Y,
        RESET, PING. Returns a reply string (PING -> "PONG", errors -> a
        message) to print/echo, or None for commands that don't reply --
        exactly like the .ino only ever Serial.println()s for PING.
        """
        parts = line.strip().split()
        if not parts:
            return None
        cmd, arg = parts[0].upper(), (parts[1] if len(parts) > 1 else None)
        try:
            if cmd == "KP" and arg is not None:
                self.kP = float(arg)
                self._controller.p.kP = self.kP
            elif cmd == "KD" and arg is not None:
                self.kD = float(arg)
                self._controller.p.kD = self.kD
            elif cmd == "R" and arg is not None:
                self.setpoint_m = float(arg) / 1000.0
            elif cmd == "U0" and arg is not None:
                self.u0_V = float(arg)
                self._controller.p.u0 = self.u0_V
            elif cmd == "SIM" and arg is not None:
                self.sim_mode = int(arg) != 0
                self._have_sim_sample = False
            elif cmd == "Y" and arg is not None:
                self._sim_y_m = float(arg) / 1000.0
                self._have_sim_sample = True
            elif cmd == "RESET":
                self._controller.reset()
            elif cmd == "PING":
                return "PONG"
            else:
                return f"ERR unrecognized: {line!r}"
        except ValueError:
            return f"ERR bad argument: {line!r}"
        return None

    # -- fixed-rate loop: mirrors loop() -----------------------------------
    def loop_tick(self, dt: float) -> tuple[bool, float]:
        """One pass through the .ino's loop() body, past the point where
        its micros()-based rate gate would have just fired -- the caller
        (run_console.py) is responsible for calling this at (approximately)
        LOOP_DT_S real-time intervals, exactly as the chip's own micros()
        check paces the real loop(). `dt` is the tick interval (time since
        the last call to loop_tick), NOT necessarily the interval passed to
        the controller -- see below.

        Returns (have_new_sample, u_volts_commanded).
        """
        if self.sim_mode:
            have_new_sample = self._have_sim_sample
            y_m = self._sim_y_m
            self._have_sim_sample = False
        else:
            y_m = self.sensor_read_gap_meters()
            have_new_sample = y_m is not None

        # The controller must see the time since it last actually ran, not
        # the tick interval -- when the sensor is slower than the tick (the
        # normal case, see PARAMETERS.md "Sample-rate reality check"), those
        # differ by 20-30x, and the derivative filter's Tustin coefficients
        # are only valid for the interval the (y - y_prev) difference
        # actually spans. Passing the tick interval instead made the filter
        # massively overestimate velocity (confirmed: ~3-8x at a 30Hz sensor
        # rate against a 1kHz tick) -- enough to destabilize the demo gains
        # outright. This mirrors the .ino's g_lastControlMicros fix.
        self._time_since_last_control += dt
        if have_new_sample:
            self.last_u_V = self._controller.update(self._time_since_last_control, self.setpoint_m, y_m)
            self._time_since_last_control = 0.0
        # else: hold self.last_u_V -- matches the VL53L0X's slower-than-loop
        # update rate, see PARAMETERS.md "Sample-rate reality check".

        self.actuator_write_voltage_command(self.last_u_V)
        return have_new_sample, self.last_u_V


# 60Hz: an achievable rate for a VL53L0X-class sensor without moving to
# more expensive hardware, and the rate params.OP.y0 (50mm) was specifically
# chosen to make stabilizable with real margin -- see PARAMETERS.md "Why a
# 30Hz sensor cannot stabilize this plant" and its "Resolution" subsection.
# This plant is NOT stabilizable at 30Hz at any y0 practical for a desktop
# rig with a single-loop design; 60Hz + y0=50mm is the validated, working
# combination (stable with margin down to ~45Hz, not just exactly at 60Hz).
DEFAULT_SENSOR_PERIOD_S = 1.0 / 60.0


class SimulatedPeripherals:
    """Stands in for the real VL53L0X + LMD18200 wiring, for driving an
    ArduinoFirmware against the simulated plant (used by run_console.py,
    and by tests that don't want a GUI/matplotlib dependency). `true_y_m`
    must be updated by the caller's physics step every tick; read_sensor()
    only hands a fresh value to the firmware at a configurable rate (else
    None, "no new sample yet" -- exactly like the real VL53L0X stub), and
    write_actuator() just records the commanded voltage for the physics
    step to apply as a zero-order hold, exactly as a real coil would
    experience a PWM-commanded voltage between updates.
    """

    def __init__(self, initial_y_m: float, sensor_period_s: float = DEFAULT_SENSOR_PERIOD_S):
        self.sim_time = 0.0
        self.sensor_period_s = sensor_period_s
        self._next_sample_time = 0.0
        self.true_y_m = initial_y_m
        self.u_volts = 0.0

    def read_sensor(self) -> Optional[float]:
        if self.sim_time + 1e-12 < self._next_sample_time:
            return None
        self._next_sample_time = self.sim_time + self.sensor_period_s
        return self.true_y_m

    def write_actuator(self, u_volts: float) -> None:
        self.u_volts = u_volts
