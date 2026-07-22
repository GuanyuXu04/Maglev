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
_DEFAULT_KP = 1018.7
_DEFAULT_KD = 43.9554

# --- mirrors of the .ino's sensor-filtering constants ---------------------
# Ported alongside the .ino's measurement filter. These are firmware
# behaviour, not plant parameters, so they live here rather than in
# params.py -- but they MUST match the .ino, or this class stops being a
# faithful port and the HIL comparison becomes meaningless.
_TAU_MEAS_S = 0.005                 # TAU_MEAS_S
_Y_VALID_MIN_M = 0.005              # Y_VALID_MIN_M
_Y_VALID_MAX_M = 0.200              # Y_VALID_MAX_M
_MAX_CONSECUTIVE_BAD_SAMPLES = 10   # MAX_CONSECUTIVE_BAD_SAMPLES
_DT_CLAMP_MAX_S = 0.20              # the .ino's dt sanity clamp threshold

# Mirrors the .ino's SAMPLE_NONE / SAMPLE_OK / SAMPLE_BAD tri-state. The
# distinction matters: "no new sample this tick" is the normal case and must
# hold the last output, whereas "fresh but invalid" has to be counted so a
# permanently blinded sensor cannot masquerade as a healthy slow one.
SAMPLE_NONE = 0
SAMPLE_OK = 1
SAMPLE_BAD = -1


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

        # --- mirrors g_haveYFilt / g_yFilt_m / g_badSampleCount ---
        self._have_y_filt = False
        self._y_filt_m = op.y0
        self._bad_sample_count = 0

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

    def sensor_read_gap_meters(self) -> tuple[int, float]:
        """Mirrors the .ino's sensorReadGapMeters(): returns
        (status, gap_m) where status is SAMPLE_NONE / SAMPLE_OK / SAMPLE_BAD.

        The provider contract is unchanged (None == "no new sample"); the
        validity gate is applied here, on the value the provider returns, so
        that the .ino's Y_VALID_MIN_M/Y_VALID_MAX_M check has an exact
        counterpart. A provider may also return a NaN/out-of-range value to
        simulate a VL53L1X that has lost the target.
        """
        if self._sensor_provider is None:
            return SAMPLE_NONE, self._y_filt_m
        y = self._sensor_provider()
        if y is None:
            return SAMPLE_NONE, self._y_filt_m
        if not (_Y_VALID_MIN_M <= y <= _Y_VALID_MAX_M):
            return SAMPLE_BAD, self._y_filt_m
        return SAMPLE_OK, y

    def filter_measurement(self, dt: float, y_raw: float) -> float:
        """Mirrors the .ino's filterMeasurement(): first-order low-pass on
        the position measurement, with alpha derived from the *actual*
        elapsed dt so the corner frequency does not move with the sample
        rate. The .ino's second EMA-on-velocity is deliberately absent in
        both places -- PDController already low-passes the derivative with
        the Tustin-discretized s/(tau*s+1).
        """
        if not self._have_y_filt:
            self._y_filt_m = y_raw
            self._have_y_filt = True
            return self._y_filt_m
        alpha = dt / (_TAU_MEAS_S + dt)
        self._y_filt_m += alpha * (y_raw - self._y_filt_m)
        return self._y_filt_m

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
                self._have_y_filt = False      # re-seed from the next sample
                self._bad_sample_count = 0
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

        Returns (have_new_sample, u_volts_commanded), where have_new_sample
        is True only for SAMPLE_OK -- a rejected sample reports False, like
        the "no data" case, because in both the controller did not run.
        """
        if self.sim_mode:
            status = SAMPLE_OK if self._have_sim_sample else SAMPLE_NONE
            y_m = self._sim_y_m
            self._have_sim_sample = False
        else:
            status, y_m = self.sensor_read_gap_meters()

        # The controller must see the time since it last actually ran, not
        # the tick interval -- when the sensor is slower than the tick (the
        # normal case, see PARAMETERS.md "Sample-rate reality check"), those
        # differ by 20-30x, and the derivative filter's Tustin coefficients
        # are only valid for the interval the (y - y_prev) difference
        # actually spans. Passing the tick interval instead made the filter
        # massively overestimate velocity (confirmed: ~3-8x at a 30Hz sensor
        # rate against a 1kHz tick) -- enough to destabilize the demo gains
        # outright. This mirrors the .ino's g_lastControlMicros fix. The
        # measurement filter's alpha is derived from the same dt.
        self._time_since_last_control += dt

        if status == SAMPLE_BAD:
            # Transient dropout: skip the sample, hold the last output. Only
            # a sustained dropout is a fault -- this plant diverges in ~45ms,
            # so de-energizing on a single bad reading is a self-inflicted
            # crash. Mirrors the .ino exactly.
            if self._bad_sample_count < _MAX_CONSECUTIVE_BAD_SAMPLES:
                self._bad_sample_count += 1
            if self._bad_sample_count >= _MAX_CONSECUTIVE_BAD_SAMPLES:
                self._controller.reset()
                self._have_y_filt = False
                self._bad_sample_count = 0
                self.last_u_V = 0.0    # de-energize
        elif status == SAMPLE_OK:
            self._bad_sample_count = 0
            control_dt = self._time_since_last_control
            if control_dt <= 0.0 or control_dt > _DT_CLAMP_MAX_S:
                control_dt = self.loop_cfg.dt      # the .ino's dt sanity clamp
            y_filtered = self.filter_measurement(control_dt, y_m)
            self.last_u_V = self._controller.update(control_dt, self.setpoint_m, y_filtered)
            self._time_since_last_control = 0.0
        # else (SAMPLE_NONE): hold self.last_u_V -- matches the VL53L1X's
        # slower-than-loop update rate, see PARAMETERS.md.

        have_new_sample = status == SAMPLE_OK
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
