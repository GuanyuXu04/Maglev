"""Correctness checks for the simulation package and the algorithmic
(software-in-the-loop) verification of the control law that will also run
on the Arduino.

Run with: pytest python/tests
"""

import math
import re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")

import numpy as np
import pytest

from maglev_sim import params, linearize, plant, metrics, visualize, arduino_port
from maglev_sim.reference_controller import ControllerParams, PDController

REPO_ROOT = Path(__file__).resolve().parents[2]
INO_PATH = REPO_ROOT / "arduino" / "maglev_controller" / "maglev_controller.ino"


def test_equilibrium_consistency():
    params.check_equilibrium_consistency()


def test_K_matches_equilibrium_derivation():
    K = params.K_from_equilibrium(params.PLANT.m, params.PLANT.g, params.OP.y0, params.OP.i0)
    assert K == pytest.approx(params.PLANT.K, rel=1e-9)


def test_electrical_pole_much_faster_than_mechanical():
    ratio = linearize.check_electrical_pole_fast_enough(min_ratio=5.0)
    assert ratio >= 5.0


def test_open_loop_pole_is_real_and_positive():
    b = linearize.linear_coeffs().b
    assert b > 0
    assert linearize.open_loop_pole() == pytest.approx(math.sqrt(b))


def test_open_loop_is_actually_unstable_in_nonlinear_sim():
    """Hold u at the equilibrium feedforward u0 (no feedback) and perturb y
    slightly; the nonlinear plant should run away from equilibrium, not
    return to it -- this is the open-loop-instability claim in README 1.2,
    checked against the *nonlinear* model rather than just trusted from the
    linearization.
    """
    op = params.OP
    plant_params = params.PLANT
    u0 = params.u0_from_equilibrium(op.i0, plant_params.R)
    state = np.array([op.y0 * 1.02, 0.0, op.i0])  # 2% high perturbation, no feedback
    dt = 0.001
    for _ in range(200):  # 0.2s open loop
        state = plant.integrate(state, u0, dt, plant_params, substeps=4)
    assert abs(state[0] - op.y0) > 0.02 * op.y0 * 2  # perturbation grew, didn't shrink


def test_open_loop_fall_is_caught_by_the_ground():
    """Same open-loop setup as above but run for long enough, and with a
    weaker current, that the magnet would otherwise fall indefinitely --
    it must stop exactly at params.LIMITS.y_max, not sail past it.
    """
    op, plant_params, limits = params.OP, params.PLANT, params.LIMITS
    state = np.array([op.y0, 0.0, 0.0])  # no current at all -- free fall
    dt = 0.001
    for _ in range(2000):  # 2s, far longer than needed to hit the ground
        state = plant.integrate(state, 0.0, dt, plant_params, substeps=4)
    assert state[0] == pytest.approx(limits.y_max)
    assert state[1] == pytest.approx(0.0)  # resting, not still falling


def test_open_loop_rise_is_caught_by_the_ceiling():
    """Symmetric case: enough current to overpower gravity indefinitely
    should stop at y_min, not crash through the electromagnet or go
    negative (which would also blow up F=K*i/y^2).
    """
    op, plant_params, limits = params.OP, params.PLANT, params.LIMITS
    big_current = 50 * op.i0  # comically large, just needs to keep pulling up
    state = np.array([op.y0, 0.0, big_current])
    dt = 0.001
    for _ in range(2000):
        state = plant.integrate(state, plant_params.R * big_current, dt, plant_params, substeps=4)
    assert state[0] == pytest.approx(limits.y_min)
    assert state[1] == pytest.approx(0.0)
    assert np.all(np.isfinite(state))


def test_magnet_leaves_the_ground_once_it_can():
    """The ground/ceiling clamp must not be a one-way trap: apply_travel_limits
    only zeros the velocity component driving *into* a boundary, so a state
    already moving away from it (e.g. once the controller pulls hard enough
    to overcome gravity at that position) must pass through unmodified --
    this is a direct unit test of that release logic; realistically,
    reaching y_max at all with this repo's placeholder K would need currents
    well beyond CURRENT_LIMIT_A to ever lift off again (F ~ 1/y^2 makes the
    electromagnet ~81x weaker at y_max than at y0), which is exactly why
    it's a "ground" -- see PARAMETERS.md "Ground and ceiling travel limits".
    """
    limits = params.LIMITS
    still_falling = np.array([limits.y_max, 0.5, 0.0])  # positive = still trying to increase y
    assert plant.apply_travel_limits(still_falling, limits)[1] == 0.0  # stopped, can't fall further

    lifting_off = np.array([limits.y_max, -0.5, 0.0])  # already moving away (negative = toward coil)
    clamped = plant.apply_travel_limits(lifting_off, limits)
    assert clamped[0] == limits.y_max
    assert clamped[1] == -0.5  # velocity away from the boundary is left alone, not zeroed


def test_kp_kd_zeta_omega_roundtrip():
    for zeta in (0.4, 0.7, 1.0, 1.5):
        for mult in (1.0, 1.5, 2.0):
            omega_n = mult * linearize.open_loop_pole()
            kP, kD = linearize.kp_kd_from_zeta_omega(zeta, omega_n)
            omega_n_back, zeta_back = linearize.zeta_omega_from_kp_kd(kP, kD)
            assert zeta_back == pytest.approx(zeta, rel=1e-6)
            assert omega_n_back == pytest.approx(omega_n, rel=1e-6)


def test_unstable_gains_are_rejected():
    with pytest.raises(ValueError):
        linearize.zeta_omega_from_kp_kd(kP=0.0, kD=1.0)


def _t_end_for(zeta: float, omega_n: float) -> float:
    decay_rate = zeta * omega_n if zeta <= 1.0 else zeta * omega_n - omega_n * math.sqrt(zeta ** 2 - 1)
    return 15.0 / decay_rate


@pytest.mark.parametrize("zeta,mult", [(1.0, 1.35), (0.7, 1.5), (1.5, 1.2)])
def test_reference_controller_matches_ideal_linear_model_in_the_fast_filter_limit(zeta, mult):
    """Tier 1 (tight): does the discrete algorithm itself -- Tustin-filtered
    derivative, sign-corrected PD, feedforward, closing the loop against the
    *nonlinear* plant -- correctly reduce to README 1.4's idealized 2nd-order
    theory in the limit where the two things that theory neglects (coil
    inductance, derivative-filter lag) are actually negligible?

    This is checked against linearize.simulate_ideal_linear_response (the
    exact ODE), not the closed-form Mp/ts point formulas, because those
    formulas are themselves only approximations for zeta>=1 (confirmed
    separately: they undershoot true settling time by ~1.6x at zeta=1 and by
    much more for zeta>1, a well-known property of the 3/(zeta*omega_n) rule,
    not a bug -- see PARAMETERS.md).

    tau here is overridden to 0.2ms purely as a controlled test harness (not
    a realistic value -- the committed tau=10ms is deliberately slower, for
    noise rejection) so this test isolates "is the algorithm right" from
    "does the realistic, deliberately-non-ideal tau/L match idealized
    theory" -- the latter is answered (with a documented, expected gap) by
    test_closed_loop_stays_bounded_and_qualitatively_correct below.
    """
    op, plant_params = params.OP, params.PLANT
    omega_n = mult * linearize.open_loop_pole()
    kP, kD = linearize.kp_kd_from_zeta_omega(zeta, omega_n)

    fast_tau = 0.0002  # 0.2ms: >>5-10x faster than every omega_n tested here
    dt = 0.0002
    ctrl_params = ControllerParams.from_design(kP, kD)
    ctrl_params.tau = fast_tau
    controller = PDController(ctrl_params)

    step = 0.001 * op.y0  # 0.1% of y0: deep in the linear range
    r_fn = lambda t: op.y0 + step
    t_end = _t_end_for(zeta, omega_n)

    result = plant.simulate_closed_loop(controller, plant_params, op, r_fn, t_end, dt)
    Mp_sim, ts_sim, _ = metrics.step_response_metrics(result.t, result.y)

    t_id, y_id = linearize.simulate_ideal_linear_response(zeta, omega_n, step, t_end, dt)
    Mp_id, ts_id, _ = metrics.step_response_metrics(t_id, y_id)

    if Mp_id < 1e-6:
        assert Mp_sim < 0.02
    else:
        # rel=0.8: tau is faked fast here, but L=20mH (a real, deliberately-kept
        # 9x-not-more margin, see PARAMETERS.md) still isn't negligible for the
        # most underdamped case tried (zeta=0.7) -- confirmed separately that
        # shrinking L further (a synthetic, numerically-stiff plant) tightens
        # this to <10%, so the residual here is that specific known gap, not a
        # bug.
        assert Mp_sim == pytest.approx(Mp_id, rel=0.8)
    # same L-residual reasoning as Mp's tolerance above applies to ts for the
    # zeta=0.7 case; the zeta>=1 cases match to <10% at this tolerance.
    assert ts_sim == pytest.approx(ts_id, rel=0.65)


@pytest.mark.parametrize("zeta,mult", [(1.0, 1.35), (0.7, 1.5), (1.5, 1.2)])
def test_closed_loop_stays_bounded_and_qualitatively_correct(zeta, mult):
    """Tier 2 (loose): at the *actual committed* placeholders (tau=10ms,
    L=20mH -- deliberately not fast/negligible, see PARAMETERS.md), the
    closed loop must still be well-behaved: bounded, settling near the
    commanded step, with overshoot present for underdamped designs and
    absent for critically-/over-damped ones. It is NOT expected to
    quantitatively match the idealized formulas this tightly -- neglecting a
    9x-separated electrical pole and a filter corner only ~1.7x above
    omega_n costs real accuracy (empirically up to ~5x on ts and ~6x on Mp
    for the most aggressive underdamped case tried here); that gap is
    exactly what Experiment 1's sweep is for.
    """
    op, plant_params, loop = params.OP, params.PLANT, params.LOOP
    omega_n = mult * linearize.open_loop_pole()
    kP, kD = linearize.kp_kd_from_zeta_omega(zeta, omega_n)
    controller = PDController(ControllerParams.from_design(kP, kD))

    step = 0.05 * op.y0
    r_fn = lambda t: op.y0 + step
    t_end = _t_end_for(zeta, omega_n) * 3

    result = plant.simulate_closed_loop(controller, plant_params, op, r_fn, t_end, loop.dt)
    Mp_sim, ts_sim, y_final = metrics.step_response_metrics(result.t, result.y)

    assert np.all(np.isfinite(result.y))
    assert abs(y_final - op.y0) < 2.0 * step  # settled somewhere sane, didn't run away
    assert 0 < ts_sim < t_end
    if zeta >= 1.0:
        assert Mp_sim < 0.15
    else:
        assert Mp_sim > 0.0


def _parse_ino_constants(text: str) -> dict:
    pattern = re.compile(
        r"static const float (\w+)\s*=\s*([-+0-9.eEfF]+)\s*;"
    )
    out = {}
    for name, value in pattern.findall(text):
        out[name] = float(value.rstrip("fF"))
    return out


def test_ino_constants_match_params_py():
    """Guards against editing a placeholder number in one place and not the
    other -- params.py is the source of truth; the .ino hardcodes the same
    values (it can't import Python), so this test is the only thing keeping
    them honest.
    """
    text = INO_PATH.read_text()
    ino = _parse_ino_constants(text)

    expected = {
        "G_ACCEL": params.PLANT.g,
        "MASS_KG": params.PLANT.m,
        "COIL_R_OHM": params.PLANT.R,
        "COIL_L_H": params.PLANT.L,
        "MAG_K": params.PLANT.K,
        "Y0_M": params.OP.y0,
        "I0_A": params.OP.i0,
        "LOOP_DT_S": params.LOOP.dt,
        "TAU_S": params.LOOP.tau,
        "SUPPLY_VOLTAGE": params.ACTUATOR.supply_voltage,
        "CURRENT_LIMIT_A": params.ACTUATOR.current_limit,
    }
    for name, value in expected.items():
        assert name in ino, f"{name} not found in {INO_PATH}"
        assert ino[name] == pytest.approx(value, rel=1e-3), (
            f"{name} = {ino[name]} in .ino but {value} in params.py"
        )


def test_ino_default_gains_match_demo_design_point():
    text = INO_PATH.read_text()
    kP_match = re.search(r"g_kP\s*=\s*([-+0-9.eEfF]+)\s*;", text)
    kD_match = re.search(r"g_kD\s*=\s*([-+0-9.eEfF]+)\s*;", text)
    assert kP_match and kD_match
    ino_kP = float(kP_match.group(1).rstrip("fF"))
    ino_kD = float(kD_match.group(1).rstrip("fF"))

    omega_n = 1.35 * linearize.open_loop_pole()
    kP, kD = linearize.kp_kd_from_zeta_omega(1.0, omega_n)
    assert ino_kP == pytest.approx(kP, rel=1e-3)
    assert ino_kD == pytest.approx(kD, rel=1e-3)


def test_animate_response_smoke():
    """Not a numerical check -- just confirms animate_response builds a
    FuncAnimation over a real simulated response without erroring, so a
    future refactor breaking the plotting code fails loudly in CI instead
    of only when someone next runs the demo by hand.
    """
    op, plant_params, loop = params.OP, params.PLANT, params.LOOP
    omega_n = 1.35 * linearize.open_loop_pole()
    kP, kD = linearize.kp_kd_from_zeta_omega(1.0, omega_n)
    controller = PDController(ControllerParams.from_design(kP, kD))
    step = 0.05 * op.y0
    result = plant.simulate_closed_loop(controller, plant_params, op,
                                         lambda t: op.y0 + step, 0.05, loop.dt)

    anim = visualize.animate_response(result, op=op, plant=plant_params, min_frames=5)
    assert anim._func(2) is not None


def test_firmware_defaults_match_ino_and_linearize():
    """The port's g_kP/g_kD-equivalent defaults must match both the .ino's
    literal constants and the demo design point they're supposed to encode
    -- three independent places (.ino, linearize.py's formula, and this
    port) that must all agree.
    """
    fw = arduino_port.ArduinoFirmware()
    omega_n = 1.35 * linearize.open_loop_pole()
    kP, kD = linearize.kp_kd_from_zeta_omega(1.0, omega_n)
    assert fw.kP == pytest.approx(kP, rel=1e-3)
    assert fw.kD == pytest.approx(kD, rel=1e-3)
    assert fw.setpoint_m == params.OP.y0
    assert fw.u0_V == pytest.approx(params.OP.i0 * params.PLANT.R)


def test_firmware_serial_protocol():
    fw = arduino_port.ArduinoFirmware()
    assert fw.handle_command("PING") == "PONG"
    assert fw.handle_command("KP 2000") is None
    assert fw.kP == 2000.0
    assert fw._controller.p.kP == 2000.0
    fw.handle_command("KD 50")
    assert fw.kD == 50.0
    fw.handle_command("R 12.5")
    assert fw.setpoint_m == pytest.approx(0.0125)
    fw.handle_command("U0 4.0")
    assert fw.u0_V == 4.0
    assert fw._controller.p.u0 == 4.0
    assert fw.handle_command("NOPE") is not None  # unrecognized -> error string
    assert fw.handle_command("KP notanumber") is not None  # bad arg -> error string
    assert fw.handle_command("") is None  # blank line -> no-op, no crash


def test_firmware_sim_mode_and_hold_behavior():
    """SIM/Y mirror the .ino's privileged-override path; when no sample is
    available (provider returns None, or SIM mode has no pending Y), the
    firmware must hold last_u_V rather than erroring or zeroing it.
    """
    fw = arduino_port.ArduinoFirmware()

    fw.handle_command("SIM 1")
    fw.handle_command(f"Y {params.OP.y0 * 1000.0}")
    had_sample, u1 = fw.loop_tick(params.LOOP.dt)
    assert had_sample is True

    had_sample, u2 = fw.loop_tick(params.LOOP.dt)  # no new Y sent
    assert had_sample is False
    assert u2 == u1  # held, not recomputed

    fw.handle_command("SIM 0")
    fw.set_sensor_provider(lambda: None)  # mirrors the real stub, no sensor wired
    had_sample, u3 = fw.loop_tick(params.LOOP.dt)
    assert had_sample is False
    assert u3 == u2


def test_firmware_closed_loop_matches_plant_reference_controller():
    """Closing the loop through ArduinoFirmware's sensor/actuator seams
    (reading the plant's true state, exactly as run_console.py's simulated
    peripherals do) must produce the identical trajectory to driving
    reference_controller.PDController directly -- ArduinoFirmware is a
    structural wrapper around the same math, not a different algorithm.
    """
    op, plant_params, loop = params.OP, params.PLANT, params.LOOP
    step = 0.02 * op.y0
    target_m = op.y0 + step

    fw = arduino_port.ArduinoFirmware()
    state = [op.y0, 0.0, op.i0]
    fw.set_sensor_provider(lambda: state[0])
    applied_u = []
    fw.set_actuator_sink(lambda u: applied_u.append(u))
    fw.handle_command(f"R {target_m * 1000.0}")

    n = 100
    fw_ys = []
    for _ in range(n):
        fw_ys.append(state[0])  # record pre-integration, matching simulate_closed_loop's convention
        fw.loop_tick(loop.dt)
        u = applied_u[-1]
        state = list(plant.integrate(np.array(state), u, loop.dt, plant_params, substeps=10))

    controller = PDController(ControllerParams.from_design(fw.kP, fw.kD))
    ref_result = plant.simulate_closed_loop(controller, plant_params, op,
                                             lambda t: target_m, n * loop.dt, loop.dt)

    np.testing.assert_allclose(fw_ys, ref_result.y, rtol=1e-9, atol=1e-12)


def _run_firmware_with_rate_limited_sensor(sensor_period_s, n_ticks, step_frac=0.02):
    """Drives ArduinoFirmware through SimulatedPeripherals exactly as
    run_console.py does, without any matplotlib/GUI dependency.
    """
    op, plant_params, loop = params.OP, params.PLANT, params.LOOP
    fw = arduino_port.ArduinoFirmware(plant=plant_params, op=op, loop=loop)
    peripherals = arduino_port.SimulatedPeripherals(op.y0, sensor_period_s=sensor_period_s)
    fw.set_sensor_provider(peripherals.read_sensor)
    fw.set_actuator_sink(peripherals.write_actuator)
    fw.handle_command(f"R {(op.y0 + step_frac * op.y0) * 1000.0}")

    state = [op.y0, 0.0, op.i0]
    sim_time = 0.0
    ys = np.empty(n_ticks)
    for k in range(n_ticks):
        peripherals.sim_time = sim_time
        peripherals.true_y_m = state[0]
        fw.loop_tick(loop.dt)
        state = list(plant.integrate(state, peripherals.u_volts, loop.dt, plant_params, substeps=5))
        sim_time += loop.dt
        ys[k] = state[0]
    return ys


def _tail_peak_to_peak_mm(ys: np.ndarray, tail: int = 1000) -> float:
    """Peak-to-peak excursion over the last `tail` samples, in mm. A
    genuinely converged response has this near 0 (see calibration in the
    tests below); a sustained, non-decaying oscillation -- this plant's
    actual failure mode once actuator saturation and the ground/ceiling
    travel limits are in play, rather than literal unbounded blow-up --
    does not.
    """
    window = np.asarray(ys[-tail:])
    return float((window.max() - window.min()) * 1000.0)


def test_realtime_console_diverges_at_realistic_30hz_sensor_rate():
    """Locks in a major finding from building the real-time console: with
    a VL53L0X-realistic ~30Hz sensor, the demo gains are NOT just poorly
    tuned -- they are unconditionally unstable, because the plant's own
    open-loop instability time constant (~1/sqrt(b)) is already faster than
    one 33ms sample period, so no feedback correction can arrive in time
    regardless of gain. See PARAMETERS.md "Why a 30Hz sensor cannot
    stabilize this plant". This does NOT show up as literal unbounded
    blow-up here (the ground/ceiling travel limits and actuator saturation
    added since that finding cap it): instead it's a sustained, non-decaying
    oscillation/limit cycle that never converges near the target -- checked
    via peak-to-peak amplitude over the last second of a 4s run, which is
    ~0 for a converged response (see the contrast tests below) and tens of
    mm here. If this test starts failing because the response converges,
    the plant/operating-point placeholders have changed enough to change
    this conclusion -- update PARAMETERS.md rather than "fixing" this test.
    """
    ys = _run_firmware_with_rate_limited_sensor(sensor_period_s=1.0 / 30.0, n_ticks=4000)
    assert _tail_peak_to_peak_mm(ys) > 2.0  # still oscillating, never settled


@pytest.mark.parametrize("sensor_period_s", [0.001, 1.0 / 60.0])
def test_realtime_console_stable_at_validated_sensor_rates(sensor_period_s):
    """Contrast case: the identical firmware and plant, only the simulated
    sensor rate changed to the idealized rate experiments/exp1-2 assume
    (1000Hz) and to 60Hz -- the real, non-expensive rate params.OP.y0=50mm
    was specifically chosen to make stable (see PARAMETERS.md "Why a 30Hz
    sensor cannot stabilize this plant" / "Resolution"). Confirms the demo
    gains and the port are correct, and that the divergence above is
    specifically a sample-rate effect at a too-small y0, not a general bug.
    """
    op = params.OP
    ys = _run_firmware_with_rate_limited_sensor(sensor_period_s=sensor_period_s, n_ticks=4000)
    assert np.all(np.isfinite(ys))
    assert _tail_peak_to_peak_mm(ys) < 0.5  # converged, not oscillating
    assert abs(ys[-1] - op.y0) < 0.1 * op.y0  # settled somewhere sane near the target


def test_realtime_console_marginal_at_45hz_sensor_rate():
    """Pins the documented safety margin: y0=50mm was chosen to stay stable
    down to ~45Hz (not just exactly at the 60Hz target), since real hardware
    won't hit exactly 60.0Hz. 45Hz should still converge; PARAMETERS.md
    documents that 40Hz does not (only checked interactively there, not
    pinned as a test, since asserting a *specific* instability boundary to
    the Hz is more brittle than this repo's other regression tests).
    """
    ys = _run_firmware_with_rate_limited_sensor(sensor_period_s=1.0 / 45.0, n_ticks=4000)
    assert _tail_peak_to_peak_mm(ys) < 1.0
