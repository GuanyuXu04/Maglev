#!/usr/bin/env python3
"""
============================================================================
 ENTRY POINT -- run this file.

     cd python && PYTHONPATH=. python run_console.py

============================================================================

A real-time console for the maglev simulation: it runs
`maglev_sim.arduino_port.ArduinoFirmware` -- the ported
`arduino/maglev_controller/maglev_controller.ino` -- continuously, at the
firmware's real 1kHz tick rate, paced against your machine's actual clock
(not sped up, not precomputed). The plant is the true *nonlinear* model
(`maglev_sim.plant`, `F=K*i/y^2`, no linearization). The scene starts with
the magnet levitated at rest at the fixed equilibrium `params.OP.y0`,
exactly like flipping on real hardware that's already holding position.

While it runs, type serial commands into THIS TERMINAL, one per line,
Enter to send -- the same commands you'd type into the Arduino Serial
Monitor against real hardware:

    R 55        set the setpoint (absolute gap, mm) -- e.g. a step input
                (y0=50mm; keep steps within roughly +-40% of y0 -- see
                PARAMETERS.md/experiments/exp2 for the safe linear region)
    KP 2000     set the proportional gain live
    KD 50       set the derivative gain live
    U0 3.2      override the equilibrium feedforward voltage
    RESET       clear the derivative-filter state
    PING        should reply PONG (link check)

A window opens showing:
  - left: the electromagnet (fixed) and the levitated magnet (moving, to
    true relative scale) -- what you'd see looking at the physical rig.
  - right: a scrolling strip-chart of the gap's actual measured history --
    a record of the past only. It is built by appending each tick's result
    as it happens; nothing is precomputed or known ahead of time, exactly
    as a real oscilloscope/serial-plotter trace works.

Ported peripheral interface (see maglev_sim/arduino_port.py and
arduino/maglev_controller/maglev_controller.ino for the real firmware side):
  - reading the air gap: SimulatedPeripherals.read_sensor(), wired to
    ArduinoFirmware.set_sensor_provider(). Rate-limited to mimic a real
    sensor's achievable update rate (see --sensor-hz below).
  - commanding the coil voltage u: SimulatedPeripherals.write_actuator(),
    wired to ArduinoFirmware.set_actuator_sink(). The plant integrates this
    voltage exactly as the LMD18200 would drive real coil current.
  - the serial command line itself: stdin, read on a background thread and
    fed line-by-line into ArduinoFirmware.handle_command() -- the identical
    parser the real firmware runs.

BACKGROUND -- why y0=40mm, not something tighter:
    Default: --sensor-hz 60, mimicking an achievable (not-expensive)
    VL53L0X-class sensor rate. Building this console at the ORIGINAL
    y0=10mm is what *discovered* that a ~30Hz sensor cannot stabilize this
    plant via direct position-to-voltage PD control AT ALL, for any gains
    -- not a tuning problem: the mechanical open-loop instability's own
    time constant at 10mm is ~22.6ms (1/sqrt(b)), already faster than one
    33ms sample period, so the loop can't react before the gap has run
    away. `b = 2g/y0` shrinks as y0 grows, slowing that instability; y0 was
    moved to 40mm specifically because it's the smallest gap (checked by
    direct discrete forward-simulation, not asserted) at which a 60Hz
    sensor gives a comfortably stable margin with this repo's existing
    single-loop design -- no cascaded current loop needed. See
    PARAMETERS.md "Why a 30Hz sensor cannot stabilize this plant" and its
    "Resolution" section for the full derivation, and for what to do if
    your own rig's `y0`/`R`/`L` differ enough to change this conclusion.
    Pass --sensor-hz 1000 for an idealized fast sensor (matching what
    experiments/exp1 and exp2 assume) if you want to isolate gain-tuning
    behavior from this sample-rate constraint.

Press Ctrl+C in this terminal to quit.
"""

from __future__ import annotations

import argparse
import queue
import sys
import threading
import time
from collections import deque

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from maglev_sim import params, plant, visualize, arduino_port

FPS = 30
HISTORY_WINDOW_S = 4.0          # scrolling strip-chart window
MAX_TICKS_PER_FRAME = 400       # real-time catch-up cap (0.4s of sim time)
SENSOR_PERIOD_S = arduino_port.DEFAULT_SENSOR_PERIOD_S  # mimics the VL53L0X's real achievable rate


def _stdin_reader(q: "queue.Queue[str]") -> None:
    for line in sys.stdin:
        line = line.strip()
        if line:
            q.put(line)


def build_app(sensor_period_s: float = SENSOR_PERIOD_S):
    """Builds the figure and the per-frame update callback, without starting
    the event loop -- split out from main() so tests/tools can drive
    `on_frame` directly without opening a real window loop. `sensor_period_s`
    is exposed for experimenting with a faster/slower simulated sensor than
    the ~30Hz VL53L0X default (e.g. pass 0.0 for an idealized always-fresh
    sensor, matching the assumption experiments/exp1 and exp2 use).
    """
    op, plant_params, loop_cfg = params.OP, params.PLANT, params.LOOP

    firmware = arduino_port.ArduinoFirmware(plant=plant_params, op=op, loop=loop_cfg)
    peripherals = arduino_port.SimulatedPeripherals(op.y0, sensor_period_s=sensor_period_s)
    firmware.set_sensor_provider(peripherals.read_sensor)
    firmware.set_actuator_sink(peripherals.write_actuator)

    state = [op.y0, 0.0, op.i0]  # true nonlinear plant state: [y, y_dot, i]
    sim_time = 0.0

    history_t = deque(maxlen=int(HISTORY_WINDOW_S / loop_cfg.dt) + 10)
    history_y = deque(maxlen=history_t.maxlen)
    history_r = deque(maxlen=history_t.maxlen)

    cmd_queue: "queue.Queue[str]" = queue.Queue()
    threading.Thread(target=_stdin_reader, args=(cmd_queue,), daemon=True).start()

    print(__doc__)
    print(f"Equilibrium: y0 = {op.y0*1000:.2f} mm, i0 = {op.i0:.3f} A, "
          f"u0 = {firmware.u0_V:.2f} V (see PARAMETERS.md)")
    print(f"Demo gains: kP = {firmware.kP:.1f}, kD = {firmware.kD:.2f}")
    sensor_hz = float("inf") if sensor_period_s <= 0 else 1.0 / sensor_period_s
    print(f"Simulated sensor rate: {sensor_hz:.0f} Hz "
          f"(sensor_period_s={sensor_period_s*1000:.1f} ms)")
    if sensor_hz < 45:
        print("WARNING: at this sensor rate, the demo gains are expected to diverge for "
              "ANY step -- verified stable only down to ~45Hz at this y0. See the module "
              "docstring / PARAMETERS.md 'Why a 30Hz sensor cannot stabilize this plant'.")
    print("Ready -- type a command and press Enter (e.g. `R 55`):\n")

    # --- figure: schematic (left) + causal-only strip chart (right) -----
    fig, (ax_schem, ax_trace) = plt.subplots(1, 2, figsize=(10, 4.8),
                                              gridspec_kw={"width_ratios": [1, 1.4]})
    fig.suptitle("Maglev real-time console -- type serial commands in the terminal")

    y0_mm = op.y0 * 1000.0
    y_hard_max_mm = params.LIMITS.y_max * 1000.0   # ground -- see PARAMETERS.md
    y_hard_min_mm = params.LIMITS.y_min * 1000.0   # ceiling/electromagnet face
    coil_height_mm = 0.12 * y0_mm * 2.0
    magnet_width_mm = y0_mm * 2.0 * 0.35
    # Starts zoomed in near y0 (so small steps are still visible), and only
    # zooms out toward the physical ground as far as it actually needs to --
    # never further, since y can never exceed y_hard_max_mm.
    view = {"y_max_mm": min(y0_mm * 2.2, y_hard_max_mm)}

    ax_schem.set_xlim(-view["y_max_mm"] * 0.6, view["y_max_mm"] * 0.6)
    ax_schem.set_ylim(view["y_max_mm"] * 1.08, -coil_height_mm * 1.6)
    ax_schem.set_aspect("equal")
    ax_schem.axis("off")
    visualize.draw_electromagnet(ax_schem, width_mm=y0_mm * 2.0 * 0.7, height_mm=coil_height_mm)
    ground_width_mm = y_hard_max_mm * 1.3
    ax_schem.add_patch(plt.Rectangle((-ground_width_mm / 2, y_hard_max_mm), ground_width_mm,
                                      y_hard_max_mm * 0.04, facecolor=visualize.GRAY,
                                      edgecolor=visualize.GRAY, hatch="//", zorder=2))
    ax_schem.text(0, y_hard_max_mm * 1.06, "ground", ha="center", va="top",
                  fontsize=9, color=visualize.GRAY)
    magnet = visualize.MagnetArtist(ax_schem, width_mm=magnet_width_mm, height_mm=magnet_width_mm * 0.35)
    gap_line, = ax_schem.plot([0, 0], [0, y0_mm], ls="--", color=visualize.GRAY_LIGHT, lw=1.2, zorder=1)
    gap_text = ax_schem.text(view["y_max_mm"] * 0.22, y0_mm / 2, "", fontsize=9,
                              color=visualize.GRAY, va="center")
    info_text = ax_schem.text(-view["y_max_mm"] * 0.58, -coil_height_mm * 1.4, "", fontsize=8.5,
                               va="top", color=visualize.GRAY, family="monospace")

    ax_trace.set_xlabel("time (s)")
    ax_trace.set_ylabel("gap y (mm)  -- history only")
    ax_trace.spines[["top", "right"]].set_visible(False)
    ax_trace.grid(True, alpha=0.25, lw=0.6)
    (line_r,) = ax_trace.plot([], [], ls="--", lw=1.4, color=visualize.GRAY_LIGHT, label="setpoint r(t)")
    (line_y,) = ax_trace.plot([], [], lw=2.0, color=visualize.BLUE, label="measured gap y(t)")
    ax_trace.legend(fontsize=8, loc="upper right", frameon=False)

    clock = {"last_wall": time.perf_counter(), "sim_time": 0.0}

    def on_frame(_frame):
        now = time.perf_counter()
        elapsed = now - clock["last_wall"]
        clock["last_wall"] = now

        while not cmd_queue.empty():
            line = cmd_queue.get_nowait()
            reply = firmware.handle_command(line)
            print(f"> {line}" + (f"   {reply}" if reply else ""))

        n_ticks = min(int(round(elapsed / loop_cfg.dt)), MAX_TICKS_PER_FRAME)
        for _ in range(n_ticks):
            peripherals.sim_time = clock["sim_time"]
            peripherals.true_y_m = state[0]
            firmware.loop_tick(loop_cfg.dt)
            state[:] = plant.integrate(state, peripherals.u_volts, loop_cfg.dt,
                                        plant_params, substeps=5)
            clock["sim_time"] += loop_cfg.dt
            history_t.append(clock["sim_time"])
            history_y.append(state[0])
            history_r.append(firmware.setpoint_m)

        y_mm = state[0] * 1000.0
        if y_mm > 0.85 * view["y_max_mm"]:
            # Zoom out only as far as needed -- capped at the physical ground,
            # since y can never exceed y_hard_max_mm (see apply_travel_limits).
            view["y_max_mm"] = min(max(y_mm * 1.3, y0_mm * 0.5), y_hard_max_mm)
            ax_schem.set_ylim(view["y_max_mm"] * 1.08, -coil_height_mm * 1.6)
            ax_schem.set_xlim(-view["y_max_mm"] * 0.6, view["y_max_mm"] * 0.6)

        magnet.set_y(y_mm)
        gap_line.set_data([0, 0], [0, y_mm])
        gap_text.set_position((view["y_max_mm"] * 0.22, y_mm / 2))
        gap_text.set_text(f"gap = {y_mm:.2f} mm")
        info_text.set_text(
            f"t  = {clock['sim_time']:7.2f} s\n"
            f"r  = {firmware.setpoint_m*1000:7.2f} mm\n"
            f"y  = {y_mm:7.2f} mm\n"
            f"u  = {peripherals.u_volts:7.2f} V\n"
            f"i  = {state[2]:7.3f} A\n"
            f"kP = {firmware.kP:7.1f}\n"
            f"kD = {firmware.kD:7.2f}\n"
            f"travel limits: [{y_hard_min_mm:.2f}, {y_hard_max_mm:.0f}] mm"
        )

        if history_t:
            t_arr = list(history_t)
            line_y.set_data(t_arr, [v * 1000.0 for v in history_y])
            line_r.set_data(t_arr, [v * 1000.0 for v in history_r])
            t_now = t_arr[-1]
            ax_trace.set_xlim(max(0.0, t_now - HISTORY_WINDOW_S), max(t_now, HISTORY_WINDOW_S))
            y_lo = min(min(history_y), min(history_r)) * 1000.0
            y_hi = max(max(history_y), max(history_r)) * 1000.0
            pad = max((y_hi - y_lo) * 0.15, 0.5)
            ax_trace.set_ylim(y_lo - pad, y_hi + pad)

        return magnet.artists + (gap_line, gap_text, info_text, line_r, line_y)

    fig.tight_layout()
    return fig, on_frame, firmware, cmd_queue


def main() -> None:
    ap = argparse.ArgumentParser(description="Real-time maglev console (see module docstring).")
    ap.add_argument("--sensor-hz", type=float, default=1.0 / SENSOR_PERIOD_S,
                     help="Simulated position-sensor update rate in Hz. Default ~30, mimicking "
                          "a real VL53L0X. Pass a large value (e.g. 1000) or 0 for an idealized "
                          "always-fresh sensor -- see the module docstring for why the realistic "
                          "default cannot stabilize the demo gains at all.")
    args = ap.parse_args()
    sensor_period_s = 0.0 if args.sensor_hz <= 0 else 1.0 / args.sensor_hz

    fig, on_frame, _firmware, _cmd_queue = build_app(sensor_period_s=sensor_period_s)
    anim = FuncAnimation(fig, on_frame, interval=1000.0 / FPS, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
