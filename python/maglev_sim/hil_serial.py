"""Hardware-in-the-loop harness: talks to a REAL Arduino running
arduino/maglev_controller/maglev_controller.ino over serial, using its
SIM-mode protocol to feed it privileged, exact position samples from this
repo's nonlinear plant model in place of the real VL53L0X -- so the actual
compiled firmware computes every control action, with only the sensor
reading intercepted/overridden and the actuator write left inert (SIM mode
never touches the real PWM/DIR pins, see the .ino).

This is the "genuine HIL" counterpart to reference_controller.py's pure
Python mirror (SIL): SIL validates the *algorithm* instantly, without
hardware, by re-implementing the same discrete recursion in Python (and is
what tests/test_maglev_sim.py and the experiments/ scripts use, since no
board is attached in this repo's dev environment). This module validates
the actual .ino once you do have a board plugged in.

Usage:
    python -m maglev_sim.hil_serial --port /dev/ttyACM0 --kp 1806.4 --kd 39.0 \\
        --step-mm 0.2 --duration 0.3
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

import numpy as np

from . import params, plant


@dataclass
class HilResult:
    t: np.ndarray
    y: np.ndarray
    u: np.ndarray


class ArduinoLink:
    """Thin wrapper around the .ino's serial command protocol (see the
    comment above pollSerial() in maglev_controller.ino).
    """

    def __init__(self, port: str, baud: int = 115200, timeout: float = 2.0):
        import serial  # imported lazily so the rest of this module (and the
                        # experiments/tests that don't need real hardware)
                        # doesn't require pyserial to be importable at import time
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2.0)  # a freshly-opened port resets most Arduino boards; let it boot
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def send(self, line: str) -> None:
        self.ser.write((line.strip() + "\n").encode("ascii"))

    def ping(self) -> bool:
        self.ser.reset_input_buffer()
        self.send("PING")
        reply = self.ser.readline().decode("ascii", errors="replace").strip()
        return reply == "PONG"

    def set_gains(self, kP: float, kD: float) -> None:
        self.send(f"KP {kP}")
        self.send(f"KD {kD}")

    def set_setpoint_mm(self, r_mm: float) -> None:
        self.send(f"R {r_mm}")

    def set_sim_mode(self, enabled: bool) -> None:
        self.send(f"SIM {1 if enabled else 0}")

    def reset_controller(self) -> None:
        self.send("RESET")

    def step_sim(self, y_mm: float) -> tuple[float, float, float, float]:
        """Send one privileged position sample; read back the telemetry line
        the .ino emits for the control tick it triggers.

        Returns (t_ms, y_mm_echoed, ydot_filt_mm_s, u_volts).
        """
        self.send(f"Y {y_mm}")
        line = self.ser.readline().decode("ascii", errors="replace").strip()
        parts = line.split(",")
        if len(parts) != 4:
            raise RuntimeError(f"unexpected telemetry line from Arduino: {line!r}")
        t_ms, y_mm_echo, ydot, u = (float(p) for p in parts)
        return t_ms, y_mm_echo, ydot, u

    def close(self) -> None:
        self.ser.close()


def run_hil_step_response(port: str, kP: float, kD: float, step_size_m: float,
                           duration_s: float, plant_params=None, op=None) -> HilResult:
    """Close the loop against the *real* Arduino: it computes u each tick
    from a privileged, exact y this function injects; this function
    integrates the nonlinear plant forward with that u and feeds back the
    new y. Uses the actual measured inter-telemetry time as the plant's
    integration dt (not the nominal LOOP_DT_S), since ticks over USB serial
    aren't perfectly periodic -- see maglev_controller.ino's loop().
    """
    plant_params = plant_params or params.PLANT
    op = op or params.OP

    link = ArduinoLink(port)
    if not link.ping():
        raise RuntimeError(f"Arduino on {port} did not respond to PING")

    link.set_sim_mode(True)
    link.reset_controller()
    link.set_gains(kP, kD)
    target_mm = (op.y0 + step_size_m) * 1000.0
    link.set_setpoint_mm(target_mm)

    state = np.array([op.y0, 0.0, op.i0])
    t_list, y_list, u_list = [], [], []
    t_prev_ms = None
    t_accum = 0.0

    try:
        while t_accum < duration_s:
            y_mm = state[0] * 1000.0
            t_ms, _, _, u = link.step_sim(y_mm)
            dt = 0.001 if t_prev_ms is None else max((t_ms - t_prev_ms) / 1000.0, 1e-6)
            t_prev_ms = t_ms

            t_list.append(t_accum)
            y_list.append(state[0])
            u_list.append(u)

            state = plant.integrate(state, u, dt, plant_params, substeps=10)
            t_accum += dt
    finally:
        link.set_sim_mode(False)
        link.close()

    return HilResult(t=np.array(t_list), y=np.array(y_list), u=np.array(u_list))


def _main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="serial port, e.g. /dev/ttyACM0 or COM3")
    ap.add_argument("--kp", type=float, required=True)
    ap.add_argument("--kd", type=float, required=True)
    ap.add_argument("--step-mm", type=float, default=0.2)
    ap.add_argument("--duration", type=float, default=0.3)
    args = ap.parse_args()

    result = run_hil_step_response(args.port, args.kp, args.kd, args.step_mm / 1000.0, args.duration)
    print("t_s, y_mm, u_V")
    for t, y, u in zip(result.t, result.y, result.u):
        print(f"{t:.4f}, {y * 1000:.4f}, {u:.4f}")


if __name__ == "__main__":
    _main()
