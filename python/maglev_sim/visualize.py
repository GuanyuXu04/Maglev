"""Simple schematic animation of a simulated response: watch the levitated
object move relative to the (fixed) electromagnet over time, alongside its
y(t) trace.

Usage:
    from maglev_sim import params, linearize, plant, visualize
    from maglev_sim.reference_controller import ControllerParams, PDController

    op, plant_params, loop = params.OP, params.PLANT, params.LOOP
    kP, kD = linearize.kp_kd_from_zeta_omega(1.0, 1.35 * linearize.open_loop_pole())
    controller = PDController(ControllerParams.from_design(kP, kD))
    result = plant.simulate_closed_loop(controller, plant_params, op,
                                         lambda t: op.y0 + 0.001, 0.3, loop.dt)

    anim = visualize.animate_response(result, op=op, plant=plant_params,
                                       save_path="results/demo.gif")

Run this module directly for a ready-made example:
    cd python && PYTHONPATH=. python -m maglev_sim.visualize
"""

from __future__ import annotations

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation

from .params import PlantParams, OperatingPoint, ActuatorLimits, PLANT, OP, ACTUATOR
from .plant import SimResult

# Colors: reference-palette categorical slot 1 (blue) for the primary y(t)
# trace/series identity; slot 6 (red) paired with blue for the classic N/S
# bar-magnet symbol; a neutral gray for the (non-data) electromagnet
# schematic; the reserved status "critical" red for the actuator-saturation
# flag, kept visually distinct from the categorical red used on the magnet.
BLUE = "#2a78d6"
RED = "#e34948"
GRAY = "#52514e"
GRAY_LIGHT = "#c3c2b7"
CRITICAL = "#d03b3b"


def draw_electromagnet(ax, width_mm: float, height_mm: float) -> None:
    """Fixed symbol at y=0: a laminated core with a coil wrapped around it."""
    core = Rectangle((-width_mm * 0.18, -height_mm), width_mm * 0.36, height_mm,
                      facecolor=GRAY_LIGHT, edgecolor=GRAY, linewidth=1.2, zorder=2)
    ax.add_patch(core)
    coil = Rectangle((-width_mm * 0.5, -height_mm), width_mm, height_mm * 0.28,
                      facecolor="none", edgecolor=GRAY, linewidth=1.4, zorder=3)
    ax.add_patch(coil)
    for i in range(5):
        x = -width_mm * 0.42 + i * (width_mm * 0.84 / 4)
        ax.plot([x, x], [-height_mm, -height_mm * 0.72], color=GRAY, lw=1.4, zorder=3)
    ax.text(0, -height_mm * 1.15, "electromagnet", ha="center", va="bottom",
            fontsize=9, color=GRAY)


class MagnetArtist:
    """The levitated permanent magnet: a small N/S bar magnet, redrawn each frame."""

    def __init__(self, ax, width_mm: float, height_mm: float):
        self.width_mm = width_mm
        self.height_mm = height_mm
        self.top = Rectangle((-width_mm / 2, 0), width_mm, height_mm / 2,
                              facecolor=RED, edgecolor="black", linewidth=1.0, zorder=5)
        self.bottom = Rectangle((-width_mm / 2, 0), width_mm, height_mm / 2,
                                 facecolor=BLUE, edgecolor="black", linewidth=1.0, zorder=5)
        ax.add_patch(self.top)
        ax.add_patch(self.bottom)
        self.label_n = ax.text(0, 0, "N", ha="center", va="center", fontsize=7,
                                color="white", weight="bold", zorder=6)
        self.label_s = ax.text(0, 0, "S", ha="center", va="center", fontsize=7,
                                color="white", weight="bold", zorder=6)

    def set_y(self, y_mm: float) -> None:
        h = self.height_mm
        self.top.set_xy((-self.width_mm / 2, y_mm - h / 2))
        self.bottom.set_xy((-self.width_mm / 2, y_mm))
        self.label_n.set_position((0, y_mm - h / 4))
        self.label_s.set_position((0, y_mm + h / 4))

    @property
    def artists(self):
        return (self.top, self.bottom, self.label_n, self.label_s)


def animate_response(result: SimResult, op: OperatingPoint = OP, plant: PlantParams = PLANT,
                      actuator: ActuatorLimits = ACTUATOR, fps: int = 30, speed: float = 0.15,
                      min_frames: int = 90, save_path: str | None = None,
                      title: str | None = None) -> FuncAnimation:
    """Animate a plant.simulate_closed_loop() result: a schematic panel (fixed
    electromagnet, moving levitated magnet, live gap readout) beside a y(t)
    trace with a synced time marker.

    `speed` is playback speed relative to real time (default 0.15, i.e.
    ~7x slow motion -- this system's dynamics settle in tens of
    milliseconds, too fast to actually see at real-time speed). `min_frames`
    puts a floor on frame count so short/fast responses still animate
    smoothly rather than jumping between a handful of frames.

    If `save_path` ends in .gif it's written with Pillow (no external
    dependency); .mp4 requires ffmpeg to be installed. Without `save_path`,
    the returned FuncAnimation can be displayed with
    `HTML(anim.to_jshtml())` in a notebook, or the caller can call
    `.save(...)` itself.
    """
    t_ms = result.t * 1000.0
    y_mm = result.y * 1000.0
    r_mm = result.r * 1000.0
    y0_mm = op.y0 * 1000.0

    n_frames = max(int(round((result.t[-1] - result.t[0]) * fps / speed)), min_frames)
    frame_idx = np.linspace(0, len(result.t) - 1, n_frames).astype(int)

    fig, (ax_schem, ax_trace) = plt.subplots(1, 2, figsize=(10, 4.5),
                                              gridspec_kw={"width_ratios": [1, 1.4]})

    # --- schematic panel ---
    y_span_mm = max(np.max(y_mm), np.max(r_mm), y0_mm) - min(np.min(y_mm), np.min(r_mm), y0_mm)
    y_max_mm = max(np.max(y_mm), np.max(r_mm), y0_mm) + 0.15 * max(y_span_mm, y0_mm)
    coil_height_mm = 0.12 * y_max_mm
    ax_schem.set_xlim(-y_max_mm * 0.6, y_max_mm * 0.6)
    ax_schem.set_ylim(y_max_mm, -coil_height_mm * 1.6)  # inverted: coil at top, larger gap lower
    ax_schem.set_aspect("equal")
    ax_schem.axis("off")
    draw_electromagnet(ax_schem, width_mm=y_max_mm * 0.7, height_mm=coil_height_mm)

    magnet_width_mm = y_max_mm * 0.35
    magnet = MagnetArtist(ax_schem, width_mm=magnet_width_mm, height_mm=magnet_width_mm * 0.35)

    gap_line, = ax_schem.plot([0, 0], [0, y0_mm], ls="--", color=GRAY_LIGHT, lw=1.2, zorder=1)
    gap_text = ax_schem.text(y_max_mm * 0.22, y0_mm / 2, "", fontsize=9, color=GRAY, va="center")
    info_text = ax_schem.text(-y_max_mm * 0.58, -coil_height_mm * 1.4, "", fontsize=8.5,
                               va="top", color=GRAY, family="monospace")
    sat_text = ax_schem.text(0, y_max_mm * 0.96, "", fontsize=9, ha="center",
                              color=CRITICAL, weight="bold")

    # --- trace panel ---
    ax_trace.plot(t_ms, r_mm, ls="--", lw=1.4, color=GRAY_LIGHT, label="reference r(t)")
    ax_trace.plot(t_ms, y_mm, lw=2.0, color=BLUE, label="gap y(t)")
    time_marker = ax_trace.axvline(t_ms[0], color=GRAY, lw=1.0, alpha=0.7)
    (dot,) = ax_trace.plot([t_ms[0]], [y_mm[0]], "o", color=BLUE, ms=6, zorder=5)
    ax_trace.set_xlabel("time (ms)")
    ax_trace.set_ylabel("gap y (mm)")
    ax_trace.spines[["top", "right"]].set_visible(False)
    ax_trace.grid(True, alpha=0.25, lw=0.6)
    ax_trace.legend(fontsize=8, loc="best", frameon=False)

    if title:
        fig.suptitle(title)
    fig.tight_layout()

    def update(k: int):
        i = frame_idx[k]
        magnet.set_y(y_mm[i])
        gap_line.set_data([0, 0], [0, y_mm[i]])
        gap_text.set_position((y_max_mm * 0.22, y_mm[i] / 2))
        gap_text.set_text(f"gap = {y_mm[i]:.2f} mm")
        info_text.set_text(
            f"t = {t_ms[i]:6.1f} ms\n"
            f"i = {result.i[i]:6.3f} A\n"
            f"u = {result.u[i]:6.2f} V"
        )
        saturated = np.abs(result.u[i]) >= actuator.supply_voltage - 1e-6
        sat_text.set_text("ACTUATOR SATURATED" if saturated else "")
        time_marker.set_xdata([t_ms[i], t_ms[i]])
        dot.set_data([t_ms[i]], [y_mm[i]])
        return magnet.artists + (gap_line, gap_text, info_text, sat_text, time_marker, dot)

    anim = FuncAnimation(fig, update, frames=n_frames, interval=1000.0 / fps, blit=False)

    if save_path:
        if save_path.endswith(".gif"):
            anim.save(save_path, writer="pillow", fps=fps)
        else:
            anim.save(save_path, writer="ffmpeg", fps=fps)
        print(f"wrote {save_path}")

    return anim


if __name__ == "__main__":
    from pathlib import Path
    from . import linearize
    from . import plant as plant_mod
    from .reference_controller import ControllerParams, PDController

    matplotlib.use("Agg")

    omega_n = 1.35 * linearize.open_loop_pole()
    kP, kD = linearize.kp_kd_from_zeta_omega(1.0, omega_n)
    controller = PDController(ControllerParams.from_design(kP, kD))
    step = 0.15 * OP.y0  # a visually clear move; still inside exp2's "safe" region on the + side
    result = plant_mod.simulate_closed_loop(controller, PLANT, OP, lambda t: OP.y0 + step, 0.3, 0.001)

    out_dir = Path(__file__).resolve().parents[1] / "results"
    out_dir.mkdir(exist_ok=True)
    out_path = out_dir / "demo_animation.gif"
    animate_response(result, save_path=str(out_path),
                      title="Demo: 15%-of-y0 step at the critical-damping design point")
