"""Step-response metrics computed the same way README 1.4 defines them:
relative to the response's own final steady-state value, not the commanded
reference (a pure PD loop has nonzero DC error, so those two differ).

Convention used throughout this package: the step is commanded starting at
t=0 and the plant/controller state is initialized at the pre-step
equilibrium, so `y[0]` is exactly the pre-step value and `t` doubles as
"time since the step."
"""

import numpy as np


def steady_state_value(y: np.ndarray, tail_fraction: float = 0.1) -> float:
    n = len(y)
    start = max(int(round(n * (1.0 - tail_fraction))), n - 1)
    return float(np.mean(y[start:]))


def step_response_metrics(t: np.ndarray, y: np.ndarray, settle_band: float = 0.05,
                           tail_fraction: float = 0.1) -> tuple[float, float, float]:
    """Returns (Mp, ts, y_final).

    Mp: overshoot as a fraction of |y_final - y_initial|, 0 if the response
        never crosses beyond y_final in the step direction.
    ts: first time after which |y(t) - y_final| stays within
        settle_band * |y_final - y_initial| for the rest of the recorded
        horizon. Equals t[-1] (i.e. "did not settle") if it never does.
    """
    y_initial = float(y[0])
    y_final = steady_state_value(y, tail_fraction=tail_fraction)
    delta = y_final - y_initial

    if abs(delta) < 1e-12:
        return 0.0, 0.0, y_final

    if delta > 0:
        overshoot_abs = float(np.max(y)) - y_final
    else:
        overshoot_abs = y_final - float(np.min(y))
    Mp = max(overshoot_abs, 0.0) / abs(delta)

    band = settle_band * abs(delta)
    within = np.abs(y - y_final) <= band
    not_within = np.where(~within)[0]
    if len(not_within) == 0:
        ts = 0.0
    else:
        last_bad = not_within[-1]
        ts = float(t[-1]) if last_bad + 1 >= len(t) else float(t[last_bad + 1] - t[0])

    return Mp, ts, y_final
