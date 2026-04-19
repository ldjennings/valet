"""
Shared math and state utilities used across Bots and Planner packages.

Consolidates duplicated angle-wrapping, state unpacking, interpolation,
and Reeds-Shepp boilerplate into a single module.
"""

import math
import numpy as np
import reeds_shepp
from Bots.BotState import PointState, DiffState, CarState, TrailerState, S


def wrap_angle(delta: float) -> float:
    """Wrap an angular difference to [-π, π]."""
    return (delta + math.pi) % (2 * math.pi) - math.pi


def angle_distance(a: float, b: float) -> float:
    """Absolute angular difference from b to a, wrapped to [-π, π]. Equivalent to abs(wrap_angle(a - b)).
    can optionally
    """
    return abs(wrap_angle(a - b))


def angle_difference(a: float, b: float) -> float:
    """Angular difference from b to a, wrapped to [-π, π]. Equivalent to wrap_angle(a - b).
    can optionally
    """
    return wrap_angle(a - b)


def pos(state) -> tuple[float, float]:
    """Extract the (x, y) position from any state type via __iter__."""
    x, y, *_ = state
    return (x, y)


def angs(state) -> tuple[float, ...] | None:
    """Extract angle components (heading, trailer heading, ...) from a state."""
    _, _, *rest = state
    if rest is None:
        return None
    else:
        return tuple(rest)

def heading(state) -> float | None:
    """Extract heading from a state. Returns None if not present"""
    _, _, rest, *_ = state
    if rest is None:
        return None
    else:
        return rest


def center_distance(s1, s2) -> float:
    """Euclidean distance between the position components of two states."""
    x1, y1 = pos(s1)
    x2, y2 = pos(s2)
    return math.hypot(x2 - x1, y2 - y1)


def lerp_angle(a: float, b: float, t: float) -> float:
    """Linearly interpolate between angles a and b, handling wrapping."""
    return a + t * wrap_angle(b - a)


def linspace_angles(a: float, b: float, resolution: float) -> list[float]:
    """Interpolate from angle a to angle b at `resolution`-radian steps, handling wrapping."""
    delta = wrap_angle(b - a)

    n = max(2, math.ceil(abs(delta) / resolution) + 1)

    return [float(h) for h in np.linspace(a, a + delta, n)]


def linspace_xy(p1: tuple[float, float], p2: tuple[float, float], resolution: float
) -> list[tuple[float, float]]:
    """Interpolate from (x1,y1) to (x2,y2) at `resolution`-meter steps."""
    dist = center_distance(p1, p2)
    num_steps = max(2, math.ceil(dist / resolution) + 1)

    ps = np.linspace(p1, p2, num_steps)

    return [(float(x), float(y)) for x, y in ps]


def rs_path_length(start, goal, turning_radius: float) -> float:
    """Reeds-Shepp path length between two states (ignoring obstacles)."""
    x0, y0, h0, *_ = start
    x1, y1, h1, *_ = goal
    return reeds_shepp.path_length((x0, y0, h0), (x1, y1, h1), turning_radius)


def rs_path_sample(start, goal, turning_radius: float, resolution: float = 0.1) -> list[tuple] | None:
    """
    Sample a Reeds-Shepp path between two states.
    Returns the raw sample list from pyReedsShepp, or None if the path is empty.
    """
    x0, y0, h0, *_ = start
    x1, y1, h1, *_ = goal
    raw = reeds_shepp.path_sample((x0, y0, h0), (x1, y1, h1), turning_radius, resolution)
    return raw if raw else None


def arc_len(s1, s2, ang_weight: float) -> float:
    x1, y1, h1, *_ = s1
    x2, y2, h2, *_ = s2
    return center_distance((x1, y1), (x2, y2)) + angle_difference(h1, h2) * ang_weight


def trajectory_length(traj: list[S], ang_weight: float) -> float:
    if isinstance(traj[0], PointState):
        return sum(
            center_distance(p0, p1)
            for p0, p1 in zip(traj, traj[1:])
        )
    else:
        # assert
        return sum(
            arc_len(p0, p1, ang_weight)
            for p0, p1 in zip(traj, traj[1:])
        )
