"""
Shared math and geometry utilities used across the Bots and Planner packages.

Covers angle arithmetic, spatial interpolation, analytic arc trajectories,
and Reeds-Shepp path helpers.
"""

import math
from typing import NewType
import numpy as np
import reeds_shepp

Position = NewType('Position', tuple[float, float])
Pose = NewType('Pose', tuple[float, float, float])


# ── Step counting ─────────────────────────────────────────────────────────────

def steps_to_cover(total: float, step_size: float, minimum: int = 1) -> int:
    """Minimum number of steps of `step_size` needed to cover `total`."""
    return max(minimum, math.ceil(total / step_size))


# ── Angle arithmetic ──────────────────────────────────────────────────────────

def wrap_angle(delta: float) -> float:
    """Wrap an angular difference to [-π, π]."""
    return (delta + math.pi) % (2 * math.pi) - math.pi


def angle_distance(a: float, b: float) -> float:
    """Absolute angular difference from b to a, wrapped to [0, π]. Equivalent to abs(wrap_angle(a - b))."""
    return abs(wrap_angle(a - b))


def angle_difference(a: float, b: float) -> float:
    """Angular difference from b to a, wrapped to [-π, π]. Equivalent to wrap_angle(a - b)."""
    return wrap_angle(a - b)


def lerp_angle(a: float, b: float, t: float) -> float:
    """Linearly interpolate between angles a and b, handling wrapping."""
    return a + t * wrap_angle(b - a)


def linspace_angles(a: float, b: float, resolution: float) -> list[float]:
    """Interpolate from angle a to angle b at `resolution`-radian steps, handling wrapping."""
    delta = angle_difference(b, a)

    n = steps_to_cover(abs(delta), resolution, minimum=2)

    return [float(h) for h in np.linspace(a, a + delta, n)]



# ── Spatial helpers ───────────────────────────────────────────────────────────

def center_distance(p1: Position, p2: Position) -> float:
    """Euclidean distance between the position components of two states."""
    x1, y1 = p1
    x2, y2 = p2
    return math.hypot(x2 - x1, y2 - y1)


def direction(p1: Position, p2: Position) -> float:
    """Angle in radians from p1 to p2."""
    x1, y1 = p1
    x2, y2 = p2
    return math.atan2(y2 - y1, x2 - x1)



def linspace_xy(p1: Position, p2: Position, resolution: float
) -> list[Position]:
    """Interpolate from (x1,y1) to (x2,y2) at `resolution`-meter steps."""
    dist = center_distance(p1, p2)
    num_steps = steps_to_cover(dist, resolution, minimum=2)

    ps = np.linspace(p1, p2, num_steps)

    return [Position((x, y)) for x, y in ps]


# ── Trajectory generation ─────────────────────────────────────────────────────

def arc_trajectory(x0: float, y0: float, h0: float,
                   v: float, omega: float,
                   n_steps: int, dt: float) -> np.ndarray:
    """
    Compute a constant-curvature arc trajectory analytically.

    Returns an array of shape (n_steps+1, 3) where each row is [x, y, heading].
    Row 0 is the starting pose; rows 1..n are the states after each timestep.

    For omega ~= 0 (straight line):
        x(i) = x0 + v * cos(h0) * i * dt
        y(i) = y0 + v * sin(h0) * i * dt
        h(i) = h0

    For omega != 0 (circular arc with signed radius R = v/omega):
        h(i) = h0 + omega·i·dt
        x(i) = x0 + R·(sin(h(i)) - sin(h0))
        y(i) = y0 - R·(cos(h(i)) - cos(h0))

    Both branches are handled, so this covers straight lines, circular arcs,
    and in-place rotation primitives (v=0, omega≠0).
    """
    arr = np.empty((n_steps + 1, 3))
    arr[0, 0] = x0
    arr[0, 1] = y0
    arr[0, 2] = h0

    ts = np.arange(1, n_steps + 1, dtype=np.float64) * dt

    if abs(omega) < 1e-9:
        arr[1:, 0] = x0 + v * math.cos(h0) * ts
        arr[1:, 1] = y0 + v * math.sin(h0) * ts
        arr[1:, 2] = h0
    else:
        hs = h0 + omega * ts
        R  = v / omega
        arr[1:, 0] = x0 + R * (np.sin(hs) - math.sin(h0))
        arr[1:, 1] = y0 - R * (np.cos(hs) - math.cos(h0))
        arr[1:, 2] = hs

    return arr


# ── Reeds-Shepp helpers ───────────────────────────────────────────────────────

def rs_path_length(start, goal, turning_radius: float) -> float:
    """Reeds-Shepp path length between two states (ignoring obstacles)."""
    x0, y0, h0, *_ = start
    x1, y1, h1, *_ = goal
    return reeds_shepp.path_length((x0, y0, h0), (x1, y1, h1), turning_radius)


def rs_path_sample(start: Pose, goal: Pose, turning_radius: float, resolution: float = 0.1) -> list[tuple] | None:
    """
    Sample a Reeds-Shepp path between two states.
    Returns the raw sample list from pyReedsShepp, or None if the path is empty.
    """

    raw = reeds_shepp.path_sample(start, goal, turning_radius, resolution)
    return raw if raw else None


def arc_len(p1: Pose, p2: Pose, ang_weight: float) -> float:
    """Weighted pose distance: Euclidean XY distance plus angular difference scaled by ang_weight."""
    x1, y1, h1 = p1
    x2, y2, h2 = p2
    return center_distance(Position((x1, y1)), Position((x2, y2))) + angle_difference(h1, h2) * ang_weight
