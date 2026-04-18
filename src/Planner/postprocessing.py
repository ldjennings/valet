"""
Post-processing steps applied to a planned path before it is returned.

smooth_path:   probabilistic shortcutting via random re-connections.
resample_path: uniform arc-length resampling for constant-velocity playback.
"""

import math
import random

from Bots import S, Bot, TrailerBot
from environment.obstacle import ObstacleEnvironment


def validate_shortcut(
    obstacles: ObstacleEnvironment, bot: Bot, path: list[S],
) -> bool:
    """Quick collision check used by smooth_path to validate candidate shortcuts."""
    for state in path:
        if not obstacles.is_valid_state(bot.footprint(state, approximate=True)):
            return False
    return True


def smooth_path(
        path: list[S],
        bot: Bot,
        obstacles: ObstacleEnvironment,
        iterations: int = 200,
    ) -> list[S]:
    """
    Probabilistic path shortcutting. Repeatedly picks two random indices, attempts
    a direct connection via bot.generate_trajectory, and replaces the span if the
    shortcut is collision-free. Skipped entirely for TrailerBot since
    generate_trajectory ignores the trailer heading.
    """
    if isinstance(bot, TrailerBot):
        return path

    path = list(path)
    initial_len = len(path)
    shortcuts_applied = 0

    for i in range(iterations):
        if len(path) < 3:
            print(f"[smooth_path] path too short to shorten further, stopping at iteration {i}")
            break

        a = random.randint(0, len(path) - 2)
        b = random.randint(a + 1, len(path) - 1)

        shortcut = bot.generate_trajectory(path[a], path[b])
        if shortcut is None:
            continue

        if validate_shortcut(obstacles, bot, shortcut):
            path = path[:a] + shortcut + path[b + 1:]
            shortcuts_applied += 1

    print(f"[smooth_path] {iterations} iterations | {shortcuts_applied} shortcuts applied | "
          f"{initial_len} -> {len(path)} states")

    return path


DT = 1 / 30  # assumed simulation timestep for velocity calculations


def _wrapped_delta(a: float, b: float) -> float:
    """Signed angular difference from a to b, wrapped to [-π, π]."""
    return (b - a + math.pi) % (2 * math.pi) - math.pi


def isolate_rotation(path: list[S], start: int) -> int:
    """
    Starting at index `start`, scan forward to find the end of the
    contiguous run of states with no positional movement (pure rotation).

    Returns end_index: the first index with positional movement,
    or len(path) if the rotation extends to the end.
    """
    end = start + 1
    while end < len(path):
        x0, y0, *_ = path[end - 1]
        x1, y1, *_ = path[end]
        if math.hypot(x1 - x0, y1 - y0) >= 1e-12:
            break
        end += 1
    return end


def resample_rotation(segment: list[S], angular_vel: float) -> list[S]:
    """
    Resample a pure-rotation segment (no XY movement) at `angular_vel` rad/s.

    Each angle component is interpolated independently from the first state's
    angles to the last state's angles. The number of output steps is determined
    by whichever angle has the largest total change. Angles that finish rotating
    earlier hold their final value for the remaining steps.
    """
    if len(segment) < 2:
        return list(segment)

    x, y, *a_start = segment[0]
    _, _, *a_end = segment[-1]

    if not a_start:
        return [segment[0], segment[-1]]

    angular_step = angular_vel * DT
    deltas = [_wrapped_delta(ai, aj) for ai, aj in zip(a_start, a_end)]

    # per-angle step counts (how many frames each angle needs independently)
    per_angle_steps = [max(1, math.ceil(abs(d) / angular_step)) for d in deltas]
    n_steps = max(per_angle_steps)

    state_type = type(segment[0])
    resampled: list[S] = []
    for k in range(n_steps + 1):
        angles = []
        for ai, di, ni in zip(a_start, deltas, per_angle_steps):
            t = min(k / ni, 1.0)
            angles.append(ai + t * di)
        resampled.append(state_type(x, y, *angles))

    return resampled


def resample_path(path: list[S], velocity: float = 3.0, angular_vel: float = math.pi / 2) -> list[S]:
    """
    Resample a path at uniform arc-length intervals so the animation
    moves at a constant `velocity` (m/s), assuming DT seconds per frame.

    Pure-rotation segments (no XY movement) are detected and resampled
    via isolate_angle at `angular_vel` rad/s, with carry state reset
    between rotation and movement runs.

    For movement segments, angular velocity is entirely determined by the
    path curvature and the chosen linear velocity — it cannot be controlled
    independently. To reduce angular velocity, either lower `velocity` or
    smooth the path more aggressively (gentler curves).
    """
    step_size = velocity * DT

    if len(path) < 2:
        return list(path)

    resampled: list[S] = [path[0]]
    carry = 0.0
    i = 1

    while i < len(path):
        x0, y0, *_ = path[i - 1]
        x1, y1, *_ = path[i]
        seg_len = math.hypot(x1 - x0, y1 - y0)

        if seg_len < 1e-12:
            # pure rotation — isolate the segment, resample each angle independently
            end = isolate_rotation(path, i - 1)
            rot_states = resample_rotation(path[i - 1:end], angular_vel)
            # skip the first state (already in resampled)
            resampled.extend(rot_states[1:])
            carry = 0.0
            i = end
            continue

        d = step_size - carry
        x0, y0, *a0 = path[i - 1]
        x1, y1, *a1 = path[i]
        while d <= seg_len:
            t = d / seg_len
            x = x0 + t * (x1 - x0)
            y = y0 + t * (y1 - y0)
            angles = [
                ai + t * _wrapped_delta(ai, aj)
                for ai, aj in zip(a0, a1)
            ]
            resampled.append(type(path[0])(x, y, *angles))
            d += step_size

        carry = seg_len - (d - step_size)
        i += 1

    # always include the final state exactly
    resampled.append(path[-1])

    # compute stats on the resampled path
    total_dist = 0.0
    total_heading_change = 0.0
    has_heading = False
    for j in range(1, len(resampled)):
        x0, y0, *a0 = resampled[j - 1]
        x1, y1, *a1 = resampled[j]
        total_dist += math.hypot(x1 - x0, y1 - y0)
        if a0 and a1:
            has_heading = True
            total_heading_change += abs((a1[0] - a0[0] + math.pi) % (2 * math.pi) - math.pi)

    n = len(resampled)
    avg_spacing = total_dist / (n - 1) if n > 1 else 0.0
    print(f"[resample_path] {len(path)} -> {n} states | "
          f"velocity = {velocity:.2f} m/s | "
          f"avg spacing = {avg_spacing:.4f} m | "
          f"total arc length = {total_dist:.2f} m"
          + (f" | avg angular vel = {total_heading_change / ((n - 1) * DT):.2f} rad/s"
             if has_heading and n > 1 else ""))

    return resampled
