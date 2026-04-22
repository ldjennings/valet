"""
Post-processing steps applied to a planned path before it is returned.

smooth_path:   probabilistic shortcutting via random re-connections.
resample_path: uniform arc-length resampling for constant-velocity playback.
"""

import math
import random

from bots import S, Bot, TrailerBot, trajectory_length
from bots.state import PointTurnCapable
from utils import center_distance
from environment.obstacle import ObstacleEnvironment
import config as cfg


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
        iterations: int = 100,
    ) -> list[S]:
    """
    Probabilistic path shortcutting. Repeatedly picks two random indices, attempts
    a direct connection via bot.generate_trajectory, and replaces the span if the
    shortcut is collision-free. Skipped entirely for TrailerBot since
    generate_trajectory doesn't guarantee the trailer heading will match up.
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


def isolate_rotation(path: list[S], start: int) -> int:
    """
    Starting at index `start`, scan forward to find the end of the
    contiguous run of states with no positional movement (pure rotation).

    Returns end_index: the first index with positional movement,
    or len(path) if the rotation extends to the end.
    """
    end = start + 1
    while end < len(path):
        if center_distance(path[end - 1].position(), path[end].position()) >= 1e-12:
            break
        end += 1
    return end


def resample_path(path: list[S], velocity: float = 3.0, angular_vel: float = math.pi / 2) -> list[S]:
    """
    Resample a path at uniform arc-length intervals so the animation
    moves at a constant `velocity` (m/s), assuming DT seconds per frame.

    Pure-rotation segments (no XY movement) are detected and resampled
    via angle_linspace at `angular_vel` rad/s, with carry state reset
    between rotation and movement runs.

    For movement segments, angular velocity is entirely determined by the
    path curvature and the chosen linear velocity — it cannot be controlled
    independently. To reduce angular velocity, either lower `velocity` or
    smooth the path more aggressively (gentler curves).
    """
    if len(path) < 2:
        return list(path)

    step_size = velocity * cfg.DT
    resampled: list[S] = [path[0]]
    carry = 0.0
    i = 1

    while i < len(path):
        prior = path[i - 1]
        current = path[i]

        seg_len = center_distance(prior.position(), current.position())

        if seg_len < 1e-12:
            if isinstance(prior, PointTurnCapable):
                end = isolate_rotation(path, i - 1)
                rot_states = prior.pure_rotation_linspace(path[end - 1], angular_vel * cfg.DT)
                resampled.extend(rot_states[1:])
                i = end
            else:
                i += 1

            carry = 0.0

            continue

        d = step_size - carry
        while d <= seg_len:
            t = d / seg_len
            resampled.append(prior.interpolate(current, t))
            d += step_size
        carry = seg_len - (d - step_size)
        i += 1

    resampled.append(path[-1])

    total_dist = trajectory_length(resampled, 0)
    n = len(resampled)
    avg_spacing = total_dist / (n - 1) if n > 1 else 0.0
    print(
        f"[resample_path] {len(path)} -> {n} states | "
        f"velocity = {velocity:.2f} m/s | "
        f"avg spacing = {avg_spacing:.4f} m | "
        f"total arc length = {total_dist:.2f} m"
    )
    return resampled
