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
    generate_trajectory doesnt guarantee the trailer heading will match up.
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


# def isolate_rotation(path: list[S], start: int) -> int:
#     """
#     Starting at index `start`, scan forward to find the end of the
#     contiguous run of states with no positional movement (pure rotation).

#     Returns end_index: the first index with positional movement,
#     or len(path) if the rotation extends to the end.
#     """
#     end = start + 1
#     while end < len(path):
#         p1 = path[end - 1].position()
#         p2 = path[end].position()
#         if center_distance(p1, p2) >= 1e-12:
#             break
#         end += 1
#     return end


# def resample_rotation(segment: list[S], angular_vel: float) -> list[S]:
#     """
#     Resample a pure-rotation segment (no XY movement) at `angular_vel` rad/s.

#     Each angle component is interpolated independently from the first state's
#     angles to the last state's angles. The number of output steps is determined
#     by whichever angle has the largest total change. Angles that finish rotating
#     earlier hold their final value for the remaining steps.
#     """
#     if len(segment) < 2:
#         return list(segment)

#     p = segment[0].position()
#     a_start = angs(segment[0])
#     a_end = angs(segment[-1])

#     if not a_start or not a_end:
#         return [segment[0], segment[-1]]

#     angular_step = angular_vel * DT
#     # interpolate each angle component independently
#     angle_sequences = [linspace_angles(ai, aj, angular_step) for ai, aj in zip(a_start, a_end)]
#     n_steps = max(len(seq) for seq in angle_sequences)

#     # pad shorter sequences by repeating their final value
#     for seq in angle_sequences:
#         seq.extend([seq[-1]] * (n_steps - len(seq)))

#     state_type = type(segment[0])
#     return [state_type(*p, *angles) for angles in zip(*angle_sequences)]


# def resample_path(path: list[S], velocity: float = 3.0, angular_vel: float = math.pi / 2) -> list[S]:
#     """
#     Resample a path at uniform arc-length intervals so the animation
#     moves at a constant `velocity` (m/s), assuming DT seconds per frame.

#     Pure-rotation segments (no XY movement) are detected and resampled
#     via isolate_angle at `angular_vel` rad/s, with carry state reset
#     between rotation and movement runs.

#     For movement segments, angular velocity is entirely determined by the
#     path curvature and the chosen linear velocity — it cannot be controlled
#     independently. To reduce angular velocity, either lower `velocity` or
#     smooth the path more aggressively (gentler curves).
#     """
#     step_size = velocity * DT

#     if len(path) < 2:
#         return list(path)

#     resampled: list[S] = [path[0]]
#     carry = 0.0
#     i = 1

#     while i < len(path):
#         prior = path[i - 1]
#         current = path[i]
#         p0  = pos(prior)
#         p1  = pos(current)

#         seg_len = center_distance(p0, p1)

#         if seg_len < 1e-12:
#             # pure rotation — isolate the segment, resample each angle independently
#             end = isolate_rotation(path, i - 1)
#             rot_states = resample_rotation(path[i - 1:end], angular_vel)
#             # skip the first state (already in resampled)
#             resampled.extend(rot_states[1:])
#             carry = 0.0
#             i = end
#             continue

#         d = step_size - carry
#         x0, y0 = p0
#         a0 = angs(prior) or []

#         x1, y1 = p1
#         a1 = angs(current) or []

#         while d <= seg_len:
#             t = d / seg_len
#             x = x0 + t * (x1 - x0)
#             y = y0 + t * (y1 - y0)
#             angles = [lerp_angle(ai, aj, t) for ai, aj in zip(a0, a1)]
#             resampled.append(type(path[0])(x, y, *angles))
#             d += step_size

#         carry = seg_len - (d - step_size)
#         i += 1

#     # always include the final state exactly
#     resampled.append(path[-1])

#     # compute stats on the resampled path
#     total_dist = trajectory_length(resampled, 0)
#     n = len(resampled)
#     avg_spacing = total_dist / (n - 1) if n > 1 else 0.0
#     print(f"[resample_path] {len(path)} -> {n} states | "
#           f"velocity = {velocity:.2f} m/s | "
#           f"avg spacing = {avg_spacing:.4f} m | "
#           f"total arc length = {total_dist:.2f} m")

#     return resampled


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

    step_size = velocity * DT
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
                rot_states = prior.pure_rotation_linspace(path[end - 1], angular_vel * DT)
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
