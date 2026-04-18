"""
Kinematics and interface implementations for each robot type.

Defines the Bot Protocol (structural interface) and four concrete implementations:
PointBot, DiffBot, CarBot, and TrailerBot. Each pairs with a corresponding state
type from BotState and implements footprint generation, goal checking, trajectory
generation, and keyboard input handling.

Robot dimensions and goal tolerances are constructor parameters with sensible
defaults, so the Bots package has no dependency on simulator.config.
"""

import reeds_shepp
from Bots.BotState import (
    S,
    PointState,
    DiffState,
    CarState,
    TrailerState,
    center_distance,
    angle_distance_rad,
)
from Bots.geometry_helpers import (
    point_geom, place, cached_geom, truck_trailer_geom,
    make_point_base, make_centered_rect_base, make_axle_rect_base,
    build_heading_cache,
)

import math
from typing import Protocol
from shapely.geometry.base import BaseGeometry
import pygame
import numpy as np

# Simulation timestep in seconds. All step() kinematics use this value.
DT = 1 / 30


STANDARD_SPEED = 5.0


def _n_steps_for_control(spacing: float, v: float, omega: float, dt: float) -> int:
    """
    Compute how many simulation steps a (v, omega) control pair needs so that
    the XY displacement is at least `spacing`.

    Straight (omega≈0): displacement = |v| * n * dt → n = spacing / (|v| * dt)
    Arc (omega≠0):      radius R = |v/omega|, displacement after angle θ = n*|omega|*dt
                         is R * sqrt(2 - 2cos(θ)).  Solve for θ then n.
    """
    if abs(omega) < 1e-9:
        return max(1, math.ceil(spacing / (abs(v) * dt)))

    R = abs(v / omega)
    # R * sqrt(2 - 2cos(θ)) = spacing → cos(θ) = 1 - spacing² / (2R²)
    cos_theta = 1.0 - (spacing ** 2) / (2.0 * R ** 2)
    if cos_theta < -1.0:
        # spacing > 2R (diameter), need a half-turn
        theta = math.pi
    else:
        theta = math.acos(cos_theta)
    return max(1, math.ceil(theta / (abs(omega) * dt))) + 3


class Bot(Protocol[S]):
    """
    Structural protocol defining the interface all bot types must satisfy.
    Generic over S (the state type) — a Bot[PointState] only accepts PointState,
    a Bot[DiffState] only accepts DiffState, etc.

    Uses Protocol rather than ABC so implementing classes don't need to explicitly
    inherit from Bot — the type checker verifies the interface is satisfied
    structurally based on method signatures alone.
    """

    def speed(self) -> float:
        """Planning speed in m/s. Used by propagated_primitives to scale arc length to grid spacing."""
        ...

    def footprint(self, state: S, approximate: bool = False) -> list[BaseGeometry]:
        """Returns the robot's collision geometries at the given state as a list of Shapely objects.
        If approximate=True, uses a cached rotation (snapped to nearest 5°) for speed."""
        ...

    def generate_trajectory(self, start: S, goal: S, resolution: float = 0.1) -> list[S] | None:
        """
        Generate a kinematically correct trajectory from start to goal.

        Returns a list of states at approximately `resolution` meter intervals,
        or None if no feasible connection exists (e.g. non-holonomic bots that
        lack a closed-form BVP solution).
        """
        ...

    def is_terminal(self, state: S, goal: S) -> bool:
        """
        Returns True if state is close enough to goal to fire the terminal
        connection (positional check only — heading is handled by generate_trajectory).
        Each bot sets its own terminal_radius based on kinematic constraints.
        """
        ...

    def heuristic(self, state: S, goal: S) -> float:
        """
        Admissible cost-to-go estimate from state to goal.
        Non-holonomic bots use a Reeds-Shepp path length (ignoring obstacles);
        holonomic bots use Euclidean distance.
        """
        ...

    def at_goal(self, state: S, goal: S) -> bool:
        """
        Returns True if state is close enough to goal state to end simulation.
        Checks position for all bots, plus heading for diff/car/trailer, plus
        hitch angle for trailer.
        """
        ...

    def handle_input(self, state: S, speed: float) -> S:
        """
        Applies keyboard input to produce the next state. Used in manual mode
        for debugging kinematics before trusting the planner.
        Kinematically correct — uses the same step() functions as primitive generation.
        """
        ...

    def propagate(self, state: S, spacing: float, angular_spacing: float, steering_granularity: int) -> list[list[S]]:
        """
        Generate one trajectory per sampled control input.

        Each trajectory is long enough that its XY displacement covers at least
        `spacing` meters, so every primitive lands in a new grid cell.
        Turning primitives are additionally lengthened so that heading changes by
        at least `angular_spacing` radians, ensuring they land in a distinct heading bin.
        `steering_granularity` controls how many steering/omega values are
        sampled between 0 and max on each side (e.g. 3 → [-max, -2/3, -1/3, 0, 1/3, 2/3, max]).
        """
        ...

    def make_state(self, x: float, y: float, h: float = 0, t: float = 0) -> S:
        """
        Construct a state of the appropriate type from raw coordinates.
        h is the heading in radians; t is the trailer heading in radians (TrailerBot only).
        Provides a uniform way to create states without knowing the concrete type.
        """
        ...


class PointBot:
    """Holonomic point robot. Can move in any direction; no heading or turning constraints."""

    # SPEED           = 7.5
    SPEED           = STANDARD_SPEED
    TERMINAL_RADIUS = 10.0

    def __init__(self, goal_radius_tol: float = 0.25):
        self.goal_radius_tol = goal_radius_tol
        self._base = make_point_base()

    def speed(self) -> float: return self.SPEED

    def footprint(self, state: PointState, approximate: bool = False) -> list[BaseGeometry]:
        return [point_geom(self._base, state)]

    def is_terminal(self, state: PointState, goal: PointState) -> bool:
        return center_distance(state, goal) < self.TERMINAL_RADIUS

    def heuristic(self, state: PointState, goal: PointState) -> float:
        return math.hypot(goal.x - state.x, goal.y - state.y)

    def at_goal(self, state: PointState, goal: PointState) -> bool:
        return center_distance(state, goal) < self.goal_radius_tol

    def generate_trajectory(
        self, start: PointState, goal: PointState, resolution: float = 0.1
    ) -> list[PointState] | None:
            dist = np.hypot(goal.x - start.x, goal.y - start.y)
            n_points = max(2, int(np.ceil(dist / resolution)))
            xs = np.linspace(start.x, goal.x, n_points)
            ys = np.linspace(start.y, goal.y, n_points)
            return [PointState(x, y) for x, y in zip(xs, ys)]

    def propagate(self, state: PointState, spacing: float, angular_spacing: float, steering_granularity: int) -> list[list[PointState]]:
        step = self.SPEED * DT
        # For a point bot, every direction displaces equally (step per tick).
        # n_steps so that diagonal displacement per axis >= spacing:
        # diagonal per-axis = n * step / sqrt(2) >= spacing → n >= spacing * sqrt(2) / step
        # angular_spacing is unused — point bot has no heading.
        n_steps = max(1, math.ceil(spacing * math.sqrt(2) / step))
        trajectories = []
        for dx, dy in [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]:
            norm = math.hypot(dx, dy)
            sx, sy = dx / norm * step, dy / norm * step
            traj: list[PointState] = [state]
            for _ in range(n_steps):
                traj.append(traj[-1].translate(sx, sy))
            trajectories.append(traj)
        return trajectories

    def handle_input(self, state: PointState, speed: float) -> PointState:
        keymap = {
            pygame.K_LEFT: (-0.5, 0),
            pygame.K_RIGHT: (0.5, 0),
            pygame.K_UP: (0, -0.5),
            pygame.K_DOWN: (0, 0.5),
        }

        keys = pygame.key.get_pressed()

        new_state = state

        for k, delta in keymap.items():
            if keys[k]:
                new_state = new_state.translate(delta[0], delta[1])

        return new_state

    def make_state(self, x: float, y: float, h: float = 0, t:float = 0) -> PointState:
        return PointState(x, y)


class DiffBot:
    """Differential drive robot. Can rotate in place; forward/backward and in-place turning."""

    # SPEED           = 7.5
    SPEED           = STANDARD_SPEED
    OMEGA_MAX       = math.pi / 2      # rad/s
    TERMINAL_RADIUS = 2.0

    def __init__(
        self,
        length: float = 0.7,
        width: float = 0.57,
        goal_radius_tol: float = 0.25,
        goal_heading_tol: float = math.pi / 12,
    ):
        self.goal_radius_tol = goal_radius_tol
        self.goal_heading_tol = goal_heading_tol
        self._base = make_centered_rect_base(length, width)
        self._cache = build_heading_cache(self._base)

    def speed(self) -> float: return self.SPEED

    def footprint(self, state: DiffState, approximate: bool = False) -> list[BaseGeometry]:
        if approximate:
            return [cached_geom(self._cache, state.center_x, state.center_y, state.heading_rad)]
        return [place(self._base, state.center_x, state.center_y, state.heading_rad)]

    def is_terminal(self, state: DiffState, goal: DiffState) -> bool:
        return center_distance(state, goal) < self.TERMINAL_RADIUS

    def heuristic(self, state: DiffState, goal: DiffState) -> float:
        dx   = goal.center_x - state.center_x
        dy   = goal.center_y - state.center_y
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return 0.0
        # penalise heading error relative to goal direction — tightens the heuristic
        # without this, all headings at the same position get equal priority
        goal_dir      = math.atan2(dy, dx)
        heading_error = abs(angle_distance_rad(state.heading_rad, goal_dir))
        return dist + heading_error * 0.5  # 0.5 matches ROTATION_COST_WEIGHT in primitives.py

    def at_goal(self, state: DiffState, goal: DiffState) -> bool:
        return (
            center_distance(state, goal) < self.goal_radius_tol
            and abs(angle_distance_rad(state.heading_rad, goal.heading_rad))
            < self.goal_heading_tol
        )

    def generate_trajectory(
        self, start: DiffState, goal: DiffState, resolution: float = 0.1
    ) -> list[DiffState] | None:
        """
        Connect start to goal via rotate-drive-rotate, interpolated at
        `resolution`-sized steps using linspace. No discrete stepping,
        so no accumulation error or overshoot.
        """
        states: list[DiffState] = []
        sx, sy, sh = start.center_x, start.center_y, start.heading_rad
        gx, gy, gh = goal.center_x, goal.center_y, goal.heading_rad
        dist = np.hypot(gx - sx, gy - sy)

        if dist > 1e-3:
            # Phase 1: rotate to face goal
            target = math.atan2(gy - sy, gx - sx)
            delta1 = angle_distance_rad(target, sh)
            n1 = max(2, math.ceil(abs(delta1) / resolution) + 1)
            for h in np.linspace(sh, sh + delta1, n1):
                states.append(DiffState(sx, sy, float(h)))

            # Phase 2: drive straight
            n2 = max(2, math.ceil(dist / resolution) + 1)
            for x, y in zip(np.linspace(sx, gx, n2), np.linspace(sy, gy, n2)):
                states.append(DiffState(float(x), float(y), target))

            heading_after_drive = target
        else:
            states.append(start)
            heading_after_drive = sh

        # Phase 3: rotate to goal heading
        delta3 = angle_distance_rad(gh, heading_after_drive)
        if abs(delta3) > 1e-6:
            n3 = max(2, math.ceil(abs(delta3) / resolution) + 1)
            for h in np.linspace(heading_after_drive, heading_after_drive + delta3, n3):
                states.append(DiffState(gx, gy, float(h)))

        return states

    def propagate(self, state: DiffState, spacing: float, angular_spacing: float, steering_granularity: int) -> list[list[DiffState]]:
        omegas = [
            self.OMEGA_MAX * i / steering_granularity
            for i in range(-steering_granularity, steering_granularity + 1)
        ]
        trajectories = []
        for v in (self.SPEED, -self.SPEED):
            for omega in omegas:
                n_xy = _n_steps_for_control(spacing, v, omega, DT)
                # for turning primitives, ensure heading changes by at least one angular bin
                if abs(omega) > 1e-9:
                    n_heading = math.ceil(angular_spacing / (abs(omega) * DT))
                    n_steps = max(n_xy, n_heading)
                else:
                    n_steps = n_xy
                traj: list[DiffState] = [state]
                for _ in range(n_steps):
                    traj.append(traj[-1].step(v, omega, DT))
                trajectories.append(traj)
        # rotate in place
        n_rot = max(1, math.ceil(angular_spacing / (self.OMEGA_MAX * DT)))
        for omega in (-self.OMEGA_MAX, self.OMEGA_MAX):
            traj = [state]
            for _ in range(n_rot):
                traj.append(traj[-1].step(0.0, omega, DT))
            trajectories.append(traj)
        return trajectories

    def handle_input(self, state: DiffState, speed: float) -> DiffState:
        v, omega = 0.0, 0.0

        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP] and not keys[pygame.K_DOWN]:
            v = speed
        elif keys[pygame.K_DOWN] and not keys[pygame.K_UP]:
            v = -speed
        if keys[pygame.K_LEFT] and not keys[pygame.K_RIGHT]:
            omega = -speed
        elif keys[pygame.K_RIGHT] and not keys[pygame.K_LEFT]:
            omega = speed
        return state.step(v, omega, DT)

    def make_state(self, x: float, y: float, h: float = 0, t:float = 0) -> DiffState:
        return DiffState(x, y, h)

class CarBot:
    """Ackermann steering (car-like) robot. Non-holonomic; minimum turning radius determined by MAX_STEER."""

    # SPEED           = 7.5
    SPEED           = STANDARD_SPEED
    MAX_STEER       = math.radians(45)
    TERMINAL_RADIUS = 10.0

    def __init__(
        self,
        wheelbase: float = 2.8,
        length: float = 5.2,
        width: float = 1.8,
        goal_radius_tol: float = 0.25,
        goal_heading_tol: float = math.pi / 12,
    ):
        self.wheelbase = wheelbase
        self.turning_radius = wheelbase / math.tan(self.MAX_STEER)
        self.goal_radius_tol = goal_radius_tol
        self.goal_heading_tol = goal_heading_tol
        self._base = make_axle_rect_base(wheelbase, length, width)
        self._cache = build_heading_cache(self._base)

    def speed(self) -> float: return self.SPEED

    def footprint(self, state: CarState, approximate: bool = False) -> list[BaseGeometry]:
        if approximate:
            return [cached_geom(self._cache, state.rear_axle_x, state.rear_axle_y, state.heading_rad)]
        return [place(self._base, state.rear_axle_x, state.rear_axle_y, state.heading_rad)]

    def is_terminal(self, state: CarState, goal: CarState) -> bool:
        return center_distance(state, goal) < self.TERMINAL_RADIUS

    def heuristic(self, state: CarState, goal: CarState) -> float:
        # TODO: SWITCH BACK TO REEDS_SHEPP ONCE DONE EXPERIMENTING

        # q0 = (state.rear_axle_x, state.rear_axle_y, state.heading_rad)
        # q1 = (goal.rear_axle_x,  goal.rear_axle_y,  goal.heading_rad)
        # return reeds_shepp.path_length(q0, q1, self.turning_radius)
        return center_distance(state, goal)

    def at_goal(self, state: CarState, goal: CarState) -> bool:
        return (
            center_distance(state, goal) < self.goal_radius_tol
            and abs(angle_distance_rad(state.heading_rad, goal.heading_rad))
            < self.goal_heading_tol
        )

    def generate_trajectory(self, start: CarState, goal: CarState, resolution: float = 0.1) -> list[CarState] | None:
        q0  = (start.rear_axle_x, start.rear_axle_y, start.heading_rad)
        q1  = (goal.rear_axle_x,  goal.rear_axle_y,  goal.heading_rad)
        raw = reeds_shepp.path_sample(q0, q1, self.turning_radius, resolution)
        if not raw:
            return None
        path = [CarState(r[0], r[1], r[2]) for r in raw]
        path.append(goal)
        return path

    def propagate(self, state: CarState, spacing: float, angular_spacing: float, steering_granularity: int) -> list[list[CarState]]:
        deltas = [
            self.MAX_STEER * i / steering_granularity
            for i in range(-steering_granularity, steering_granularity + 1)
        ]
        trajectories = []
        for v in (self.SPEED, -self.SPEED):
            for delta in deltas:
                omega = v * math.tan(delta) / self.wheelbase if delta != 0 else 0.0
                n_xy = _n_steps_for_control(spacing, v, omega, DT)
                if abs(omega) > 1e-9:
                    n_heading = math.ceil(angular_spacing / (abs(omega) * DT))
                    n_steps = max(n_xy, n_heading)
                else:
                    n_steps = n_xy
                traj: list[CarState] = [state]
                for _ in range(n_steps):
                    traj.append(traj[-1].step(v, delta, self.wheelbase, DT))
                trajectories.append(traj)
        return trajectories

    def handle_input(self, state: CarState, speed: float) -> CarState:
        v, delta = 0.0, 0.0

        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP] and not keys[pygame.K_DOWN]:
            v = speed
        elif keys[pygame.K_DOWN] and not keys[pygame.K_UP]:
            v = -speed

        if keys[pygame.K_LEFT] and not keys[pygame.K_RIGHT]:
            delta = -self.MAX_STEER
        elif keys[pygame.K_RIGHT] and not keys[pygame.K_LEFT]:
            delta = self.MAX_STEER

        return state.step(v, delta, self.wheelbase, DT)

    def make_state(self, x: float, y: float, h: float = 0, t:float = 0) -> CarState:
        return CarState(x, y, h)


class TrailerBot:
    """
    Truck-and-trailer system. The truck uses Ackermann steering; the trailer follows
    passively via hitch kinematics. Goal checking requires both truck and trailer headings
    to be within tolerance.
    """

    # SPEED           = 7.5
    SPEED           = STANDARD_SPEED
    MAX_STEER       = math.radians(35)
    TERMINAL_RADIUS = 15.0

    def __init__(
        self,
        wheelbase: float = 3.4,
        length: float = 5.4,
        width: float = 2.0,
        hitch_distance: float = 5.0,
        trailer_length: float = 4.5,
        trailer_width: float = 2.0,
        goal_radius_tol: float = 0.25,
        goal_heading_tol: float = math.pi / 12,
        trailer_heading_tol: float = math.pi / 32,
    ):
        self.wheelbase = wheelbase
        self.hitch_distance = hitch_distance
        self.turning_radius = wheelbase / math.tan(self.MAX_STEER)
        self.goal_radius_tol = goal_radius_tol
        self.goal_heading_tol = goal_heading_tol
        self.trailer_heading_tol = trailer_heading_tol
        self._truck_base = make_axle_rect_base(wheelbase, length, width)
        self._trailer_base = make_centered_rect_base(trailer_length, trailer_width)
        self._truck_cache = build_heading_cache(self._truck_base)
        self._trailer_cache = build_heading_cache(self._trailer_base)

    def speed(self) -> float: return self.SPEED

    def footprint(self, state: TrailerState, approximate: bool = False) -> list[BaseGeometry]:
        return truck_trailer_geom(
            self._truck_base, self._trailer_base, state, self.hitch_distance,
            self._truck_cache, self._trailer_cache, approximate,
        )

    def is_terminal(self, state: TrailerState, goal: TrailerState) -> bool:
        return center_distance(state, goal) < self.TERMINAL_RADIUS

    def heuristic(self, state: TrailerState, goal: TrailerState) -> float:
        # RS on the truck state only — trailer heading is handled by generate_trajectory
        # q0 = (state.rear_axle_x, state.rear_axle_y, state.heading_rad)
        # q1 = (goal.rear_axle_x,  goal.rear_axle_y,  goal.heading_rad)
        # return reeds_shepp.path_length(q0, q1, self.turning_radius)
        return center_distance(state, goal)

    def at_goal(self, state: TrailerState, goal: TrailerState) -> bool:
        return (
            center_distance(state, goal) < self.goal_radius_tol
            and abs(angle_distance_rad(state.heading_rad, goal.heading_rad))
            < self.goal_heading_tol
            and abs(angle_distance_rad(state.trailer_heading_rad, goal.trailer_heading_rad))
            < self.trailer_heading_tol
        )

    def generate_trajectory(
        self, start: TrailerState, goal: TrailerState, resolution: float = 0.1
    ) -> list[TrailerState] | None:
        q0  = (start.rear_axle_x, start.rear_axle_y, start.heading_rad)
        q1  = (goal.rear_axle_x,  goal.rear_axle_y,  goal.heading_rad)
        raw = reeds_shepp.path_sample(q0, q1, self.turning_radius, resolution)
        if not raw:
            return None

        JACKKNIFE_LIMIT = math.pi / 2

        phi    = start.trailer_heading_rad
        states: list[TrailerState] = [start]

        for i in range(1, len(raw)):
            x, y, theta  = raw[i][0], raw[i][1], raw[i][2]
            length_sign  = math.copysign(1.0, raw[i][4])  # s[4] = signed segment length; sign = direction
            ds           = math.hypot(x - raw[i-1][0], y - raw[i-1][1])

            # integrate trailer heading along arc (derived from TrailerState.step kinematics)
            phi += length_sign * math.sin(angle_distance_rad(theta, phi)) * ds / self.hitch_distance

            if abs(angle_distance_rad(theta, phi)) > JACKKNIFE_LIMIT:
                return None

            states.append(TrailerState(x, y, theta, phi))

        return states

    def propagate(self, state: TrailerState, spacing: float, angular_spacing: float, steering_granularity: int) -> list[list[TrailerState]]:
        deltas = [
            self.MAX_STEER * i / steering_granularity
            for i in range(-steering_granularity, steering_granularity + 1)
        ]
        trajectories = []
        for v in (self.SPEED, -self.SPEED):
            for delta in deltas:
                omega = v * math.tan(delta) / self.wheelbase if delta != 0 else 0.0
                n_xy = _n_steps_for_control(spacing, v, omega, DT)
                if abs(omega) > 1e-9:
                    n_heading = math.ceil(angular_spacing / (abs(omega) * DT))
                    n_steps = max(n_xy, n_heading)
                else:
                    n_steps = n_xy
                traj: list[TrailerState] = [state]
                for _ in range(n_steps):
                    traj.append(traj[-1].step(v, delta, self.wheelbase, self.hitch_distance, DT))
                trajectories.append(traj)
        return trajectories

    def handle_input(self, state: TrailerState, speed: float) -> TrailerState:
        v, delta = 0.0, 0.0

        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP] and not keys[pygame.K_DOWN]:
            v = speed
        elif keys[pygame.K_DOWN] and not keys[pygame.K_UP]:
            v = -speed

        if keys[pygame.K_LEFT] and not keys[pygame.K_RIGHT]:
            delta = -self.MAX_STEER
        elif keys[pygame.K_RIGHT] and not keys[pygame.K_LEFT]:
            delta = self.MAX_STEER

        return state.step(v, delta, self.wheelbase, self.hitch_distance, DT)

    def make_state(self, x: float, y: float, h: float = 0, t:float = 0) -> TrailerState:
        return TrailerState(x, y, h, t)
