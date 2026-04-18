"""
Kinematics and interface implementations for each robot type.

Defines the Bot Protocol (structural interface) and four concrete implementations:
PointBot, DiffBot, CarBot, and TrailerBot. Each pairs with a corresponding state
type from BotState and implements footprint generation, goal checking, trajectory
generation, and keyboard input handling.
"""

import simulator.config as cfg
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
    point_geom, diff_geom, car_geom, truck_trailer_geom,
    make_point_base, make_centered_rect_base, make_axle_rect_base,
)

import math
from typing import Protocol
from shapely.geometry.base import BaseGeometry
import pygame
import numpy as np

# Simulation timestep in seconds. All step() kinematics use this value.
DT = 1 / 30


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

    def footprint(self, state: S) -> list[BaseGeometry]:
        """Returns the robot's collision geometries at the given state as a list of Shapely objects."""
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

    def propagate(self, state: S, spacing: float, steering_granularity: int) -> list[list[S]]:
        """
        Generate one trajectory per sampled control input.

        Each trajectory is long enough that its XY displacement covers at least
        `spacing` meters, so every primitive lands in a new grid cell.
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

    SPEED           = 1.0
    TERMINAL_RADIUS = 1.0

    def __init__(self):
        self._base = make_point_base()

    def speed(self) -> float: return self.SPEED

    def footprint(self, state: PointState):
        return [point_geom(self._base, state)]

    def is_terminal(self, state: PointState, goal: PointState) -> bool:
        return center_distance(state, goal) < self.TERMINAL_RADIUS

    def heuristic(self, state: PointState, goal: PointState) -> float:
        return math.hypot(goal.x - state.x, goal.y - state.y)

    def at_goal(self, state: PointState, goal: PointState) -> bool:
        return center_distance(state, goal) < cfg.goal_radius_tolerance

    def generate_trajectory(
        self, start: PointState, goal: PointState, resolution: float = 0.1
    ) -> list[PointState] | None:
            dist = np.hypot(goal.x - start.x, goal.y - start.y)
            n_points = max(2, int(np.ceil(dist / resolution)))
            xs = np.linspace(start.x, goal.x, n_points)
            ys = np.linspace(start.y, goal.y, n_points)
            return [PointState(x, y) for x, y in zip(xs, ys)]

    def propagate(self, state: PointState, spacing: float, steering_granularity: int) -> list[list[PointState]]:
        step = self.SPEED * DT
        # For a point bot, every direction displaces equally (step per tick).
        # n_steps so that diagonal displacement per axis >= spacing:
        # diagonal per-axis = n * step / sqrt(2) >= spacing → n >= spacing * sqrt(2) / step
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

    SPEED           = 10.0
    OMEGA_MAX       = math.pi / 2      # rad/s
    TERMINAL_RADIUS = 2.0

    def __init__(self):
        self._base = make_centered_rect_base(cfg.ROBOT_LENGTH_METERS, cfg.ROBOT_WIDTH_METERS)

    def speed(self) -> float: return self.SPEED

    def footprint(self, state: DiffState):
        return [diff_geom(self._base, state)]

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
            center_distance(state, goal) < cfg.goal_radius_tolerance
            and abs(angle_distance_rad(state.heading_rad, goal.heading_rad))
            < cfg.goal_heading_tolerance
        )

    def generate_trajectory(
        self, start: DiffState, goal: DiffState, resolution: float = 0.1
    ) -> list[DiffState] | None:
        states: list[DiffState] = [start]
        current = start
        max_rotate = round(2 * math.pi / (self.OMEGA_MAX * DT)) + 5

        dx = goal.center_x - start.center_x
        dy = goal.center_y - start.center_y
        dist = math.hypot(dx, dy)

        if dist > 1e-3:
            # Phase 1: rotate to face goal
            target = math.atan2(dy, dx)
            for _ in range(max_rotate):
                if abs(angle_distance_rad(current.heading_rad, target)) <= self.OMEGA_MAX * DT:
                    break
                omega = math.copysign(self.OMEGA_MAX, angle_distance_rad(target, current.heading_rad))
                current = current.step(0.0, omega, DT)
                states.append(current)

            # Phase 2: drive straight
            for _ in range(max(1, round(dist / (self.SPEED * DT)))):
                current = current.step(self.SPEED, 0.0, DT)
                states.append(current)

        # Phase 3: rotate to goal heading
        for _ in range(max_rotate):
            if abs(angle_distance_rad(current.heading_rad, goal.heading_rad)) <= self.OMEGA_MAX * DT:
                break
            omega = math.copysign(self.OMEGA_MAX, angle_distance_rad(goal.heading_rad, current.heading_rad))
            current = current.step(0.0, omega, DT)
            states.append(current)

        states.append(goal)
        return states

    def propagate(self, state: DiffState, spacing: float, steering_granularity: int) -> list[list[DiffState]]:
        omegas = [
            self.OMEGA_MAX * i / steering_granularity
            for i in range(-steering_granularity, steering_granularity + 1)
        ]
        trajectories = []
        for v in (self.SPEED, -self.SPEED):
            for omega in omegas:
                n_steps = _n_steps_for_control(spacing, v, omega, DT)
                traj: list[DiffState] = [state]
                for _ in range(n_steps):
                    traj.append(traj[-1].step(v, omega, DT))
                trajectories.append(traj)
        # rotate in place
        n_rot = max(1, math.ceil(spacing / (self.OMEGA_MAX * DT)))
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

    SPEED           = 10.0
    MAX_STEER       = math.radians(45)
    TURNING_RADIUS  = cfg.CAR_WHEELBASE_METERS / math.tan(MAX_STEER)
    TERMINAL_RADIUS = 10.0

    def __init__(self):
        self._base = make_axle_rect_base(cfg.CAR_WHEELBASE_METERS, cfg.CAR_LENGTH_METERS, cfg.CAR_WIDTH_METERS)

    def speed(self) -> float: return self.SPEED

    def footprint(self, state: CarState):
        return [car_geom(self._base, state)]

    def is_terminal(self, state: CarState, goal: CarState) -> bool:
        return center_distance(state, goal) < self.TERMINAL_RADIUS

    def heuristic(self, state: CarState, goal: CarState) -> float:
        q0 = (state.rear_axle_x, state.rear_axle_y, state.heading_rad)
        q1 = (goal.rear_axle_x,  goal.rear_axle_y,  goal.heading_rad)
        return reeds_shepp.path_length(q0, q1, self.TURNING_RADIUS)

    def at_goal(self, state: CarState, goal: CarState) -> bool:
        return (
            center_distance(state, goal) < cfg.goal_radius_tolerance
            and abs(angle_distance_rad(state.heading_rad, goal.heading_rad))
            < cfg.goal_heading_tolerance
        )

    def generate_trajectory(self, start: CarState, goal: CarState, resolution: float = 0.1) -> list[CarState] | None:
        q0  = (start.rear_axle_x, start.rear_axle_y, start.heading_rad)
        q1  = (goal.rear_axle_x,  goal.rear_axle_y,  goal.heading_rad)
        raw = reeds_shepp.path_sample(q0, q1, self.TURNING_RADIUS, resolution)
        return [CarState(r[0], r[1], r[2]) for r in raw] if raw else None

    def propagate(self, state: CarState, spacing: float, steering_granularity: int) -> list[list[CarState]]:
        deltas = [
            self.MAX_STEER * i / steering_granularity
            for i in range(-steering_granularity, steering_granularity + 1)
        ]
        trajectories = []
        for v in (self.SPEED, -self.SPEED):
            for delta in deltas:
                omega = v * math.tan(delta) / cfg.CAR_WHEELBASE_METERS if delta != 0 else 0.0
                n_steps = _n_steps_for_control(spacing, v, omega, DT)
                traj: list[CarState] = [state]
                for _ in range(n_steps):
                    traj.append(traj[-1].step(v, delta, cfg.CAR_WHEELBASE_METERS, DT))
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

        return state.step(v, delta, cfg.CAR_WHEELBASE_METERS, DT)

    def make_state(self, x: float, y: float, h: float = 0, t:float = 0) -> CarState:
        return CarState(x, y, h)


class TrailerBot:
    """
    Truck-and-trailer system. The truck uses Ackermann steering; the trailer follows
    passively via hitch kinematics. Goal checking requires both truck and trailer headings
    to be within tolerance.
    """

    SPEED           = 7.5
    MAX_STEER       = math.radians(35)
    TURNING_RADIUS  = cfg.TRUCK_WHEELBASE_METERS / math.tan(MAX_STEER)
    TERMINAL_RADIUS = 15.0

    def __init__(self):
        self._truck_base = make_axle_rect_base(cfg.TRUCK_WHEELBASE_METERS, cfg.TRUCK_LENGTH_METERS, cfg.TRUCK_WIDTH_METERS)
        self._trailer_base = make_centered_rect_base(cfg.TRAILER_LENGTH_METERS, cfg.TRAILER_WIDTH_METERS)

    def speed(self) -> float: return self.SPEED

    def footprint(self, state: TrailerState):
        return truck_trailer_geom(self._truck_base, self._trailer_base, state)

    def is_terminal(self, state: TrailerState, goal: TrailerState) -> bool:
        return center_distance(state, goal) < self.TERMINAL_RADIUS

    def heuristic(self, state: TrailerState, goal: TrailerState) -> float:
        # RS on the truck state only — trailer heading is handled by generate_trajectory
        q0 = (state.rear_axle_x, state.rear_axle_y, state.heading_rad)
        q1 = (goal.rear_axle_x,  goal.rear_axle_y,  goal.heading_rad)
        return reeds_shepp.path_length(q0, q1, self.TURNING_RADIUS)

    def at_goal(self, state: TrailerState, goal: TrailerState) -> bool:
        return (
            center_distance(state, goal) < cfg.goal_radius_tolerance
            and abs(angle_distance_rad(state.heading_rad, goal.heading_rad))
            < cfg.goal_heading_tolerance
            and abs(angle_distance_rad(state.trailer_heading_rad, goal.trailer_heading_rad))
            < cfg.trailer_heading_tolerance
        )

    def generate_trajectory(
        self, start: TrailerState, goal: TrailerState, resolution: float = 0.1
    ) -> list[TrailerState] | None:
        q0  = (start.rear_axle_x, start.rear_axle_y, start.heading_rad)
        q1  = (goal.rear_axle_x,  goal.rear_axle_y,  goal.heading_rad)
        raw = reeds_shepp.path_sample(q0, q1, self.TURNING_RADIUS, resolution)
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
            phi += length_sign * math.sin(angle_distance_rad(theta, phi)) * ds / cfg.TRUCK_HITCH_TO_TRAILER_AXLE

            if abs(angle_distance_rad(theta, phi)) > JACKKNIFE_LIMIT:
                return None

            states.append(TrailerState(x, y, theta, phi))

        return states

    def propagate(self, state: TrailerState, spacing: float, steering_granularity: int) -> list[list[TrailerState]]:
        deltas = [
            self.MAX_STEER * i / steering_granularity
            for i in range(-steering_granularity, steering_granularity + 1)
        ]
        trajectories = []
        for v in (self.SPEED, -self.SPEED):
            for delta in deltas:
                omega = v * math.tan(delta) / cfg.TRUCK_WHEELBASE_METERS if delta != 0 else 0.0
                n_steps = _n_steps_for_control(spacing, v, omega, DT)
                traj: list[TrailerState] = [state]
                for _ in range(n_steps):
                    traj.append(traj[-1].step(v, delta, cfg.TRUCK_WHEELBASE_METERS, cfg.TRUCK_HITCH_TO_TRAILER_AXLE, DT))
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

        next_state = state.step(
            v, delta, cfg.TRUCK_WHEELBASE_METERS, cfg.TRUCK_HITCH_TO_TRAILER_AXLE, DT
        )
        # if abs(next_state.trailer_heading) > cfg.MAX_HITCH_ANGLE: # potential check for jacknifed trailer
        #     return state
        return next_state

    def make_state(self, x: float, y: float, h: float = 0, t:float = 0) -> TrailerState:
        return TrailerState(x, y, h, t)
