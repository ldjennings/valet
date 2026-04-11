from Planner.LatticeConfig import LatticeConfig
import simulator.config as cfg
from Bots.BotState import (
    S,
    PointState,
    DiffState,
    CarState,
    TrailerState,
    center_distance,
    angle_distance_rad,
)
from Bots.geometry_helpers import point_geom, diff_geom, car_geom, truck_trailer_geom

import math
from typing import Protocol
from shapely.geometry.base import BaseGeometry
import pygame
import numpy as np

# just a constant for now
DT = 1 / 30


class Bot(Protocol[S]):
    """
    Structural protocol defining the interface all bot types must satisfy.
    Generic over S (the state type) — a Bot[PointState] only accepts PointState,
    a Bot[DiffState] only accepts DiffState, etc.

    Uses Protocol rather than ABC so implementing classes don't need to explicitly
    inherit from Bot — the type checker verifies the interface is satisfied
    structurally based on method signatures alone.
    """

    def footprint(self, state: S) -> BaseGeometry:
        """Returns the robot's collision geometry at the given state as a Shapely object."""
        ...

    def generate_trajectory(self, start: S, goal: S, resolution: float = 0.1) -> list[S] | None:
        """
        Generates a kinematically correct trajectory from the start state to the
        end state. Returns None if no feasible connection exists.
        TODO: figure out if I actually need to worry about none points, should be feasible
        """
        ...

    def is_terminal(self, state: S, goal: S, cfg: LatticeConfig) -> bool:
        """
        Returns True if state is close enough to goal to fire the terminal
        connection. Checks position for all bots, plus heading for diff/car/trailer,
        plus hitch angle for trailer.
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

    def make_state(self, x: float, y: float, h: float = 0, t:float = 0) -> S:
        ...


def check_collision(bot: Bot, state: S, obstacle: BaseGeometry) -> bool:
    """
    Returns True if the bot's footprint at the given state intersects the obstacle geometry.
    Defined as a free function rather than a protocol method because the implementation
    is identical for all bots.
    """
    return bot.footprint(state).intersects(obstacle)


class PointBot:
    def footprint(self, state: PointState):
        return point_geom(state)

    def is_terminal(
        self, state: PointState, goal: PointState, cfg: LatticeConfig
    ) -> bool:
        return center_distance(state, goal) < 1.0

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
    def footprint(self, state: DiffState):
        return diff_geom(state)

    def is_terminal(
        self, state: DiffState, goal: DiffState, cfg: LatticeConfig
    ) -> bool:
        return False

    def at_goal(self, state: DiffState, goal: DiffState) -> bool:
        return (
            center_distance(state, goal) < cfg.goal_radius_tolerance
            and abs(angle_distance_rad(state.heading_rad, goal.heading_rad))
            < cfg.goal_heading_tolerance
        )

    def generate_trajectory(
        self, start: DiffState, goal: DiffState, resolution: float = 0.1
    ) -> list[DiffState] | None:
        return None

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
    MAX_STEER = math.radians(35)

    def footprint(self, state: CarState):
        return car_geom(state)

    def is_terminal(self, state: CarState, goal: CarState, cfg: LatticeConfig) -> bool:
        return False

    def at_goal(self, state: CarState, goal: CarState) -> bool:
        return (
            center_distance(state, goal) < cfg.goal_radius_tolerance
            and abs(angle_distance_rad(state.heading_rad, goal.heading_rad))
            < cfg.goal_heading_tolerance
        )

    def generate_trajectory(self, start: CarState, goal: CarState, resolution: float = 0.1) -> list[CarState] | None:
        return None

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
    MAX_STEER = math.radians(35)

    def footprint(self, state: TrailerState):
        return truck_trailer_geom(state)

    def is_terminal(
        self, state: TrailerState, goal: TrailerState, cfg: LatticeConfig
    ) -> bool:
        return False

    def at_goal(self, state: TrailerState, goal: TrailerState) -> bool:
        return (
            center_distance(state, goal) < cfg.goal_radius_tolerance
            and abs(angle_distance_rad(state.heading_rad, goal.heading_rad))
            < cfg.goal_heading_tolerance
            and abs(angle_distance_rad(state.heading_rad, goal.heading_rad))
            < cfg.trailer_heading_tolerance
        )

    def generate_trajectory(
        self, start: TrailerState, goal: TrailerState, resolution: float = 0.1
    ) -> list[TrailerState] | None:
        return None

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
