from simulator.LatticeConfig import LatticeConfig
import simulator.config as cfg
from Bots.BotState import S, PointState, DiffState, CarState, TrailerState
from Bots.geometry_helpers import point_geom, diff_geom, car_geom, trailer_geom

import math
from typing import Protocol, Generic
from shapely.geometry.base import BaseGeometry
from dataclasses import dataclass
import pygame

# just a constant for now
DT = 1/30




class Bot(Protocol[S]):
    def footprint(self, state: S) -> BaseGeometry: ...
    def primitives(self, state: S, cfg: LatticeConfig) -> list[S]: ...
    def is_terminal(self, state: S, goal: S, cfg: LatticeConfig) -> bool: ...
    def connect_to_goal(self, state: S, goal: S) -> list[S] | None: ...
    def handle_input(self, state: S, speed: float) -> S: ...



@dataclass
class BotBundle(Generic[S]):
    bot: Bot[S]
    state: S

class PointBot:
    def footprint(self, state: PointState):
        return point_geom(state)
    
    def primitives(self, state: PointState, cfg: LatticeConfig) -> list[PointState]:
        return []
    
    def is_terminal(self, state: PointState, goal: PointState, cfg: LatticeConfig) -> bool:
        return False
    
    def connect_to_goal(self, state: PointState, goal: PointState) -> list[PointState] | None: 
        return None
    
    def handle_input(self, state: PointState, speed: float) -> PointState:
        keymap = {
            pygame.K_LEFT:  (-0.5,    0),
            pygame.K_RIGHT: ( 0.5,    0),
            pygame.K_UP:    (   0, -0.5),
            pygame.K_DOWN:  (   0,  0.5),
        }

        keys = pygame.key.get_pressed()

        new_state = state        

        for k, delta in keymap.items():
            if keys[k]:
                new_state = new_state.translate(delta[0], delta[1])

        return new_state
    
class DiffBot:
    def footprint(self, state: DiffState):
        return diff_geom(state)
    
    def primitives(self, state: DiffState, cfg: LatticeConfig) -> list[DiffState]:
        return []
    
    def is_terminal(self, state: DiffState, goal: DiffState, cfg: LatticeConfig) -> bool:
        return False
    
    def connect_to_goal(self, state: DiffState, goal: DiffState) -> list[DiffState] | None: 
        return None
    
    def handle_input(self, state: DiffState, speed: float) -> DiffState:
        v, omega = 0.0, 0.0

        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP] and not keys[pygame.K_DOWN]:
            v =  speed
        elif keys[pygame.K_DOWN] and not keys[pygame.K_UP]:
            v = -speed
        if keys[pygame.K_LEFT] and not keys[pygame.K_RIGHT]:
            omega = -speed
        elif keys[pygame.K_RIGHT] and not keys[pygame.K_LEFT]:
            omega =  speed
        return state.step(v, omega, DT)

class CarBot:
    MAX_STEER = math.radians(35)

    def footprint(self, state: CarState):
        return car_geom(state)
    
    def primitives(self, state: CarState, cfg: LatticeConfig) -> list[CarState]:
        return []
    
    def is_terminal(self, state: CarState, goal: CarState, cfg: LatticeConfig) -> bool:
        return False
    
    def connect_to_goal(self, state: CarState, goal: CarState) -> list[CarState] | None: 
        return None
    
    def handle_input(self, state: CarState, speed: float) -> CarState:
        v, delta = 0.0, 0.0

        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP] and not keys[pygame.K_DOWN]:
            v =  speed
        elif keys[pygame.K_DOWN] and not keys[pygame.K_UP]:
            v = -speed

        if keys[pygame.K_LEFT] and not keys[pygame.K_RIGHT]:
            delta = -self.MAX_STEER
        elif keys[pygame.K_RIGHT] and not keys[pygame.K_LEFT]:
            delta =  self.MAX_STEER

        return state.step(v, delta, cfg.CAR_WHEELBASE_METERS, DT)


class TrailerBot:
    MAX_STEER = math.radians(35)

    def footprint(self, state: TrailerState):
        return trailer_geom(state)
    
    def primitives(self, state: TrailerState, cfg: LatticeConfig) -> list[TrailerState]:
        return []
    
    def is_terminal(self, state: TrailerState, goal: TrailerState, cfg: LatticeConfig) -> bool:
        return False
    
    def connect_to_goal(self, state: TrailerState, goal: TrailerState) -> list[TrailerState] | None: 
        return None
    
    def handle_input(self, state: TrailerState, speed: float) -> TrailerState:
        v, delta = 0.0, 0.0

        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP] and not keys[pygame.K_DOWN]:
            v =  speed
        elif keys[pygame.K_DOWN] and not keys[pygame.K_UP]:
            v = -speed

        if keys[pygame.K_LEFT] and not keys[pygame.K_RIGHT]:
            delta = -self.MAX_STEER
        elif keys[pygame.K_RIGHT] and not keys[pygame.K_LEFT]:
            delta =  self.MAX_STEER

        next_state = state.step(v, delta, cfg.TRUCK_WHEELBASE_METERS, cfg.TRUCK_HITCH_TO_TRAILER_AXLE, DT)
        # if abs(next_state.trailer_heading) > cfg.MAX_HITCH_ANGLE: # potential check for jacknifed trailer
        #     return state
        return next_state