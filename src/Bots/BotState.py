from dataclasses import dataclass
from typing import TypeVar
import math



@dataclass(frozen=True)
class PointState:
    x: float
    y: float

    def translate(self, dx: float, dy: float) -> 'PointState':
        return PointState(self.x + dx, self.y + dy)

    def __iter__(self):
        yield self.x
        yield self.y

@dataclass(frozen=True)
class DiffState:
    center_x: float
    center_y: float
    heading_rad: float

    def step(self, v: float, omega: float, dt: float) -> 'DiffState':
        return DiffState(
            center_x       = self.center_x + v * math.cos(self.heading_rad) * dt,
            center_y       = self.center_y + v * math.sin(self.heading_rad) * dt,
            heading_rad = self.heading_rad + omega * dt,
        )

    def __iter__(self):
        yield self.center_x
        yield self.center_y
        yield self.heading_rad

@dataclass(frozen=True)
class CarState:
    rear_axle_x: float
    rear_axle_y: float
    heading_rad: float

    def step(self, v: float, delta: float, L: float, dt: float) -> 'CarState':
        return CarState(
            rear_axle_x = self.rear_axle_x + v * math.cos(self.heading_rad) * dt,
            rear_axle_y = self.rear_axle_y + v * math.sin(self.heading_rad) * dt,
            heading_rad = self.heading_rad + (v * math.tan(delta) / L) * dt,
        )

    def __iter__(self):
        yield self.rear_axle_x
        yield self.rear_axle_y
        yield self.heading_rad

@dataclass(frozen=True)
class TrailerState:
    rear_axle_x: float
    rear_axle_y: float
    heading_rad: float
    trailer_heading_rad: float

    def step(self, v: float, delta: float, L: float, M: float, dt: float) -> 'TrailerState':
        phi    = self.trailer_heading_rad
        dtheta = v * math.tan(delta) / L
        return TrailerState(
            rear_axle_x         = self.rear_axle_x + v * math.cos(self.heading_rad) * dt,
            rear_axle_y         = self.rear_axle_y + v * math.sin(self.heading_rad) * dt,
            heading_rad         = self.heading_rad + dtheta * dt,
            trailer_heading_rad = phi + (-(v * math.sin(phi) / M) - dtheta * math.cos(phi)) * dt,
        )

    def __iter__(self):
        yield self.rear_axle_x
        yield self.rear_axle_y
        yield self.heading_rad
        yield self.trailer_heading_rad

S = TypeVar('S', PointState, DiffState, CarState, TrailerState)
