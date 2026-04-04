from dataclasses import dataclass
from typing import TypeVar
import numpy as np
import math

# frozen=True: immutable after construction, generates __hash__ so instances
#              can be used as dict keys and set members (needed for A* bookkeeping)
# slots=True:  replaces the per-instance __dict__ with fixed memory slots,
#              faster attribute access and lower memory — important since the
#              planner instantiates these hundreds of thousands of times

@dataclass(frozen=True, slots=True)
class PointState:
    """
    State for a point robot: just a 2D position.
    No heading — the robot is a dimensionless point that can move in any direction.
    x/y are world-space coordinates in meters.
    """
    x: float
    y: float

    def translate(self, dx: float, dy: float) -> 'PointState':
        return PointState(self.x + dx, self.y + dy)

    def __iter__(self):
        yield self.x
        yield self.y

@dataclass(frozen=True, slots=True)
class DiffState:
    """
    State for a differential drive robot: position + heading.
    x/y is the center of the wheel axle (geometric center of the robot body).
    heading_rad is the direction the robot faces in world space (radians).
    Diff drive can rotate in place, so heading and velocity are independent.
    """
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

@dataclass(frozen=True, slots=True)
class CarState:
    """
    State for a car-like (Ackermann steering) robot: position + heading.
    x/y is the center of the rear axle — this is the standard kinematic
    reference point for car models (Reeds-Shepp, Dubins, bicycle model).
    The car cannot rotate in place; turning radius is bounded by max steering angle.
    """
    rear_axle_x: float
    rear_axle_y: float
    heading_rad: float

    def step(self, v: float, delta: float, L: float, dt: float) -> 'CarState':
        """
        delta: steering angle (radians), L: wheelbase (meters)
        """
        return CarState(
            rear_axle_x = self.rear_axle_x + v * math.cos(self.heading_rad) * dt,
            rear_axle_y = self.rear_axle_y + v * math.sin(self.heading_rad) * dt,
            heading_rad = self.heading_rad + (v * math.tan(delta) / L) * dt,
        )

    def __iter__(self):
        yield self.rear_axle_x
        yield self.rear_axle_y
        yield self.heading_rad

@dataclass(frozen=True, slots=True)
class TrailerState:
    """
    State for a truck-and-trailer system: x/y position + truck heading + hitch angle.
    x/y is the center of the truck's rear axle (hitch attachment point).
    heading_rad is the truck's heading in world space.
    trailer_heading_rad is the trailer angle *relative to the truck heading* —
    zero means the trailer is aligned with the truck, positive is left.
    The trailer has no independent drive — its motion is fully determined by
    the truck's motion and the current hitch angle.
    """
    rear_axle_x: float
    rear_axle_y: float
    heading_rad: float
    trailer_heading_rad: float

    def step(self, v: float, delta: float, L: float, M: float, dt: float) -> 'TrailerState':
        """
        delta: steering angle (radians)
        L: truck wheelbase (meters)
        M: hitch to trailer axle distance (meters)
        """
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

# TypeVar constraining S to exactly these four types.
# Used to make Bot, Primitive, and LatticePlanner generic over the state type,
# so Bot[PointState] only accepts PointState, Bot[DiffState] only accepts DiffState, etc.
# This is purely a static analysis tool, no actual runtime existence.
S = TypeVar('S', PointState, DiffState, CarState, TrailerState)

def center_distance(s1: S, s2: S) -> float:
    x1, y1, *_ = s1
    x2, y2, *_ = s2

    return np.hypot((x2-x1), (y2 - y1))

def angle_distance_rad(a: float, b: float) -> float:
    """
    Smallest signed difference between two angles in radians.
    range is wrapped to [-pi, pi].
    """
    return (a - b + math.pi) % (2 * math.pi) - math.pi
