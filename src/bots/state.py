"""
State dataclasses for each robot type.

All states use @dataclass(frozen=True, slots=True):
  frozen=True: immutable after construction, generates __hash__ so instances
               can be used as dict keys and set members (needed for A* bookkeeping).
  slots=True:  replaces the per-instance __dict__ with fixed memory slots —
               faster attribute access and lower memory, important since the
               planner instantiates these hundreds of thousands of times.
"""

from dataclasses import dataclass
from typing import Protocol, Self, TypeVar, cast, runtime_checkable
import math

from utils import arc_len, center_distance, lerp_angle, linspace_angles, Pose, Position



@runtime_checkable
class PointTurnCapable(Protocol):
    def pure_rotation_linspace(self, end: Self, angular_step: float) -> list[Self]: ...

@runtime_checkable
class Rotateable(Protocol):
    def pose(self) -> Pose: ...
    def heading(self) -> float: ...


@dataclass(frozen=True, slots=True)
class PointState:
    """
    State for a point robot: just a 2D position.

    No heading as the robot is a dimensionless point that can move in any direction.

    x/y are world-space coordinates in meters.
    """

    x: float
    y: float

    def translate(self, dx: float, dy: float) -> "PointState":
        return PointState(self.x + dx, self.y + dy)

    def __iter__(self):
        yield self.x
        yield self.y

    def position(self) -> Position:
        return Position((self.x, self.y))

    def interpolate(self, end: "PointState", t: float) -> "PointState":
        return PointState(
            self.x + t * (end.x - self.x),
            self.y + t * (end.y - self.y)
        )



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

    def step(self, v: float, omega: float, dt: float) -> "DiffState":
        """
        v: linear velocity (m/s), omega: angular velocity (rad/s), dt: timestep (s)
        """
        return DiffState(
            center_x    = self.center_x + v * math.cos(self.heading_rad) * dt,
            center_y    = self.center_y + v * math.sin(self.heading_rad) * dt,
            heading_rad = self.heading_rad + omega * dt,
        )

    def position(self) -> Position:
        return Position((self.center_x, self.center_y))

    def interpolate(self, end: "DiffState", t: float) -> "DiffState":
        return DiffState(
            self.center_x + t * (end.center_x - self.center_x),
            self.center_y + t * (end.center_y - self.center_y),
            lerp_angle(self.heading_rad, end.heading_rad, t)
        )

    def pure_rotation_linspace(self, end: "DiffState", angular_step: float) -> list["DiffState"]:
        headings = linspace_angles(self.heading_rad, end.heading_rad, angular_step)
        return [DiffState(self.center_x, self.center_y, h) for h in headings]

    def pose(self) -> Pose:
        return Pose((self.center_x, self.center_y, self.heading_rad))

    def heading(self) -> float:
        return self.heading_rad

    def __iter__(self):
        yield self.center_x
        yield self.center_y
        yield self.heading_rad


@dataclass(frozen=True, slots=True)
class CarState:
    """
    State for a car-like (Ackermann steering) robot: position + heading.

    x/y is the center of the rear axle,  standard kinematic reference
    point for car models (Reeds-Shepp, Dubins, bicycle model).

    The car cannot rotate in place; turning radius is bounded by max steering angle.
    """

    rear_axle_x: float
    rear_axle_y: float
    heading_rad: float

    def step(self, v: float, delta: float, L: float, dt: float) -> "CarState":
        """
        delta: steering angle (radians), L: wheelbase (meters)
        """
        return CarState(
            rear_axle_x = self.rear_axle_x + v * math.cos(self.heading_rad) * dt,
            rear_axle_y = self.rear_axle_y + v * math.sin(self.heading_rad) * dt,
            heading_rad = self.heading_rad + (v * math.tan(delta) / L) * dt,
        )

    def position(self) -> Position:
        return Position((self.rear_axle_x, self.rear_axle_y))

    def interpolate(self, end: "CarState", t: float) -> "CarState":
        return CarState(
            rear_axle_x= self.rear_axle_x + t * (end.rear_axle_x - self.rear_axle_x),
            rear_axle_y= self.rear_axle_y + t * (end.rear_axle_y - self.rear_axle_y),
            heading_rad= lerp_angle(self.heading_rad, end.heading_rad, t)
        )

    def pose(self) -> Pose:
        return Pose((self.rear_axle_x, self.rear_axle_y, self.heading_rad))

    def heading(self) -> float:
        return self.heading_rad

    def __iter__(self):
        yield self.rear_axle_x
        yield self.rear_axle_y
        yield self.heading_rad


@dataclass(frozen=True, slots=True)
class TrailerState:
    """
    State for a truck-and-trailer system: x/y position + truck heading + trailer heading.

    x/y is the center of the truck's rear axle (hitch attachment point).
    heading_rad is the truck's heading in world space.
    trailer_heading_rad is the trailer heading in world space

    The trailer motion is fully determined by the truck's motion and the current trailer heading.
    """

    rear_axle_x: float
    rear_axle_y: float
    heading_rad: float
    trailer_heading_rad: float

    def step(
        self, v: float, delta: float, L: float, M: float, dt: float
    ) -> "TrailerState":
        """
        delta: steering angle (radians)
        L: truck wheelbase (meters)
        M: hitch to trailer axle distance (meters)
        """
        phi = self.trailer_heading_rad
        dtheta = v * math.tan(delta) / L
        return TrailerState(
            rear_axle_x         = self.rear_axle_x + v * math.cos(self.heading_rad) * dt,
            rear_axle_y         = self.rear_axle_y + v * math.sin(self.heading_rad) * dt,
            heading_rad         = self.heading_rad + dtheta * dt,
            trailer_heading_rad = phi + ((v / M) * math.sin(self.heading_rad - phi) * dt)
        )

    def position(self) -> Position:
        return Position((self.rear_axle_x, self.rear_axle_y))

    def pose(self) -> Pose:
        return Pose((self.rear_axle_x, self.rear_axle_y, self.heading_rad))

    def heading(self) -> float:
        return self.heading_rad

    def interpolate(self, end: "TrailerState", t: float) -> "TrailerState":
        x0,y0, h0, th0 = self
        x1,y1, h1, th1 = end

        return TrailerState(
            rear_axle_x=            x0 + t * (x1 - x0),
            rear_axle_y=            y0 + t * (y1 - y0),
            heading_rad=            lerp_angle(h0, h1, t),
            trailer_heading_rad=    lerp_angle(th0, th1, t)
        )

    def __iter__(self):
        yield self.rear_axle_x
        yield self.rear_axle_y
        yield self.heading_rad
        yield self.trailer_heading_rad


# TypeVar constraining S to exactly these four types.
# Used to make Bot, Primitive, and Hybrid A* generic over the state types,
# so Bot[PointState] only accepts PointState, Bot[DiffState] only accepts DiffState, etc.
# This also makes it so that functions only accept S's of the same type, not allowing mixing.
# This is purely a static analysis tool, no actual runtime existence.
S = TypeVar("S", PointState, DiffState, CarState, TrailerState)


def trajectory_length(traj: list[S], ang_weight: float) -> float:
    """Sum of weighted pose distances along a trajectory.

    For Rotateable states, each step contributes XY distance + angular difference * ang_weight.
    For PointState, contributes XY distance only.
    """
    if len(traj) == 0:
        return 0.0

    if isinstance(traj[0], Rotateable):
        posed = cast(list[Rotateable], traj)
        return sum(
            arc_len(p0.pose(), p1.pose(), ang_weight)
            for p0, p1 in zip(posed, posed[1:])
        )
    else:
        return sum(
            center_distance(p0.position(), p1.position())
            for p0, p1 in zip(traj, traj[1:])
        )
