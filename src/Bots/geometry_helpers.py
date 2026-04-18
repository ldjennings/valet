"""
Shapely geometry construction for robot collision footprints.

Each function takes a bot state and returns a Shapely geometry representing
the robot's physical extent at that state, used for collision checking against
the obstacle environment.

Base shapes (rectangles, circles) are built once via make_*_base() and cached
by the bot instance. Per-state footprint calls only rotate + translate.
"""

from Bots.BotState import PointState, DiffState, CarState, TrailerState

import math
from shapely.affinity import rotate, translate
from shapely.geometry import box, Point, LineString
from shapely.geometry.base import BaseGeometry


# ── base shape constructors (call once, cache the result) ────────────────

def make_point_base(radius: float = 1.0) -> BaseGeometry:
    """Unit circle at origin."""
    return Point(0, 0).buffer(radius)


def make_centered_rect_base(length: float, width: float) -> BaseGeometry:
    """Rectangle centered at origin (for center-referenced bots like DiffBot)."""
    return box(-length / 2, -width / 2, length / 2, width / 2)


def make_axle_rect_base(wheelbase: float, length: float, width: float) -> BaseGeometry:
    """Rectangle with rear axle at origin (for Ackermann bots)."""
    rect = box(-length / 2, -width / 2, length / 2, width / 2)
    rear_overhang = length - wheelbase
    offset = (length / 2) - rear_overhang
    return translate(rect, xoff=offset)


# ── per-state placement (rotate + translate only) ────────────────────────

def place(base: BaseGeometry, x: float, y: float, angle_rad: float) -> BaseGeometry:
    """Rotate base shape by angle_rad about origin, then translate to (x, y)."""
    return translate(rotate(base, math.degrees(angle_rad), origin=(0, 0)), xoff=x, yoff=y)


def point_geom(base: BaseGeometry, state: PointState) -> BaseGeometry:
    x, y = state
    return translate(base, xoff=x, yoff=y)


def diff_geom(base: BaseGeometry, state: DiffState) -> BaseGeometry:
    x, y, heading_rad = state
    return place(base, x, y, heading_rad)


def car_geom(base: BaseGeometry, state: CarState) -> BaseGeometry:
    x, y, heading_rad = state
    return place(base, x, y, heading_rad)


def truck_trailer_geom(
    truck_base: BaseGeometry, trailer_base: BaseGeometry,
    state: TrailerState, hitch_distance: float,
) -> list[BaseGeometry]:
    """
    Returns [connection, truck, trailer] as separate geometries.

    hitch_distance: distance from hitch point (rear axle) to trailer axle center.
    """
    x, y, truck_heading, trailer_heading = state

    truck = place(truck_base, x, y, truck_heading)

    x_t = x - hitch_distance * math.cos(trailer_heading)
    y_t = y - hitch_distance * math.sin(trailer_heading)

    trailer = place(trailer_base, x_t, y_t, trailer_heading)
    connection = LineString([(x, y), (x_t, y_t)])

    return [connection, truck, trailer]
