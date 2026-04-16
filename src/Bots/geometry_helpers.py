"""
Shapely geometry construction for robot collision footprints.

Each function takes a bot state and returns a Shapely geometry representing
the robot's physical extent at that state, used for collision checking against
the obstacle environment.
"""

import simulator.config as cfg
from Bots.BotState import PointState, DiffState, CarState, TrailerState

import math
from shapely.ops import unary_union
from shapely.affinity import rotate, translate
from shapely.geometry import box, Point, Polygon, LineString
from shapely.geometry.base import BaseGeometry


def point_geom(state: PointState) -> BaseGeometry:
    """Return a circular footprint for a point robot. Radius is fixed at 1.0 m."""
    x, y = state
    return Point(x, y).buffer(1.0)


def make_rect_geom(
    centerX: float, centerY: float, angle_rad: float, length: float, width: float
) -> BaseGeometry:
    """
    Build a rotated rectangle centered at (centerX, centerY).

    Constructs the rectangle at the origin, rotates it by angle_rad, then
    translates to the final position. The center of the rectangle coincides
    with the given coordinates — use ackermann_car_geom for rear-axle-referenced models.
    """
    angle_deg = math.degrees(angle_rad)
    initial = box(-length / 2, -width / 2, length / 2, width / 2)
    rot = rotate(initial, angle_deg)
    positioned = translate(rot, centerX, centerY)

    return positioned




def diff_geom(state: DiffState) -> BaseGeometry:
    x, y, heading_rad = state
    return make_rect_geom(
        x, y, heading_rad, cfg.ROBOT_LENGTH_METERS, cfg.ROBOT_WIDTH_METERS
    )


def ackermann_car_geom(
        axle_center_x: float,
        axle_center_y: float,
        angle_rad: float,
        wheelbase: float,
        length: float,
        width: float,
    ) -> BaseGeometry:
    """
    Build a rectangle whose rear axle is at (axle_center_x, axle_center_y).

    Cars are referenced to the rear axle, so the body extends forward by the
    wheelbase and back by the rear overhang (length - wheelbase). The rectangle
    is first built centered at the origin, then shifted forward along the x-axis
    so the rear axle sits at x=0, then rotated and translated to world position.

    Args:
        axle_center_x/y: World position of the rear axle center in meters.
        angle_rad: Heading of the vehicle in world space.
        wheelbase: Distance between front and rear axles in meters.
        length: Total vehicle length in meters.
        width: Total vehicle width in meters.
    """

    # making base rectangle centered at the origin
    rect = box(-length / 2, -width / 2, length / 2, width / 2)

    # calculating offset to place rear axle center at the origin
    rear_overhang = length - wheelbase
    offset = (length / 2) - rear_overhang
    rect = translate(rect, xoff=offset)

    # rotating/translating to final world position
    rect = rotate(rect, math.degrees(angle_rad), origin=(0, 0))
    rect = translate(rect, xoff=axle_center_x, yoff=axle_center_y)
    return rect

def car_geom(state: CarState) -> BaseGeometry:
    x, y, heading_rad = state
    return ackermann_car_geom(
        x,
        y,
        heading_rad,
        cfg.CAR_WHEELBASE_METERS,
        cfg.CAR_LENGTH_METERS,
        cfg.CAR_WIDTH_METERS,
    )


def truck_geom(x: float, y: float, heading_rad: float) -> BaseGeometry:
    TRUCK_LEN = cfg.TRUCK_LENGTH_METERS
    TRUCK_WID = cfg.TRUCK_WIDTH_METERS
    WHEELBASE = cfg.TRUCK_WHEELBASE_METERS
    return ackermann_car_geom(x, y, heading_rad, WHEELBASE, TRUCK_LEN, TRUCK_WID)


def truck_trailer_geom(state: TrailerState) -> BaseGeometry:
    """
    Build the combined footprint for a truck-and-trailer system.

    The truck is referenced to its rear axle (the hitch point). The trailer axle
    position is derived by stepping back from the hitch along the trailer heading
    by TRUCK_HITCH_TO_TRAILER_AXLE. The returned geometry is the union of the
    truck rectangle, trailer rectangle, and a line connecting the two axles
    (representing the hitch arm) for visual and collision purposes.
    """

    x, y, truck_heading_rad, trailer_heading_rad = state

    truck = truck_geom(x, y, truck_heading_rad)

    TRAILER_LEN = cfg.TRAILER_LENGTH_METERS
    TRAILER_WID = cfg.TRAILER_WIDTH_METERS
    D1 = cfg.TRUCK_HITCH_TO_TRAILER_AXLE

    # Trailer axle x/y
    x_t = x - D1 * math.cos(trailer_heading_rad)
    y_t = y - D1 * math.sin(trailer_heading_rad)

    trailer = make_rect_geom(
        x_t, y_t, trailer_heading_rad, TRAILER_LEN, TRAILER_WID
    )

    # line between the centers of the rear axles of both
    connection = LineString([(x, y), (x_t, y_t)])

    return unary_union([truck, trailer, connection])
