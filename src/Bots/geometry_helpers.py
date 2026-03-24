import simulator.config as cfg
from Bots.BotState import PointState, DiffState, CarState, TrailerState

import math
from shapely.ops import unary_union
from shapely.affinity import rotate, translate
from shapely.geometry import box, Point, Polygon, LineString
from shapely.geometry.base import BaseGeometry



def make_rect_geom(centerX: float, centerY: float, angle_rad: float, length: float, width:float) -> BaseGeometry:
    angle_deg = math.degrees(angle_rad)
    initial = box(-length/2, -width/2, length/2, width/2)
    rot = rotate(initial, angle_deg)
    positioned = translate(rot, centerX, centerY)

    return positioned

def diff_drive_geom(axle_center_x: float, axle_center_y: float, angle_rad: float, wheelbase: float, length: float, width:float) -> BaseGeometry:
    rect = box(-length / 2, -width / 2, length / 2, width / 2)

    rear_overhang = length - wheelbase
    offset = (length / 2) - rear_overhang
    rect = translate(rect, xoff=offset)

    rect = rotate(rect, math.degrees(angle_rad), origin=(0, 0))
    rect = translate(rect, xoff=axle_center_x, yoff=axle_center_y)
    return rect




def point_geom(state: PointState) -> BaseGeometry:
    x, y = state
    return Point(x, y).buffer(1.0) # change it for different radii

def diff_geom(state: DiffState) -> BaseGeometry:
    x, y, heading_rad = state
    return make_rect_geom(x, y, heading_rad, cfg.ROBOT_LENGTH_METERS, cfg.ROBOT_WIDTH_METERS)

def car_geom(state: CarState) -> BaseGeometry:
    x, y, heading_rad = state
    # return make_rect_geom(x, y, heading_rad, cfg.CAR_LENGTH_METERS, cfg.CAR_WIDTH_METERS)
    return diff_drive_geom(x, y, heading_rad, cfg.CAR_WHEELBASE_METERS, cfg.CAR_LENGTH_METERS, cfg.CAR_WIDTH_METERS)

def truck_geom(x: float, y: float, heading_rad: float) -> BaseGeometry:
    TRUCK_LEN = cfg.TRUCK_LENGTH_METERS
    TRUCK_WID = cfg.TRUCK_WIDTH_METERS
    WHEELBASE = cfg.TRUCK_WHEELBASE_METERS
    return diff_drive_geom(x, y, heading_rad, WHEELBASE, TRUCK_LEN, TRUCK_WID)



def truck_trailer_geom(state: TrailerState) -> BaseGeometry:
    """
    Returns a Shapely geometry (truck + trailer + connection)
    given truck rear-axle center (x, y), truck heading theta, and trailer angle phi.
    The trailer is attached directly at the truck's rear axle.
    """

    x, y, truck_heading_rad, trailer_heading_rad = state

    truck = truck_geom(x, y, truck_heading_rad)

    TRAILER_LEN = cfg.TRAILER_LENGTH_METERS
    TRAILER_WID = cfg.TRAILER_WIDTH_METERS
    D1 = cfg.TRUCK_HITCH_TO_TRAILER_AXLE

    # Trailer axle x/y
    x_t = x - D1 * math.cos(truck_heading_rad + trailer_heading_rad)
    y_t = y - D1 * math.sin(truck_heading_rad + trailer_heading_rad)

    trailer = make_rect_geom(x_t, y_t, truck_heading_rad + trailer_heading_rad, TRAILER_LEN, TRAILER_WID)

    # line between the centers of the rear axles of both
    connection = LineString([(x, y), (x_t, y_t)])

    return unary_union([truck, trailer, connection])