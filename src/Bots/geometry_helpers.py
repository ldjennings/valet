import simulator.config as cfg
from Bots.BotState import PointState, DiffState, CarState, TrailerState

import math
from shapely.ops import unary_union
from shapely.affinity import rotate, translate
from shapely.geometry import box, Point, Polygon, LineString
from shapely.geometry.base import BaseGeometry



def make_rect_geom(centerX: float, centerY: float, angle_rad: float, width: float, height:float) -> BaseGeometry:
    angle_deg = math.degrees(angle_rad)
    initial = box(-width/2, -height/2, width/2, height/2)
    rot = rotate(initial, angle_deg)
    positioned = translate(rot, centerX, centerY)

    return positioned

def point_geom(state: PointState) -> BaseGeometry:
    x, y = state
    return Point(x, y)

def diff_geom(state: DiffState) -> BaseGeometry:
    x, y, heading_rad = state
    return make_rect_geom(x, y, heading_rad, cfg.ROBOT_WIDTH_METERS, cfg.ROBOT_LENGTH_METERS)

def car_geom(state: CarState) -> BaseGeometry:
    x, y, heading_rad = state
    return make_rect_geom(x, y, heading_rad, cfg.CAR_WIDTH_METERS, cfg.CAR_WIDTH_METERS)



def truck_geom(state: TrailerState) -> BaseGeometry:

    x, y, truck_heading_rad, _ = state



def trailer_geom(state: TrailerState) -> BaseGeometry:
    """
    Returns a Shapely geometry (truck + trailer + connection)
    given truck rear-axle center (x, y), truck heading theta, and trailer angle phi.
    The trailer is attached directly at the truck's rear axle.
    """

    x, y, truck_heading_rad, trailer_heading_rad = state

    # shortening the constants 
    TRUCK_LEN = cfg.TRUCK_LENGTH_METERS
    TRUCK_WID = cfg.TRUCK_WIDTH_METERS
    WHEELBASE = cfg.TRUCK_WHEELBASE_METERS
    TRAILER_LEN = cfg.TRAILER_LENGTH_METERS
    TRAILER_WID = cfg.TRAILER_WIDTH_METERS
    D1 = cfg.TRUCK_HITCH_TO_TRAILER_AXLE

    # Trailer axle x/y
    x_t = x - D1 * math.cos(truck_heading_rad + trailer_heading_rad)
    y_t = y - D1 * math.sin(truck_heading_rad + trailer_heading_rad)

    # truck rectangle with the rear axle at 0,0
    # truck_rect = Polygon([
    #     [WHEELBASE, -TRUCK_W/2],
    #     [-TRUCK_LEN + WHEELBASE, -TRUCK_W/2],
    #     [-TRUCK_LEN + WHEELBASE,  TRUCK_W/2],
    #     [WHEELBASE,  TRUCK_W/2]
    # ])
    # truck_world = rotate(truck_rect, math.degrees(theta_rad), origin=(0, 0))
    # truck_world = translate(truck_world, xoff=x, yoff=y)

    truck = make_rect_geom(x, y, truck_heading_rad, TRUCK_WID, TRUCK_LEN)
    trailer = make_rect_geom(x_t, y_t, truck_heading_rad + trailer_heading_rad, TRAILER_WID, TRAILER_LEN)

    # trailer rectangle
    # trailer_rect = Polygon([
    #     [-TRAILER_LEN/2, -TRAILER_W/2],
    #     [ TRAILER_LEN/2, -TRAILER_W/2],
    #     [ TRAILER_LEN/2,  TRAILER_W/2],
    #     [-TRAILER_LEN/2,  TRAILER_W/2]
    # ])
    # trailer_world = rotate(trailer_rect, math.degrees(theta_rad + phi_rad), origin=(0, 0))
    # trailer_world = translate(trailer_world, xoff=x_t, yoff=y_t)

    # line between them
    connection = LineString([(x, y), (x_t, y_t)])

    return unary_union([truck_world, trailer_world, connection])