from shapely.geometry import Polygon, LineString
from shapely.affinity import rotate, translate
from shapely.ops import unary_union
import simulator.config as cfg
import math

def truck_trailer_geom(x, y, theta, phi):
    """
    Returns a Shapely geometry (truck + trailer + connection)
    given truck rear-axle center (x, y), truck heading theta, and trailer angle phi.
    The trailer is attached directly at the truck's rear axle.
    """
    # shortening the constants 
    TRUCK_LEN = cfg.TRUCK_LENGTH_METERS
    TRUCK_W = cfg.TRUCK_WIDTH_METERS
    WHEELBASE = cfg.TRUCK_WHEELBASE_METERS
    TRAILER_LEN = cfg.TRAILER_LENGTH_METERS
    TRAILER_W = cfg.TRAILER_WIDTH_METERS
    D1 = cfg.TRUCK_HITCH_TO_TRAILER_AXLE

    # Trailer axle
    x_t = x - D1 * math.cos(math.radians(theta + phi))
    y_t = y - D1 * math.sin(math.radians(theta + phi))

    # truck rectangle with the rear axle at 0,0
    truck_rect = Polygon([
        [WHEELBASE, -TRUCK_W/2],
        [-TRUCK_LEN + WHEELBASE, -TRUCK_W/2],
        [-TRUCK_LEN + WHEELBASE,  TRUCK_W/2],
        [WHEELBASE,  TRUCK_W/2]
    ])
    truck_world = rotate(truck_rect, theta, origin=(0, 0))
    truck_world = translate(truck_world, xoff=x, yoff=y)

    # trailer rectangle
    trailer_rect = Polygon([
        [-TRAILER_LEN/2, -TRAILER_W/2],
        [ TRAILER_LEN/2, -TRAILER_W/2],
        [ TRAILER_LEN/2,  TRAILER_W/2],
        [-TRAILER_LEN/2,  TRAILER_W/2]
    ])
    trailer_world = rotate(trailer_rect, theta + phi, origin=(0, 0))
    trailer_world = translate(trailer_world, xoff=x_t, yoff=y_t)

    # line between them
    connection = LineString([(x, y), (x_t, y_t)])

    return unary_union([truck_world, trailer_world, connection])