"""
Shapely geometry construction for robot collision footprints.

Each function takes a bot state and returns a Shapely geometry representing
the robot's physical extent at that state, used for collision checking against
the obstacle environment.

Base shapes (rectangles, circles) are built once via make_*_base() and cached
by the bot instance. Per-state footprint calls only rotate + translate.
"""

from Bots.BotState import PointState, TrailerState

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


# ── heading cache ─────────────────────────────────────────────────────────

HEADING_CACHE_SIZE = 72  # one slot per 5°; good enough for collision checking

def build_heading_cache(base: BaseGeometry) -> list[BaseGeometry]:
    """Pre-rotate base shape at HEADING_CACHE_SIZE evenly-spaced headings."""
    return [
        rotate(base, math.degrees(i * 2 * math.pi / HEADING_CACHE_SIZE), origin=(0, 0))
        for i in range(HEADING_CACHE_SIZE)
    ]


def lookup_cached(cache: list[BaseGeometry], heading_rad: float) -> BaseGeometry:
    """Find the closest pre-rotated shape for a given heading."""
    idx = round(heading_rad % (2 * math.pi) / (2 * math.pi) * HEADING_CACHE_SIZE) % HEADING_CACHE_SIZE
    return cache[idx]


# ── per-state placement (translate only, rotation is cached) ──────────────

def place(base: BaseGeometry, x: float, y: float, angle_rad: float) -> BaseGeometry:
    """Rotate base shape by angle_rad about origin, then translate to (x, y).
    Used only for rendering (exact heading); collision checking uses the cache."""
    return translate(rotate(base, math.degrees(angle_rad), origin=(0, 0)), xoff=x, yoff=y)


def point_geom(base: BaseGeometry, state: PointState) -> BaseGeometry:
    x, y = state.position()
    return translate(base, xoff=x, yoff=y)


def cached_geom(cache: list[BaseGeometry], x: float, y: float, heading_rad: float) -> BaseGeometry:
    """Look up the nearest pre-rotated shape and translate to (x, y)."""
    return translate(lookup_cached(cache, heading_rad), xoff=x, yoff=y)


def truck_trailer_geom(
    state: TrailerState,
    truck_base: BaseGeometry,
    trailer_base: BaseGeometry,
    hitch_distance: float,

) -> list[tuple[float, float, BaseGeometry]]:
    """
    Returns [connection, truck, trailer] as separate geometries.

    hitch_distance: distance from hitch point (rear axle) to trailer axle center.
    If approximate=True, uses pre-rotated caches instead of exact rotation.
    """
    x, y, truck_heading = state.pose()
    trailer_heading = state.trailer_heading_rad

    truck = place(truck_base, x, y, truck_heading)

    x_t = x - hitch_distance * math.cos(trailer_heading)
    y_t = y - hitch_distance * math.sin(trailer_heading)

    trailer = place(trailer_base, x_t, y_t, trailer_heading)

    connection = LineString([(x, y), (x_t, y_t)])

    return [(0, 0, connection), (0, 0, truck), (0, 0, trailer)]

def truck_trailer_approximate(
    state: TrailerState, hitch_distance: float,
    truck_cache: list[BaseGeometry],
    trailer_cache: list[BaseGeometry],
) -> list[tuple[float, float, BaseGeometry]]:
    x, y, truck_h, trailer_h = state

    x_t = x - hitch_distance * math.cos(trailer_h)
    y_t = y - hitch_distance * math.sin(trailer_h)
    connection = LineString([(x, y), (x_t, y_t)])


    truck   = lookup_cached(truck_cache, truck_h)
    trailer = lookup_cached(trailer_cache, trailer_h)
    return [(0, 0, connection), (x, y, truck), (x_t, y_t, trailer)]


# def truck_trailer_geom(
#     truck_base: BaseGeometry, trailer_base: BaseGeometry,
#     state: TrailerState, hitch_distance: float,
#     truck_cache: list[BaseGeometry] | None = None,
#     trailer_cache: list[BaseGeometry] | None = None,
#     approximate: bool = False,
# ) -> list[BaseGeometry]:
#     """
#     Returns [connection, truck, trailer] as separate geometries.

#     hitch_distance: distance from hitch point (rear axle) to trailer axle center.
#     If approximate=True, uses pre-rotated caches instead of exact rotation.
#     """
#     x, y = pos(state)
#     truck_heading, trailer_heading = angs(state) or []

#     if approximate and truck_cache is not None and trailer_cache is not None:
#         truck = cached_geom(truck_cache, x, y, truck_heading)
#     else:
#         truck = place(truck_base, x, y, truck_heading)

#     x_t = x - hitch_distance * math.cos(trailer_heading)
#     y_t = y - hitch_distance * math.sin(trailer_heading)

#     if approximate and truck_cache is not None and trailer_cache is not None:
#         trailer = cached_geom(trailer_cache, x_t, y_t, trailer_heading)
#     else:
#         trailer = place(trailer_base, x_t, y_t, trailer_heading)

#     connection = LineString([(x, y), (x_t, y_t)])

#     return [connection, truck, trailer]
