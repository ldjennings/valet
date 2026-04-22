"""
Shapely geometry construction for robot collision footprints.

Each function takes a bot state and returns a Shapely geometry representing
the robot's physical extent at that state, used for collision checking against
the obstacle environment.

Base shapes (rectangles, circles) are built once via make_*_base() and cached
by the bot instance. Per-state footprint calls only rotate + translate.
"""

from bots.state import PointState, TrailerState

import math
from shapely.affinity import rotate, translate
from shapely.geometry import box, Point
from shapely.geometry.base import BaseGeometry
from dataclasses import dataclass
from typing import TypeAlias


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

CachedShape: TypeAlias = tuple[BaseGeometry, tuple[float, float, float, float]]

def build_heading_cache(base: BaseGeometry) -> list[CachedShape]:
    """Pre-rotate base shape at HEADING_CACHE_SIZE evenly-spaced headings, storing bounds alongside."""
    result = []
    for i in range(HEADING_CACHE_SIZE):
        geom = rotate(base, math.degrees(i * 2 * math.pi / HEADING_CACHE_SIZE), origin=(0, 0))
        result.append((geom, geom.bounds))
    return result


def lookup_cached(cache: list[CachedShape], heading_rad: float) -> CachedShape:
    """Find the closest pre-rotated (geom, bounds) pair for a given heading."""
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


@dataclass(frozen=True, slots=True)
class LineFootprint:
    """A hitch-bar segment stored as raw endpoints for pure-math grid traversal (no Shapely)."""
    x0: float
    y0: float
    x1: float
    y1: float


RectFootprint: TypeAlias = tuple[float, float, BaseGeometry, tuple[float, float, float, float]]
FootprintEntry: TypeAlias = RectFootprint | LineFootprint

def truck_trailer_geom(
    state: TrailerState,
    truck_base: BaseGeometry,
    trailer_base: BaseGeometry,
    hitch_distance: float,
) -> list[FootprintEntry]:
    """
    Returns [connection, truck, trailer] as fully-placed geometries with pre-computed bounds.
    Used for exact (non-approximate) collision checking and rendering.
    """
    x, y, truck_heading = state.pose()
    trailer_heading = state.trailer_heading_rad

    x_t = x - hitch_distance * math.cos(trailer_heading)
    y_t = y - hitch_distance * math.sin(trailer_heading)

    truck   = place(truck_base, x, y, truck_heading)
    trailer = place(trailer_base, x_t, y_t, trailer_heading)

    return [
        LineFootprint(x, y, x_t, y_t),
        (0, 0, truck,   truck.bounds),
        (0, 0, trailer, trailer.bounds),
    ]


def truck_trailer_approximate(
    state: TrailerState, hitch_distance: float,
    truck_cache: list[CachedShape],
    trailer_cache: list[CachedShape],
) -> list[FootprintEntry]:
    """
    Returns [connection, truck, trailer] using pre-rotated cached shapes (offset not yet applied).
    Bounds are pre-computed — no Shapely property access at check time.
    """
    x, y, truck_h, trailer_h = state

    x_t = x - hitch_distance * math.cos(trailer_h)
    y_t = y - hitch_distance * math.sin(trailer_h)

    truck_geom,   truck_bounds   = lookup_cached(truck_cache,   truck_h)
    trailer_geom, trailer_bounds = lookup_cached(trailer_cache, trailer_h)

    return [
        LineFootprint(x, y, x_t, y_t),
        (x,   y,   truck_geom,   truck_bounds),
        (x_t, y_t, trailer_geom, trailer_bounds),
    ]
