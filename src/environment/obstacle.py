"""
Obstacle environment: collision detection and state validation.

ObstacleEnvironment wraps the occupancy grid and a Shapely STRtree of obstacle
polygons. Collision queries use a three-phase strategy — bounds check, broad-phase
grid lookup, exact Shapely intersection — to keep the common (free) case cheap.
"""

from environment.grid import populate_grid, clear_start_goal
from bots.geometry import LineFootprint, FootprintEntry

import numpy as np
from shapely import STRtree
from shapely.affinity import translate
from shapely.geometry import box
from shapely.geometry.base import BaseGeometry

from typing import Final



# ── Helpers ───────────────────────────────────────────────────────────────────

def intersects_any(tree: STRtree, geom: BaseGeometry) -> bool:
    """Return True if `geom` intersects any geometry in `tree`."""
    return len(tree.query(geom, predicate='intersects')) > 0

# ── Environment ───────────────────────────────────────────────────────────────

class ObstacleEnvironment:
    """
    World environment for collision checking and state validation.

    Stores obstacles as a Shapely STRtree for efficient spatial queries.
    All coordinates are in meters; `cell_size` is the conversion factor
    from grid indices to meters.

    The grid origin (0, 0) is the top-left corner. The start region is
    near (0, 0) and the parking goal is at the bottom-right.
    """

    def __init__(
        self, grid_shape: tuple[int, int], cell_size_meters: float, proportion_filled: float, trailer: bool,
        seed: int | None = None,
    ) -> None:
        """
        Args:
            grid_shape: (num_rows, num_cols) of the grid.
            cell_size_meters: Side length of one grid cell in meters.
            proportion_filled: Target fraction of cells to fill with obstacles, in [0, 1].
            trailer: If True, uses wider start clearance and trailer-sized parking spot.
            seed: Optional RNG seed for reproducible obstacle layouts.
        """
        self.grid: np.ndarray = populate_grid(grid_shape, proportion_filled, seed).astype(np.bool_)
        self.cell_size = cell_size_meters

        clear_start_goal(self.grid, trailer)


        polys = []
        for y, row in enumerate(self.grid):
            for x, filled in enumerate(row):
                if filled:
                    # Shapely’s box is defined by (minx, miny, maxx, maxy)
                    polys.append(
                        box(
                            x * cell_size_meters,
                            y * cell_size_meters,
                            (x + 1) * cell_size_meters,
                            (y + 1) * cell_size_meters,
                        )
                    )


        self.obstacles = STRtree(polys)

        rows, cols = grid_shape

        self.enclosure_geom = box(0, 0, cols * cell_size_meters, rows * cell_size_meters)
        self._bounds: Final  = (0, 0, cols * cell_size_meters, rows * cell_size_meters)



    def get_cell_val(self, x: float, y: float):
        """Return the grid value (0=free, 1=occupied) at world coordinates (x, y)."""
        x_cell = int(x // self.cell_size)
        y_cell = int(y // self.cell_size)

        return self.grid[y_cell][x_cell]

    def is_point_free(self, x: float, y: float) -> bool:
        """Fast check: is (x, y) inside the boundary and not inside any obstacle cell?"""
        if not (0 <= x <= self.enclosure_geom.bounds[2] and 0 <= y <= self.enclosure_geom.bounds[3]):
            return False
        cx, cy = int(x // self.cell_size), int(y // self.cell_size)

        if 0 <= cy < self.grid.shape[0] and 0 <= cx < self.grid.shape[1]:
            return self.grid[cy][cx] == 0

        return False

    def _has_possible_collision(self, bounds) -> bool:
        """Conservative broad-phase check: True if any obstacle cell overlaps the AABB.

        Converts the bounding box to grid indices and checks whether any cell in
        that region is occupied. A False result guarantees no collision; True means
        the exact check is needed.

        Must be called after _within_bounds — does not guard against out-of-range indices.
        """
        minx, miny, maxx, maxy = bounds

        # Convert world coordinates → grid indices
        min_i = int(minx // self.cell_size)
        max_i = int(maxx // self.cell_size)
        min_j = int(miny // self.cell_size)
        max_j = int(maxy // self.cell_size)

        subgrid = self.grid[min_j:max_j + 1, min_i:max_i + 1]
        return bool(subgrid.any())

    def _within_bounds(self, bounds) -> bool:
        """Fast axis-aligned bounding box containment check."""
        minx, miny, maxx, maxy = bounds
        bx0, by0, bx1, by1 = self._bounds
        return minx >= bx0 and miny >= by0 and maxx <= bx1 and maxy <= by1

    def _segment_hits_obstacle(self, x0: float, y0: float, x1: float, y1: float) -> bool:
        """
        Check whether the line segment (x0,y0)→(x1,y1) passes through any occupied grid cell,
        using the fast voxel traversal algorithm (Amanatides & Woo, 1987).
        """
        cs = self.cell_size
        rows, cols = self.grid.shape

        cx, cy = int(x0 // cs), int(y0 // cs)
        ex, ey = int(x1 // cs), int(y1 // cs)

        dx = x1 - x0
        dy = y1 - y0

        step_x = 1 if dx > 0 else -1
        step_y = 1 if dy > 0 else -1

        # t increment to cross one full cell in each axis
        t_delta_x = abs(cs / dx) if dx != 0 else float('inf')
        t_delta_y = abs(cs / dy) if dy != 0 else float('inf')

        # t at which the ray first crosses a boundary in each axis
        next_x = (cx + (1 if dx > 0 else 0)) * cs
        next_y = (cy + (1 if dy > 0 else 0)) * cs
        t_max_x = abs((next_x - x0) / dx) if dx != 0 else float('inf')
        t_max_y = abs((next_y - y0) / dy) if dy != 0 else float('inf')

        while True:
            if 0 <= cy < rows and 0 <= cx < cols and self.grid[cy, cx]:
                return True
            if cx == ex and cy == ey:
                return False
            if t_max_x < t_max_y:
                t_max_x += t_delta_x
                cx += step_x
            else:
                t_max_y += t_delta_y
                cy += step_y

    def is_valid_state(self, bot_geoms: list[FootprintEntry]) -> bool:
        """Return True if none of the geometries intersect any obstacle and all lie within the boundary."""
        for entry in bot_geoms:
            if isinstance(entry, LineFootprint):
                # check if the line is contained within the environment
                bx0, by0, bx1, by1 = self._bounds
                if not (bx0 <= entry.x0 <= bx1 and by0 <= entry.y0 <= by1 and
                        bx0 <= entry.x1 <= bx1 and by0 <= entry.y1 <= by1):
                    return False
                # test if the line hits an obstacle
                if self._segment_hits_obstacle(entry.x0, entry.y0, entry.x1, entry.y1):
                    return False
            else:
                ox, oy, g, (minx, miny, maxx, maxy) = entry
                bounds = (minx + ox, miny + oy, maxx + ox, maxy + oy)

                if not self._within_bounds(bounds):
                    return False

                # Broad-phase rejection
                if not self._has_possible_collision(bounds):
                    continue

                # Very rare, also very expensive. Not called in the majority of cases.
                # Only translate if the geometry isn't already placed.
                if ox != 0.0 or oy != 0.0:
                    positioned = translate(g, xoff=ox, yoff=oy)
                else:
                    positioned = g
                if intersects_any(self.obstacles, positioned):
                    return False

        return True
