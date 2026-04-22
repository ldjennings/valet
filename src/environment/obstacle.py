from environment.grid import populate_grid, clear_start_goal
from bots.geometry import LineFootprint, FootprintEntry

import numpy as np
from shapely import STRtree
from shapely.affinity import translate
from shapely.geometry import box
from shapely.geometry.base import BaseGeometry

from typing import Final



def intersects_any(tree: STRtree, geom: BaseGeometry) -> bool:
    """Return True if `geom` intersects any geometry in `tree`."""
    return len(tree.query(geom, predicate='intersects')) > 0

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

    def has_possible_collision(self, bounds) -> bool:
        """
        Fast broad-phase collision check between a geometry and the occupancy grid.

        This method:
        1. Computes the axis-aligned bounding box (AABB) of the input geometry.
        2. Converts that AABB into grid index space.
        3. Slices the corresponding subgrid region.
        4. Determines if any of those cells are occupied

        This is a conservative test:
        - Returns True if there could be a collision
        - Returns False if there is definitely no collision.

        It is intended as a broad-phase rejection step before more precise checks.

        **DOES NOT CHECK IF THE AABB IS INSIDE GRID, CAN THROW ERRORS**

        **EXPECTED TO BE APPLIED AFTER `_within_bounds`**

        Args:
            g: A Shapely geometry (typically your OBB robot shape).

        Returns:
            bool: True if any occupied grid cell lies within the geometry's AABB,
                False otherwise.
        """
        # minx, miny, maxx, maxy = g.bounds
        minx, miny, maxx, maxy = bounds

        # Convert world coordinates → grid indices
        min_i = int(minx // self.cell_size)
        max_i = int(maxx // self.cell_size)
        min_j = int(miny // self.cell_size)
        max_j = int(maxy // self.cell_size)

        # # Clamp to grid bounds to avoid slicing errors
        # min_i = max(min_i, 0)
        # min_j = max(min_j, 0)
        # max_i = min(max_i, self.grid.shape[1] - 1)
        # max_j = min(max_j, self.grid.shape[0] - 1)

        # # If the AABB lies completely outside the grid, no collision
        # if min_i > max_i or min_j > max_j:
        #     return False

        # Slice the candidate region
        subgrid = self.grid[min_j:max_j + 1, min_i:max_i + 1]

        # Check if any in the region are actually occupied
        return bool(subgrid.any())

    def _within_bounds(self, bounds) -> bool:
        """Fast axis-aligned bounding box containment check."""
        minx, miny, maxx, maxy = bounds
        bx0, by0, bx1, by1 = self._bounds
        return minx >= bx0 and miny >= by0 and maxx <= bx1 and maxy <= by1

    def _segment_hits_obstacle(self, x0: float, y0: float, x1: float, y1: float) -> bool:
        """
        Check whether the line segment (x0,y0)→(x1,y1) passes through any occupied grid cell,
        using Bresenham's line algorithm. https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm

        No allocation — pure integer arithmetic directly on the grid array.

        How it works:
          1. Convert both endpoints from world coordinates to integer grid-cell indices.
          2. Compute the axis-aligned deltas (dx, dy) and step directions (sx, sy).
          3. Maintain an integer error term `err = dx - dy` that tracks how far the
             true line has drifted from the current grid cell centre.
          4. Each iteration, double the error (e2 = 2*err) and compare against the
             thresholds -dy and dx to decide whether to step in x, y, or both.
             This keeps the rasterised path within 0.5 cells of the true line at all times.
          5. Return True immediately on the first occupied cell; return False if the
             endpoint is reached without a hit.
        """
        cs = self.cell_size
        cx, cy     = int(x0 // cs), int(y0 // cs)
        ex, ey     = int(x1 // cs), int(y1 // cs)
        dx, dy     = abs(ex - cx), abs(ey - cy)
        sx, sy     = (1 if ex > cx else -1), (1 if ey > cy else -1)
        err        = dx - dy
        rows, cols = self.grid.shape

        while True:
            # hit an obstacle
            if 0 <= cy < rows and 0 <= cx < cols and self.grid[cy, cx]:
                return True
            # hit the end of the line
            if cx == ex and cy == ey:
                return False
            e2 = 2 * err
            # step horizontally when the error favours x
            if e2 > -dy:
                err -= dy
                cx  += sx
            # step vertically when the error favours y
            if e2 < dx:
                err += dx
                cy  += sy

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
                if not self.has_possible_collision(bounds):
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
