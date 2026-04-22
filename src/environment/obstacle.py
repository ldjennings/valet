from environment.grid import populate_grid, clear_start_goal

import numpy as np
from shapely import STRtree
from shapely.affinity import translate
from shapely.geometry import box
from shapely.geometry.base import BaseGeometry

from typing import TypeAlias

FootprintEntry: TypeAlias = tuple[float, float, BaseGeometry]


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
        self, grid_shape: tuple[int, int], cell_size_meters: float, proportion_filled: float, trailer: bool
    ) -> None:
        """
        Args:
            grid_shape: (num_rows, num_cols) of the grid.
            cell_size_meters: Side length of one grid cell in meters.
            proportion_filled: Target fraction of cells to fill with obstacles, in [0, 1].
            trailer: If True, uses wider start clearance and trailer-sized parking spot.
        """
        self.grid: np.ndarray = populate_grid(grid_shape, proportion_filled).astype(np.bool_)
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
        self._bounds        =    (0, 0, cols * cell_size_meters, rows * cell_size_meters)



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

    def _geom_AABB_grid_check(self, bounds) -> bool:
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

    def is_valid_state(self, bot_geoms: list[FootprintEntry]) -> bool:
        """Return True if none of the geometries intersect any obstacle and all lie within the boundary."""
        for ox, oy, g in bot_geoms:
            minx, miny, maxx, maxy = g.bounds
            bounds = (minx + ox, miny + oy, maxx + ox, maxy + oy)

            if not self._within_bounds(bounds):
                return False

            # Broad-phase rejection
            if not self._geom_AABB_grid_check(bounds):
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
