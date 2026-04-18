from environment.grid import populate_grid, clear_start_goal

import numpy as np
from shapely import STRtree
from shapely.geometry import box, Point
from shapely.geometry.base import BaseGeometry



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
        self.grid: np.ndarray = populate_grid(grid_shape, proportion_filled)
        self.cell_size = cell_size_meters

        clear_start_goal(self.grid, trailer)

        polys = []
        for y, row in enumerate(self.grid):
            for x, filled in enumerate(row):
                if filled == 1:
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

        self.enclosure_geom = box(0,0, cols * cell_size_meters, rows * cell_size_meters)



    def get_cell_val(self, x: float, y: float):
        """Return the grid value (0=free, 1=occupied) at world coordinates (x, y)."""
        x_cell = int(x // self.cell_size)
        y_cell = int(y // self.cell_size)

        return self.grid[x_cell][y_cell]

    def is_point_free(self, x: float, y: float) -> bool:
        """Fast check: is (x, y) inside the boundary and not inside any obstacle cell?"""
        if not (0 <= x <= self.enclosure_geom.bounds[2] and 0 <= y <= self.enclosure_geom.bounds[3]):
            return False
        cx, cy = int(x // self.cell_size), int(y // self.cell_size)

        if 0 <= cy < self.grid.shape[0] and 0 <= cx < self.grid.shape[1]:
            return self.grid[cy][cx] == 0

        return False

    def is_valid_state(self, bot_geoms: list[BaseGeometry]) -> bool:
        """Return True if none of the geometries intersect any obstacle and all lie within the boundary."""
        return all(
            not intersects_any(self.obstacles, g) and self.enclosure_geom.contains(g)
            for g in bot_geoms
        )
