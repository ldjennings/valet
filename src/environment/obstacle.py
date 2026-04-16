from environment.grid import populate_grid, clear_start_goal

import numpy as np
from shapely import STRtree
from shapely.geometry import box
from shapely.geometry.base import BaseGeometry



def intersects_any(tree: STRtree, geom: BaseGeometry) -> bool:
    return len(tree.query(geom, predicate='intersects')) > 0

class ObstacleEnvironment:
    """
    This class needs to encapsulate the environment, particularly the scaling factor between the numpy array and the actual grid.

    Should:
    - do the initial creation (randomly populating the grid, and clearing the top left and bottom right)
    - allow translation between grid indices and actual x,y coordinaes
    - have a method for drawing them
    """

    def __init__(
        self, grid_shape: tuple[int, int], cell_size_meters: float ,proportion_filled: float, trailer: bool
    ) -> None:
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

        x, y = grid_shape

        self.enclosure_geom = box(0,0, x * cell_size_meters, y * cell_size_meters)



    def get_cell_val(self, x: float, y: float):
        x_cell = int(x // self.cell_size)
        y_cell = int(y // self.cell_size)

        return self.grid[x_cell][y_cell]
    
    def is_valid_state(self, bot_geom: BaseGeometry) -> bool:
        return not intersects_any(self.obstacles, bot_geom) and self.enclosure_geom.contains(bot_geom)



