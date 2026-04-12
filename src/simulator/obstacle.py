import numpy as np
import simulator.config as cfg
from simulator.draw import draw_shape

import pygame
import random

from shapely import STRtree
from shapely.geometry import box
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

def intersects_any(tree: STRtree, geom: BaseGeometry) -> bool:
    return len(tree.query(geom, predicate='intersects')) > 0

def placeRandomUnoccupied(
    grid: np.ndarray, val, modify: bool = True
) -> tuple[int, int]:
    while True:
        x = np.random.randint(0, grid.shape[0])
        y = np.random.randint(0, grid.shape[1])

        if grid[x][y] == 0:
            if modify:
                grid[x][y] = val
            return (x, y)


tetrominoes = {
    0: [(0, 0), (1, 0), (0, 1), (1, 1)],  # square
    1: [(0, 0), (1, 0), (2, 0), (1, 1)],  # T-shape
    2: [(0, 0), (0, 1), (1, 1), (2, 1)],  # L-shape
    3: [(0, 0), (0, 1), (1, 0), (2, 0)],  # other L-shape
    4: [(0, 0), (1, 0), (2, 0), (3, 0)],  # line
    5: [(0, 0), (0, 1), (1, 1), (1, 2)],  # squiggly shape
    6: [(1, 0), (1, 1), (0, 1), (0, 2)],  # other squiggly
}


def rotate(shape, k):
    """Rotate shape by 90 degrees k times."""

    rotated = shape
    for _ in range(k):
        rotated = [(c, -r) for (r, c) in rotated]

    return rotated


def populate_grid(grid_shape: tuple[int, int], probability: float) -> np.ndarray:
    num_rows, num_cols = grid_shape

    grid = np.zeros((num_rows, num_cols), dtype=int)

    num_cells_expected = int(num_rows * num_cols * probability)

    while grid.sum() < num_cells_expected:
        tet_type = random.randint(0, len(tetrominoes) - 1)

        shape = tetrominoes[tet_type]

        shape = rotate(shape, random.randint(0, 3))

        row = random.randint(0, num_rows - 1)
        col = random.randint(0, num_cols - 1)

        for dr, dc in shape:
            r, c = row + dr, col + dc
            if r < num_rows and c < num_cols and r >= 0 and c >= 0:
                grid[r, c] = 1

    return grid


regular_parking_spot = np.array([
    [0, 0, 0, 0, 0, 0],
    [1, 1, 0, 0, 1, 1]
])
trailer_parking_spot = np.array([
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 1]
])

def clear_start_goal(grid: np.ndarray, is_trailer: bool = False):
    clearance = 4 if is_trailer else 2
    spot = trailer_parking_spot if is_trailer else regular_parking_spot
    spot_h, spot_w = spot.shape

    assert grid.shape[0] >= spot_h, "Grid too short"
    assert grid.shape[1] >= max(clearance, spot_w), "Grid too narrow"

    # clearing space for start position
    grid[:1, :clearance] = 0
    # stamping parking spot into bottom-right corner
    grid[-spot_h:, -spot_w:] = spot


class ObstacleEnvironment:
    """
    This class needs to encapsulate the environment, particularly the scaling factor between the numpy array and the actual grid.

    Should:
    - do the initial creation (randomly populating the grid, and clearing the top left and bottom right)
    - allow translation between grid indices and actual x,y coordinaes
    - have a method for drawing them
    """

    def __init__(
        self, grid_shape: tuple[int, int], proportion_filled: float, trailer: bool
    ) -> None:
        self.grid: np.ndarray = populate_grid(grid_shape, proportion_filled)

        # robot_cell_length = int((robot_length // cfg.CELLS_TO_METERS) + 1)

        clear_start_goal(self.grid, trailer)

        CELL_SIZE = cfg.CELLS_TO_METERS
        polys = []
        for y, row in enumerate(self.grid):
            for x, filled in enumerate(row):
                if filled == 1:
                    # Shapely’s box is defined by (minx, miny, maxx, maxy)
                    polys.append(
                        box(
                            x * CELL_SIZE,
                            y * CELL_SIZE,
                            (x + 1) * CELL_SIZE,
                            (y + 1) * CELL_SIZE,
                        )
                    )


        self.obstacles = STRtree(polys)

        x, y = grid_shape

        self.enclosure_geom = box(0,0, x * cfg.CELLS_TO_METERS, y * cfg.CELLS_TO_METERS)



    def get_cell_val(self, x: float, y: float):
        x_cell = int(x // cfg.CELLS_TO_METERS)
        y_cell = int(y // cfg.CELLS_TO_METERS)

        return self.grid[x_cell][y_cell]
    
    def is_valid_state(self, bot_geom: BaseGeometry) -> bool:
        return not intersects_any(self.obstacles, bot_geom) and self.enclosure_geom.contains(bot_geom)

    def draw_grid(
        self,
        screen: pygame.Surface,
    ) -> None:
        # print(type(self.obstacles.geometries[0]))
        for g in self.obstacles.geometries:
            draw_shape(screen, g, cfg.BLACK, True, cfg.GRAY)

