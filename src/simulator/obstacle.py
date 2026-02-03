import numpy as np
import matplotlib.pyplot as plt
import random
import simulator.config as cfg
import pygame
from enum import Enum

from shapely.geometry import box
from shapely.ops import unary_union
from shapely import Polygon, MultiPolygon

def grid_to_coords(x_cell, y_cell, center=True):
    if center:
        x_cell +=.5
        y_cell +=.5
    
    return scale((x_cell, y_cell), cfg.CELLS_TO_METERS)

def draw_shape(surface: pygame.Surface, geom, color, width=0):
    """
    Draw a Shapely geometry on a pygame surface.

    Args:
        surface: pygame.Surface to draw on
        geom: Shapely geometry (Polygon, LineString, MultiPolygon, etc.)
        color: RGB tuple
        width: line width (0 = filled)
    """
    if geom.geom_type == 'Polygon':
        pygame.draw.polygon(surface, color, scale(list(geom.exterior.coords)), width)  # type: ignore

    elif geom.geom_type == 'LineString':
        pygame.draw.lines(surface, color, False, scale(list(geom.coords)), width)  # type: ignore

    elif geom.geom_type == 'MultiPolygon':
        for g in geom.geoms:  # type: ignore
            draw_shape(surface, g, color, width)

    elif geom.geom_type == 'GeometryCollection':
        for g in geom.geoms:
            draw_shape(surface, g, color, width)

def scale(points, scale = cfg.METERS_TO_PIXELS):
    return (np.array(points) * scale).tolist()



def placeRandomUnoccupied(grid: np.ndarray, val, modify:bool = True) -> tuple[int,int]:
    while True:
        x = np.random.randint(0, grid.shape[0])
        y = np.random.randint(0, grid.shape[1])

        if grid[x][y] == 0:
            if modify:
                grid[x][y] = val
            return (x, y)


tetrominoes = {
    0: [(0,0), (1,0), (0,1), (1,1)],   # square
    1: [(0,0), (1,0), (2,0), (1,1)],   # T-shape
    2: [(0,0), (0,1), (1,1), (2,1)],   # L-shape
    3: [(0,0), (0,1), (1,0), (2,0)],   # other L-shape
    4: [(0,0), (1,0), (2,0), (3,0)],   # line 
    5: [(0,0), (0,1), (1,1), (1,2)],   # squiggly shape
    6: [(1,0), (1,1), (0,1), (0,2)],   # other squiggly
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
        tet_type = random.randint(0, len(tetrominoes)-1)

        shape = tetrominoes[tet_type]

        shape = rotate(shape, random.randint(0, 3))

        row = random.randint(0, num_rows-1)
        col = random.randint(0, num_cols-1)

        for dr, dc in shape:
            r, c = row + dr, col + dc
            if r < num_rows and c < num_cols and r >= 0 and c >= 0:
                grid[r, c] = 1


    return grid


def clear_start_goal(grid: np.ndarray, robot_length: int):
    print(f"Clearing {robot_length} cells")
    # clearing space for start position
    grid[0:robot_length, 0:robot_length] = 0

    # clearing bottom right corner
    grid[-robot_length:, -robot_length:] = 0

class ObstacleEnvironment():
    '''
    This class needs to encapsulate the environment, particularly the scaling factor between the numpy array and the actual grid.

    Should:
    - do the initial creation (randomly populating the grid, and clearing the top left and bottom right)
    - allow translation between grid indices and actual x,y coordinaes
    - have a method for drawing them
    '''


    def __init__(self, grid_shape: tuple[int, int], proportion_filled: float, robot_length: float) -> None:
        self.grid: np.ndarray = populate_grid(grid_shape, proportion_filled)

        robot_cell_length = int((robot_length // cfg.CELLS_TO_METERS) + 1)

        clear_start_goal(self.grid, robot_cell_length)

        CELL_SIZE = cfg.CELLS_TO_METERS
        polys = []
        for y, row in enumerate(self.grid):
            for x, filled in enumerate(row):
                if filled == 1:
                    # Shapely’s box is defined by (minx, miny, maxx, maxy)
                    polys.append(box(x * CELL_SIZE, y * CELL_SIZE,
                                    (x + 1) * CELL_SIZE, (y + 1) * CELL_SIZE))

        self.world_geom = unary_union(polys)

        x, y = grid_shape
        self.dims  = (x * cfg.CELLS_TO_METERS, y * cfg.CELLS_TO_METERS)



    def get_cell_val(self, x, y):
        x_cell = x // cfg.CELLS_TO_METERS
        y_cell = y // cfg.CELLS_TO_METERS

        return self.grid[x_cell][y_cell]


    def draw_grid(self, screen: pygame.Surface,) -> None:


        CELLS_TO_PIXELS = cfg.CELLS_TO_METERS * cfg.METERS_TO_PIXELS

        backing = pygame.Rect(0,0, cfg.NUM_COLS * CELLS_TO_PIXELS, cfg.NUM_ROWS * CELLS_TO_PIXELS)

        pygame.draw.rect(screen, cfg.WHITE, backing)

        draw_shape(screen, self.world_geom, cfg.BLACK)
        draw_shape(screen, self.world_geom, cfg.GRAY, 2)

