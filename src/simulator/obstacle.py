import numpy as np
import matplotlib.pyplot as plt
import random


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