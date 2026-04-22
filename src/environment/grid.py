"""
Procedural generation of obstacle grids.

Grids are 2D integer arrays where 1 = occupied, 0 = free.
Obstacles are placed as randomly rotated tetromino shapes to create
realistic-looking clustered obstacles rather than uniform random noise.
"""

import numpy as np
import numpy.random as random



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


def populate_grid(grid_shape: tuple[int, int], probability: float, seed: int | None = None) -> np.ndarray:
    """
    Generate a random obstacle grid using tetromino-shaped obstacles.

    Places random tetrominoes until the fraction of filled cells reaches
    `probability`. May slightly exceed the target due to shape overlap.

    Args:
        grid_shape: (num_rows, num_cols) of the grid.
        probability: Target fraction of cells to fill, in [0, 1].
        seed: Optional RNG seed for reproducibility.
    """

    assert 0 <= probability <= 1.0, "probability not valid, must be in [0, 1]"

    rng = np.random.default_rng(seed)
    num_rows, num_cols = grid_shape

    grid = np.zeros((num_rows, num_cols), dtype=int)

    num_cells_expected = int(num_rows * num_cols * probability)

    while grid.sum() < num_cells_expected:
        tet_type = rng.integers(0, len(tetrominoes) - 1, endpoint=True)

        shape = tetrominoes[tet_type]

        shape = rotate(shape, rng.integers(0, 3, endpoint=True))

        row = rng.integers(0, num_rows - 1, endpoint=True)
        col = rng.integers(0, num_cols - 1, endpoint=True)

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
    """
    Stamp the start and goal regions into the grid in-place.

    Clears a column of cells at the top-left for the start position and
    stamps a parking spot pattern into the bottom-right corner for the goal.
    The trailer variant uses a wider start clearance and a longer parking spot
    to accommodate the truck-trailer geometry.
    """
    clearance = 4 if is_trailer else 2
    spot = trailer_parking_spot if is_trailer else regular_parking_spot
    spot_h, spot_w = spot.shape

    assert grid.shape[0] >= spot_h, "Grid too short"
    assert grid.shape[1] >= max(clearance, spot_w), "Grid too narrow"

    # clearing space for start position
    grid[:1, :clearance] = 0
    # stamping parking spot into bottom-right corner
    grid[-spot_h:, -spot_w:] = spot