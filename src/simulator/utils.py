import numpy as np
import simulator.config as cfg


def grid_to_coords(x_cell, y_cell, center=True) -> tuple[float, float]:
    if center:
        x_cell += 0.5
        y_cell += 0.5

    return (x_cell * cfg.CELLS_TO_METERS, y_cell * cfg.CELLS_TO_METERS) 


def scale(points, scale=cfg.METERS_TO_PIXELS):
    return (np.array(points) * scale).tolist()
