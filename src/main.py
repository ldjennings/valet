import argparse
import math
import random

import config as cfg
from simulator import make_bot
from simulator.simulator import Simulator
from environment import ObstacleEnvironment
from planner import HybridConfig


def grid_to_coords(x_cell: float, y_cell: float, center: bool = True) -> tuple[float, float]:
    if center:
        x_cell += 0.5
        y_cell += 0.5
    return (x_cell * cfg.CELLS_TO_METERS, y_cell * cfg.CELLS_TO_METERS)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="valet sim", description="simulator for RBE 550 assignment"
    )
    parser.add_argument(
        "-m", "--manual",
        action="store_true",
        help="Manually control the bot's state through key presses.",
    )
    parser.add_argument(
        "-r", "--record",
        action="store_true",
        help="Save the pygame output as an mp4 file ('./recording.mp4' by default).",
    )
    parser.add_argument(
        "bot_type",
        nargs="?",
        choices=["point", "diff", "car", "trailer"],
        default="diff",
        help="Type of bot to simulate",
    )
    parser.add_argument(
        "-s", "--seed",
        type=int,
        default=None,
        help="RNG seed for reproducible obstacle layouts.",
    )
    return parser.parse_args()


# (start_col, start_row) from top-left; (col_offset, row_offset) from bottom-right
_SCENARIOS: dict[str, tuple[tuple[float, float], tuple[float, float]]] = {
    "point":   ((0,   0), (3.5,   1)),
    "diff":    ((0,   0), (3.5,   1)),
    "car":     ((0.5, 0), (3.575, 1.35)),
    "trailer": ((2,   0), (4,     1)),
}


def _make_scenario(bot_type: str, seed: int | None):
    start_col, start_row     = _SCENARIOS[bot_type][0]
    col_offset, row_offset   = _SCENARIOS[bot_type][1]

    startxy = grid_to_coords(start_col, start_row)
    goalxy  = grid_to_coords(cfg.NUM_COLS - col_offset, cfg.NUM_ROWS - row_offset)

    bundle      = make_bot(bot_type, startxy, goalxy)
    environment = ObstacleEnvironment(
        grid_shape          = (cfg.NUM_ROWS, cfg.NUM_COLS),
        cell_size_meters    = cfg.CELLS_TO_METERS,
        proportion_filled   = cfg.OBSTACLE_COVERAGE,
        trailer             = (bot_type == "trailer"),
        seed                = seed,
    )
    return bundle, environment


def main() -> None:
    args = parse_args()

    seed = args.seed if args.seed is not None else random.randint(0, 2**32 - 1)
    print(f"Using seed: {seed}  (re-run with -s {seed} to reproduce)")
    bundle, environment = _make_scenario(args.bot_type, seed)
    config = HybridConfig(spacing=1, angular_spacing=math.pi / 3, max_iterations=25000, fine_collision=False)

    Simulator(bundle, environment, config).run(args.manual, args.record)


if __name__ == "__main__":
    main()
