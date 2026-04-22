import argparse
import math

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


def main() -> None:
    args = parse_args()

    trailer = (args.bot_type == "trailer")

    start_goal = {
        "point":   ((0, 0),     (-3.5, -1)),
        "diff":    ((0, 0),     (-3.5, -1)),
        "car":     ((0.5, 0),   (-3.575, -1.35)),
        "trailer": ((2, 0),     (-4, -1)),
    }

    (sr, sc), (gr, gc) = start_goal[args.bot_type]
    startxy = grid_to_coords(sr, sc)
    goalxy  = grid_to_coords(cfg.NUM_COLS + gr, cfg.NUM_ROWS + gc)

    bundle      = make_bot(args.bot_type, startxy, goalxy)
    environment = ObstacleEnvironment((cfg.NUM_ROWS, cfg.NUM_COLS), cfg.CELLS_TO_METERS, 0.1, trailer, seed=args.seed)
    config      = HybridConfig(spacing=1, angular_spacing=math.pi / 3, max_iterations=15000, fine_collision=False)

    Simulator(bundle, environment, config).run(args.manual, args.record)


if __name__ == "__main__":
    main()
