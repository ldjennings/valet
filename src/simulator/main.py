import simulator.config as cfg
from simulator.recorder import MP4Recorder, NoOpRecorder
from simulator.obstacle import ObstacleEnvironment
from simulator.render import Renderer
from simulator.utils import grid_to_coords
from Bots.Bundle import BotBundle, make_bot
from Bots.BotState import S
from Planner.astar import hybrid_astar
from Planner.LatticeConfig import LatticeConfig
from Planner.primitives import PrimitiveTable


import argparse
import numpy as np
import pygame

lat_conf = LatticeConfig(.25)


def surface_to_numpy(surface: pygame.Surface) -> np.ndarray:
    return pygame.surfarray.array3d(surface).swapaxes(0, 1)


def init_pygame(starterkit: BotBundle, env: ObstacleEnvironment) -> tuple[Renderer, pygame.time.Clock]:
    pygame.init()
    renderer = Renderer(starterkit.bot, starterkit.goal, env)
    clock          = pygame.time.Clock()
    
    return renderer, clock

def run(
    bundle: BotBundle[S],
    environment: ObstacleEnvironment,
    manual: bool = False,
    record: bool = False,
):


    recorder        = MP4Recorder() if record else NoOpRecorder()   # screen recorder
    renderer, clock = init_pygame(bundle, environment)              # screen renderer (homemade) and clock
    state           = bundle.start                                  # starting state
    goal            = bundle.goal                                   # goal state
    bot             = bundle.bot                                    # bot type
    # immediately draw to screen before waiting for input/path planning
    renderer.render(state)


    primitives = PrimitiveTable(bot, lat_conf)

    # precompute before the loop
    path: list[S] | None = None
    path_index = 0

    if not manual:
        path = hybrid_astar(environment, bot, bundle.start, goal, lat_conf, primitives)
        if path is None:
            print("No path found.")

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        recorder.capture(renderer.screen)

        if manual:
            next_state = bot.handle_input(state, 3.0)
            next_geom = bot.footprint(next_state)
            
            if environment.is_valid_state(next_geom):
                state = next_state
        else:
            # step along precomputed path at N states per frame
            if path and path_index < len(path):
                state = path[path_index]
                path_index += 1


        renderer.render(state, path)
        clock.tick(30)


    recorder.save()





def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="valet sim", description="simulator for RBE 550 assignment"
    )
    parser.add_argument(
        "-m",
        "--manual",
        action="store_true",
        help="Choice to manually control the bots state through key presses.",
    )

    parser.add_argument(
        "-r",
        "--record",
        action="store_true",
        help="Choice to save the pygame output as an mp4 file ('./recording.mp4' by default).",
    )

    parser.add_argument(
        "--bot_type",
        choices=["point", "diff", "car", "trailer"],
        default="diff",
        help="Type of bot to simulate",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    # starting states and number of cells to clear 
    match args.bot_type:
        case "point":
            startxy = grid_to_coords(0, 0)  
            goalxy  = grid_to_coords(cfg.NUM_ROWS - 1, cfg.NUM_COLS - 1)
            cell_clearance = 1
        case "diff":
            startxy = grid_to_coords(0, 0)  
            goalxy  = grid_to_coords(cfg.NUM_ROWS - 1, cfg.NUM_COLS - 1)
            cell_clearance = 1
        case "car":
            startxy = grid_to_coords( 0.5, 0)  
            goalxy  = grid_to_coords(cfg.NUM_ROWS - 1.5, cfg.NUM_COLS - 1)
            cell_clearance = 2
        case "trailer":
            startxy = grid_to_coords( 2, 0)  
            goalxy  = grid_to_coords(cfg.NUM_ROWS - 2, cfg.NUM_COLS - 1)
            cell_clearance = 4
        case _:
            raise ValueError(args.bot_type)


    bundle = make_bot(args.bot_type, startxy, goalxy)
    environment = ObstacleEnvironment((cfg.NUM_ROWS, cfg.NUM_COLS), 0.2, cell_clearance)

    run(bundle, environment, args.manual, args.record)


if __name__ == "__main__":
    main()
