import simulator.config as cfg
from simulator.recorder import MP4Recorder, NoOpRecorder
from environment.obstacle import ObstacleEnvironment
from simulator.render import Renderer
from Bots.Bundle import BotBundle, make_bot
from Bots.BotState import S
from Planner.astar import lattice_astar
from Planner.LatticeConfig import LatticeConfig
from Planner.primitives import PrimitiveTable





import argparse
import pygame

lat_conf = LatticeConfig(.25)


def init_pygame(bundle: BotBundle, env: ObstacleEnvironment) -> tuple[Renderer, pygame.time.Clock]:
    pygame.init()

    renderer = Renderer(bundle.bot, bundle.goal, env)
    clock    = pygame.time.Clock()
    
    return renderer, clock

def run(
    bundle: BotBundle[S],
    environment: ObstacleEnvironment,
    manual: bool = False,
    record: bool = False,
):
    ## Initializing Variables ##
    recorder        = MP4Recorder() if record else NoOpRecorder()   # screen recorder, no-ops unless render is true
    renderer, clock = init_pygame(bundle, environment)              # screen renderer (homemade) and clock
    state           = bundle.start                                  # starting state
    goal            = bundle.goal                                   # goal state
    bot             = bundle.bot                                    # bot type

    # immediately draw to screen before waiting for input/path planning, makes debugging easier
    renderer.render(state)

    ## Generating a kinematically correct path ##
    # bind variables outside of conditionals so pylance doesnt yell at me
    path: list[S] | None = None
    path_index = 0

    # calculate path planning before loop
    if not manual:
        # generate navigation primitives
        primitives = PrimitiveTable(bot, lat_conf)
        
        # do path planning
        path = lattice_astar(environment, bot, bundle.start, goal, lat_conf, primitives)
        if path is None:
            print("No path found.")

    ## Main Pygame loop ##
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        

        if manual: # manual mode, control robot with keyboard
            next_state = bot.handle_input(state, 3.0)
            next_geom = bot.footprint(next_state)

            if environment.is_valid_state(next_geom):
                state = next_state
        else:
            # Animate by stepping along precomputed path at N states per frame
            if path and path_index < len(path):
                state = path[path_index]
                path_index += 1


        renderer.render(state, path)
        recorder.capture(renderer.screen)
        clock.tick(30)

    recorder.save()

# helper method for initial positions 
def grid_to_coords(x_cell, y_cell, center=True) -> tuple[float, float]:
    if center:
        x_cell += 0.5
        y_cell += 0.5

    return (x_cell * cfg.CELLS_TO_METERS, y_cell * cfg.CELLS_TO_METERS) 


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
        "bot_type",
        nargs="?",
        choices=["point", "diff", "car", "trailer"],
        default="diff",
        help="Type of bot to simulate",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_args()

    # starting states and number of cells to clear on obstacle grid
    trailer = (args.bot_type == "trailer")

    start_goal = {
        "point":   ((0, 0),     (-3.5, -1)),
        "diff":    ((0, 0),     (-3.5, -1)),
        "car":     ((0.5, 0),   (-3.55, -1)),
        "trailer": ((2, 0),     (-4, -1)),
    }

    if args.bot_type not in start_goal:
        raise ValueError(args.bot_type)

    (sr, sc), (gr, gc) = start_goal[args.bot_type]
    startxy = grid_to_coords(sr, sc)
    goalxy  = grid_to_coords(cfg.NUM_ROWS + gr, cfg.NUM_COLS + gc)


    bundle = make_bot(args.bot_type, startxy, goalxy)
    environment = ObstacleEnvironment((cfg.NUM_ROWS, cfg.NUM_COLS), cfg.CELLS_TO_METERS, 0.1, trailer)

    run(bundle, environment, args.manual, args.record)


if __name__ == "__main__":
    main()
