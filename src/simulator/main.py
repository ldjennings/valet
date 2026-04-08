import simulator.config as cfg
from simulator.recorder import MP4Recorder, NoOpRecorder
from simulator.obstacle import ObstacleEnvironment
from simulator.draw import draw_shape, draw_path
from simulator.utils import grid_to_coords
from Bots.Bundle import BotBundle, make_bot
from Bots.BotState import S
from Bots.Bots import Bot
from Planner.astar import hybrid_astar
from Planner.LatticeConfig import LatticeConfig
from Planner.primitives import PrimitiveTable

import argparse
import numpy as np
import pygame

lat_conf = LatticeConfig(1)


def surface_to_numpy(surface: pygame.Surface) -> np.ndarray:
    return pygame.surfarray.array3d(surface).swapaxes(0, 1)


def init_pygame() -> tuple[pygame.Surface, pygame.Surface, pygame.time.Clock]:
    pygame.init()
    virtual_screen = pygame.Surface(cfg.VIRTUAL_SIZE)
    screen         = pygame.display.set_mode(cfg.VIRTUAL_SIZE, pygame.RESIZABLE)
    clock          = pygame.time.Clock()
    pygame.display.set_caption("Planner Sim")
    return virtual_screen, screen, clock

def draw_frame(
    surface: pygame.Surface,
    bot: Bot,
    state: S,
    goal: S,
    environment: ObstacleEnvironment,
    path: list[S] | None = None,
) -> None:
    surface.fill(cfg.WHITE)
    environment.draw_grid(surface)
    if path:
        draw_path(surface, path, cfg.GRAY)
    draw_shape(surface, bot.footprint(goal), cfg.YELLOW, True, cfg.BLACK)
    draw_shape(surface, bot.footprint(state), cfg.GREEN, True, cfg.BLACK)


def run(
    bundle: BotBundle[S],
    environment: ObstacleEnvironment,
    manual: bool = False,
    record: bool = False,
):
    state = bundle.start
    goal = bundle.goal
    bot = bundle.bot

    recorder = MP4Recorder() if record else NoOpRecorder()
    virtual_screen, screen, clock = init_pygame()

    primitives = PrimitiveTable(bot, lat_conf)

    # precompute before the loop
    path: list[S] | None = None
    path_index = 0

    if not manual:
        path = hybrid_astar(environment, bot, bundle.start, goal, lat_conf, primitives)
        if path is None:
            print("No path found.")
            # instead of exiting, wait for user to close window before exiting
            # this allows debugging
            # return

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

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

        if bot.at_goal(state, goal):
            # running = False
            print("Goal state reached.")

        draw_frame(virtual_screen, bot, state, goal, environment, path)
        draw_to_screen(screen, virtual_screen)
        recorder.capture(screen)

        # swithing screen, ticking forward at set rate
        pygame.display.flip()
        clock.tick(30)

    recorder.save()


def draw_to_screen(screen: pygame.Surface, virtual_screen: pygame.Surface):
    # Preserving aspect ratio
    window_width, window_height = screen.get_size()
    scale = min(window_width / cfg.VIRTUAL_SIZE[0], window_height / cfg.VIRTUAL_SIZE[1])

    # scaling the virtual surface to the actual screen size
    scaled_surface = pygame.transform.smoothscale(
        virtual_screen,
        (int(cfg.VIRTUAL_SIZE[0] * scale), int(cfg.VIRTUAL_SIZE[1] * scale)),
    )
    # centering the screen
    offset = (
        (window_width - scaled_surface.get_width()) // 2,
        (window_height - scaled_surface.get_height()) // 2,
    )

    screen.fill(cfg.BLACK)
    screen.blit(scaled_surface, offset)


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

    match args.bot_type:
        case "point":
            robot_length = 1
        case "diff":
            robot_length = cfg.ROBOT_LENGTH_METERS
        case "car":
            robot_length = cfg.CAR_LENGTH_METERS
        case "trailer":
            robot_length = cfg.TRUCK_LENGTH_METERS + cfg.TRAILER_LENGTH_METERS + 2
        case _:
            raise ValueError(args.bot_type)

    robot_length_cell = robot_length / cfg.CELLS_TO_METERS

    # tuned by hand because I'm too lazy to come up with something smarter
    startxy = grid_to_coords(robot_length_cell / 2, 0.25)  
    goalxy  = grid_to_coords(cfg.NUM_ROWS - 1 - (robot_length_cell / 2), cfg.NUM_COLS - 1)

    bundle = make_bot(args.bot_type, startxy, goalxy)
    environment = ObstacleEnvironment((cfg.NUM_ROWS, cfg.NUM_COLS), 0.2, robot_length + 1)

    run(bundle, environment, args.manual, args.record)


if __name__ == "__main__":
    main()
