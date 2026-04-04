import simulator.config as cfg
import simulator.recorder as rec
from simulator.obstacle import ObstacleEnvironment
from simulator.draw import draw_shape
from simulator.utils import grid_to_coords
from Bots.Bundle import BotBundle, make_bot

import argparse
import numpy as np
import pygame




def wrap_to_pi(ang_rad: float):
    """Wraps arbitrary radian angle to pi to negative pi"""

    return (ang_rad + np.pi) % (2 * np.pi) - np.pi


def surface_to_numpy(surface: pygame.Surface) -> np.ndarray:
    return pygame.surfarray.array3d(surface).swapaxes(0, 1)




def run(bundle: BotBundle, environment: ObstacleEnvironment, manual: bool = False, record: bool = False):
    state = bundle.start
    goal = bundle.goal
    bot = bundle.bot

    if record:
        recorder = rec.MP4Recorder()
    else:
        recorder = rec.NoOpRecorder()

    # pygame boilerplate
    pygame.init()
    virtual_screen = pygame.Surface(cfg.VIRTUAL_SIZE)
    screen = pygame.display.set_mode(cfg.VIRTUAL_SIZE, pygame.RESIZABLE)
    clock = pygame.time.Clock()
    pygame.display.set_caption("Planner Sim")

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        if manual:
            state = bot.handle_input(state, 6.0)
        # else:
        #     state = planner.next_state(state)

        if bot.at_goal(state, goal):
            running = False
            print("Goal state reached.")
            

        geom = bot.footprint(state)

        if environment.world_geom.intersects(geom):
            col = cfg.RED
        else:
            col = cfg.GREEN

        virtual_screen.fill(cfg.WHITE)
        environment.draw_grid(virtual_screen)
        draw_shape(virtual_screen, bot.footprint(goal), cfg.YELLOW, True, cfg.BLACK) # draw goal before robot so robot overlaps it


        draw_shape(virtual_screen, geom, col, True, cfg.BLACK)        # draw robot geometry        

        draw_screen(screen, virtual_screen)
        recorder.capture(screen)

        # swithing screen, ticking forward at set rate
        pygame.display.flip()
        clock.tick(30)

    recorder.save()




def draw_screen(screen: pygame.Surface, virtual_screen: pygame.Surface):
    # Preserving aspect ratio
    window_width, window_height = screen.get_size()
    scale = min(window_width / cfg.VIRTUAL_SIZE[0], window_height / cfg.VIRTUAL_SIZE[1])

    # scaling the virtual surface to the actual screen size
    scaled_surface = pygame.transform.smoothscale(
        virtual_screen,
        (int(cfg.VIRTUAL_SIZE[0] * scale), int(cfg.VIRTUAL_SIZE[1] * scale))
    )
    # centering the screen
    offset = (
        (window_width - scaled_surface.get_width()) // 2,
        (window_height - scaled_surface.get_height()) // 2
    )

    screen.fill(cfg.BLACK)
    screen.blit(scaled_surface, offset)



def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="valet sim",
        description="simulator for RBE 550 assignment"
    )
    parser.add_argument(
        "-m",
        "--manual",
        action="store_true",
        help="Choice to manually control the bots state through key presses."
    )

    parser.add_argument(
        "-r",
        "--record",
        action="store_true",
        help="Choice to save the pygame output as an mp4 file ('./recording.mp4' by default)."
    )

    parser.add_argument(
        "--bot_type",
        choices=["point", "diff", "car", "trailer"],
        default="diff",
        help="Type of bot to simulate"
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()


    match args.bot_type:
        case "point":   robot_length = 1
        case "diff":    robot_length = cfg.ROBOT_LENGTH_METERS
        case "car":     robot_length = cfg.CAR_LENGTH_METERS
        case "trailer": robot_length = cfg.TRUCK_LENGTH_METERS + cfg.TRAILER_LENGTH_METERS + 2
        case _:         raise ValueError(args.bot_type)

    robot_length_cell = robot_length / cfg.CELLS_TO_METERS


    (x_s, y_s) = grid_to_coords(robot_length_cell / 2 , .25) # tuned by hand because I'm too lazy to come up with something smarter
    (x_g, y_g) = grid_to_coords(cfg.NUM_ROWS - 1 - (robot_length_cell / 2), cfg.NUM_COLS - .75)

    bundle = make_bot(args.bot_type, (x_s, y_s), (x_g, y_g))
    environment = ObstacleEnvironment((cfg.NUM_ROWS, cfg.NUM_COLS), .2, robot_length+1)
        
    run(bundle, environment, args.manual, args.record)   




if __name__ == "__main__":
    main()