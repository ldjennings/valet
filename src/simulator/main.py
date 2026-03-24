import simulator.config as cfg
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




def run(bundle: BotBundle, environment: ObstacleEnvironment, manual:bool = False):
    state = bundle.state
    bot = bundle.bot

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
            state = bot.handle_input(state, 2.0)
        # else:
        #     state = planner.next_state(state)
            

        geom = bot.footprint(state)

        if environment.world_geom.intersects(geom):
            col = cfg.RED
        else:
            col = cfg.GREEN

        virtual_screen.fill(cfg.BLACK)
        environment.draw_grid(virtual_screen)
        draw_shape(virtual_screen, geom, col, True, cfg.BLACK)        # draw robot geometry
        # draw_shape(virtual_screen, geom, cfg.BLACK, 3)  # draw outline of the robot geometry

        draw_screen(screen, virtual_screen)

        # swithing screen, ticking forward at set rate
        pygame.display.flip()
        clock.tick(30)




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
        "--bot_type",
        choices=["point", "diff", "car", "trailer"],
        default="diff",
        help="Type of bot to simulate"
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()


    (x_s, y_s) = grid_to_coords(6,7)

    bundle = make_bot(args.bot_type, x_s, y_s)
    environment = ObstacleEnvironment((cfg.NUM_ROWS, cfg.NUM_COLS), .2, .7)
        
    run(bundle, environment, args.manual)            

    # col = cfg.GREEN
    # if environment.world_geom.intersects(geom):
    #     col = cfg.RED


    # virtual_screen.fill(cfg.BLACK)
    # environment.draw_grid(virtual_screen)
    # draw_shape(virtual_screen, geom, col, 0)        # draw robot geometry
    # draw_shape(virtual_screen, geom, cfg.BLACK, 3)  # draw outline of the robot geometry

    # draw_screen(screen, virtual_screen)

    # pygame.display.flip()
    # clock.tick(30)



if __name__ == "__main__":
    main()