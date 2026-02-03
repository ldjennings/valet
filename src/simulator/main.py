import argparse
import numpy as np
import pygame
import simulator.config as cfg
from simulator.obstacle import ObstacleEnvironment, draw_shape, grid_to_coords
from simulator.robot_geometry import rectangular_robot_geom, truck_trailer_geom
from simulator.BotState import BotState




def wrap_to_pi(ang_rad: float):
    """Wraps arbitrary radian angle to pi to negative pi"""

    return (ang_rad + np.pi) % (2 * np.pi) - np.pi







def handle_input(state: BotState, speed=2):
    keymap = {
        pygame.K_LEFT:  (-0.5,    0,  0,  0),
        pygame.K_RIGHT: ( 0.5,    0,  0,  0),
        pygame.K_UP:    (   0, -0.5,  0,  0),
        pygame.K_DOWN:  (   0,  0.5,  0,  0),
        pygame.K_a:     (   0,    0, -5,  0),
        pygame.K_d:     (   0,    0,  5,  0),
        pygame.K_w:     (   0,    0,  0, -5),
        pygame.K_s:     (   0,    0,  0,  5),
    }

    keys = pygame.key.get_pressed()
    new_state = state

    for k, delta in keymap.items():
        if keys[k]:
            new_state = new_state.add_delta(delta, speed)

    return new_state




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





# DifferentialDriveState = Sequence[float]

# goal_state = [(cfg.NUM_COLS -.5) * cfg.CELLS_TO_METERS, (cfg.NUM_ROWS-.5) * cfg.CELLS_TO_METERS, -90]
# class DifferentialDriveModel:
#     """Defines kinematics and geometry for a differential-drive robot."""

#     # ---- Motion model ----
#     def propagate(self, state: DifferentialDriveState, control: tuple[float, float], dt: float) -> np.ndarray:
#         """Propagate one step using Euler integration."""
#         assert len(state) == 3, "State should be [x, y, theta]"
#         x, y, theta = state
#         v, w = control
        
#         return np.array([
#             x + v * np.cos(theta) * dt,
#             y + v * np.sin(theta) * dt,
#             theta + w * dt
#         ])
    
#     @staticmethod
#     def generate_trajectory(start, goal, w:float = 1.0, v:float = 1.0, dt=0.1):
#         # make inputs nicer
#         x_start, y_start, t_start = start
#         x_goal, y_goal, t_goal = goal

#         t_start = wrap_to_pi(t_start)
#         t_goal = wrap_to_pi(t_goal)

#         traj = []

#         dx = x_goal - x_start
#         dy = y_goal - y_start
#         theta_to_goal = np.arctan2(dy, dx)
#         dtheta1 = wrap_to_pi(theta_to_goal - t_start)

#         # first turning stage
#         steps_to_rotate = max(1, int(abs(dtheta1) / (w*dt)))

#         for i in range(steps_to_rotate):
#             theta = t_start + dtheta1 * (i+1)/steps_to_rotate
#             traj.append((x_start, y_start, wrap_to_pi(theta)))

#         # drive straignt to goal
#         distance = np.hypot(dx, dy)
#         n_steps_straight = max(1, int(distance / (v*dt)))

#         for i in range(n_steps_straight):
#             x = x_start + dx * (i+1)/n_steps_straight
#             y = y_start + dy * (i+1)/n_steps_straight
#             traj.append((x, y, theta_to_goal))

#         # rotate to goal heading
#         dtheta2 = wrap_to_pi(t_goal - theta_to_goal)
#         steps_to_rotate2 = max(1, int(abs(dtheta2) / (w*dt)))

#         for i in range(steps_to_rotate2):
#             theta = theta_to_goal + dtheta2 * (i+1)/steps_to_rotate2
#             traj.append((x_goal, y_goal, wrap_to_pi(theta)))

#         return np.array(traj)



#     @staticmethod
#     def distance(s1: DifferentialDriveState, s2: DifferentialDriveState) -> float:
#         """Distance metric that combines xy distance and heading difference"""
#         x1, y1, t1 = s1
#         x2, y2, t2 = s2

#         t1 = wrap_to_pi(t1)
#         t2 = wrap_to_pi(t2)

#         dx, dy = x2 - x1, y2-y1
#         dtheta = np.arctan2(np.sin(t2-t1), np.cos(t2-t1))
#         return np.sqrt(dx**2 + dy**2) + 0.1 * abs(dtheta)

#     @staticmethod
#     def check_collision(state: DifferentialDriveState, geom: Geometry) -> bool:
#         """Check if robot geometry intersects with supplied geometry"""
#         x, y, a = state
#         rect = make_rect((x, y), cfg.ROBOT_WIDTH_METERS, cfg.ROBOT_LENGTH_METERS, a)
#         return rect.intersects(geom)

#     @staticmethod
#     def draw(screen: pygame.Surface, state: DifferentialDriveState, color: tuple[int, int, int] = cfg.RED) -> None:
#         """Draws differential drive robot on screen as a rectangle"""
#         x, y, a = state
#         rect = make_rect((x, y), cfg.ROBOT_WIDTH_METERS, cfg.ROBOT_LENGTH_METERS, a)
#         draw_shape(screen, rect, color)
#         draw_shape(screen, rect, cfg.BLACK, 1)





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
        "--geom",
        choices=["manual", "diff", "car", "trailer"],
        default="diff",
        help="Bot Geometry to simulate"
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    pygame.init()

    virtual_screen = pygame.Surface(cfg.VIRTUAL_SIZE)
    screen = pygame.display.set_mode(cfg.VIRTUAL_SIZE, pygame.RESIZABLE)
    clock = pygame.time.Clock()

    pygame.display.set_caption("Planner Sim")

    environment = ObstacleEnvironment((cfg.NUM_ROWS, cfg.NUM_COLS), .2, .7)

    starting_xy = grid_to_coords(6,7)

    state = BotState(starting_xy[0], starting_xy[1], 0, 0)


    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            

        if args.manual:
            state = handle_input(state)
        else:
            raise Exception(f"Planner Not implemented yet for geometry: {args.geom}")
        


        if args.geom == "trailer":
            geom = truck_trailer_geom(state)
        elif args.geom == "car":
            geom = rectangular_robot_geom(state, cfg.CAR_WIDTH_METERS, cfg.CAR_LENGTH_METERS)
        elif args.geom == "diff":
            geom = rectangular_robot_geom(state, cfg.ROBOT_WIDTH_METERS, cfg.ROBOT_LENGTH_METERS)
        else:
            raise ValueError(f"attempted to generate unknown robot geometry: {args.geom}")
        

        col = cfg.GREEN
        if environment.world_geom.intersects(geom):
            col = cfg.RED


        virtual_screen.fill(cfg.BLACK)
        environment.draw_grid(virtual_screen)
        draw_shape(virtual_screen, geom, col, 0)        # draw robot geometry
        draw_shape(virtual_screen, geom, cfg.BLACK, 3)  # draw outline of the robot geometry

        draw_screen(screen, virtual_screen)

        pygame.display.flip()
        clock.tick(30)



if __name__ == "__main__":
    main()