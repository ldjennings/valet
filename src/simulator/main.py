import argparse
import numpy as np
import matplotlib as plt
import pygame
import simulator.config as cfg
from .obstacle import ObstacleEnvironment, draw_shape, grid_to_coords
from shapely.affinity import rotate, translate
from shapely import Polygon, Geometry
from shapely.geometry import box
from typing import Sequence






def make_rect(center: tuple[float, float], w: float, h:float, angle: float = 0) -> Polygon:
    cx, cy = center
    initial = box(-w/2, -h/2, w/2, h/2)
    rot = rotate(initial, angle)
    positioned = translate(rot, cx, cy)

    return positioned




keymap = {
    pygame.K_LEFT:  (-.5, 0, 0),
    pygame.K_RIGHT: ( .5, 0, 0),
    pygame.K_UP:    ( 0,-.5, 0),
    pygame.K_DOWN:  ( 0, .5, 0),
    pygame.K_a:     ( 0, 0,-5),
    pygame.K_r:     ( 0, 0, 5),
    pygame.K_w:     ( 0, 0, 0, -5),
    pygame.K_f:     ( 0, 0, 0, 5),
}

def handle_input(state, speed=2):
    keys = pygame.key.get_pressed()
    for k, delta in keymap.items():
        if keys[k]:
            for i, d in enumerate(delta):
                if len(state) > i:
                    state[i] += d * speed




# def rotated_rect_points(x, y, w, h, angle_deg, origin=None):
#     """
#     Returns the 4 corner points of a rectangle rotated by angle_deg around origin.
#     Uses NumPy for vectorized rotation.

#     Args:
#         x, y: rectangle center
#         w, h: width and height
#         angle_deg: rotation angle in degrees
#         origin: rotation origin (defaults to rectangle center)
#     """
#     if origin is None:
#         # origin = np.array([x + w / 2, y + h / 2])
#         origin = np.array([x,y])

#     hw, hh = w / 2, h / 2

#     # corners in clockwise order
#     corners = np.array([
#         [x - hw, y - hh],  # top-left
#         [x + hw, y - hh],  # top-right
#         [x + hw, y + hh],  # bottom-right
#         [x - hw, y + hh],  # bottom-left
#     ], dtype=float)

#     # translate corners relative to origin
#     rel_corners = corners - origin

#     # rotation matrix
#     theta = np.radians(angle_deg)
#     c, s = np.cos(theta), np.sin(theta)
#     R = np.array([[c, -s],
#                   [s,  c]])

#     # rotate and translate back
#     rotated = rel_corners @ R.T + origin
#     return rotated


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





DifferentialDriveState = Sequence[float]

goal_state = [(cfg.NUM_COLS -.5) * cfg.CELLS_TO_METERS, (cfg.NUM_ROWS-.5) * cfg.CELLS_TO_METERS, -90]
class DifferentialDriveModel:
    """Defines kinematics and geometry for a differential-drive robot."""


    # ---- Motion model ----
    def propagate(self, state: DifferentialDriveState, control: tuple[float, float], dt: float) -> np.ndarray:
        """Propagate one step using Euler integration."""
        assert len(state) >= 3, "State must have [x, y, theta]"
        x, y, theta = state
        v, w = control
        
        return np.array([
            x + v * np.cos(theta) * dt,
            y + v * np.sin(theta) * dt,
            theta + w * dt
        ])

    @staticmethod
    def distance(s1: DifferentialDriveState, s2: DifferentialDriveState) -> float:
        """Distance metric that combines xy distance and heading difference"""
        dx, dy = s2[0]-s1[0], s2[1]-s1[1]
        dtheta = np.arctan2(np.sin(s2[2]-s1[2]), np.cos(s2[2]-s1[2]))
        return np.sqrt(dx**2 + dy**2) + 0.1 * abs(dtheta)

    @staticmethod
    def check_collision(state: DifferentialDriveState, geom: Geometry) -> bool:
        """Check if robot geometry intersects with supplied geometry"""
        x, y, a = state
        rect = make_rect((x, y), cfg.ROBOT_WIDTH_METERS, cfg.ROBOT_LENGTH_METERS, a)
        return rect.intersects(geom)

    @staticmethod
    def draw(screen: pygame.Surface, state: DifferentialDriveState, color: tuple[int, int, int] = cfg.RED) -> None:
        """Draws differential drive robot on screen as a rectangle"""
        x, y, a = state
        rect = make_rect((x, y), cfg.ROBOT_WIDTH_METERS, cfg.ROBOT_LENGTH_METERS, a)
        draw_shape(screen, rect, color)
        draw_shape(screen, rect, cfg.BLACK, 1)





def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="valet sim",
        description="simulator for RBE 550 assignment"
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

    diff = DifferentialDriveModel()

    state = [*grid_to_coords(6,7), 0]

    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                
        handle_input(state)
        # virtual_screen.fill(cfg.WHITE)
        virtual_screen.fill(cfg.BLACK)

        # state[2] += 2
        x, y, a = state

        col = cfg.GREEN

        # truck_geom = truck_trailer_geom(*state)
        
        if diff.check_collision(state, environment.world_geom):
            col = cfg.RED

        environment.draw_grid(virtual_screen)
        diff.draw(virtual_screen, state, col)
        DifferentialDriveModel.draw(virtual_screen, goal_state, cfg.GREEN)

        draw_screen(screen, virtual_screen)



        pygame.display.flip()
        clock.tick(30)



if __name__ == "__main__":
    main()