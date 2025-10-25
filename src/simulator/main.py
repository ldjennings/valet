import argparse
import numpy as np
import matplotlib as plt
import pygame
import simulator.config as cfg
from .obstacle import ObstacleEnvironment, scale, draw_shape
from shapely.affinity import rotate, translate
from shapely import Polygon, MultiPolygon
from shapely.geometry import box





def grid_to_coords(x_cell, y_cell, center=True):
    if center:
        x_cell +=.5
        y_cell +=.5
    
    return scale((x_cell, y_cell), cfg.CELLS_TO_METERS)


def make_rect(center: tuple[float, float], w: float, h:float, angle: float = 0) -> Polygon:
    cx, cy = center
    initial = box(-w/2, -h/2, w/2, h/2)
    rot = rotate(initial, angle)
    positioned = translate(rot, cx, cy)

    return positioned


# def draw_polygon(surface: pygame.Surface, polygon: Polygon, color, width=0):
#     # Convert Shapely coordinates (x, y) into a list of tuples
#     points = [(x, y) for x, y in polygon.exterior.coords]
#     points = scale(points)
#     # print(f"calling draw polygon with the following points: {points}")
#     pygame.draw.polygon(surface, color, points, width)






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
    offset = (
        (window_width - scaled_surface.get_width()) // 2,
        (window_height - scaled_surface.get_height()) // 2
    )
    screen.fill(cfg.BLACK)  # black bars
    screen.blit(scaled_surface, offset)





class Robot:
    goal_state = [(cfg.NUM_COLS -.5) * cfg.CELLS_TO_METERS, (cfg.NUM_ROWS-.5) * cfg.CELLS_TO_METERS, -90]

    def __init__(self, max_v=1.0, max_w=1.0, ):
        self.max_v = max_v
        self.max_w = max_w

    def propagate(self, state, control, dt):
        x, y, theta = state
        v, w = control

        # integrate using simple Euler
        x_next = x + v * np.cos(theta) * dt
        y_next = y + v * np.sin(theta) * dt
        theta_next = theta + w * dt

        return np.array([x_next, y_next, theta_next])

    def distance(self, s1, s2):
        # Euclidean distance in x,y + orientation difference
        dx, dy = s2[0]-s1[0], s2[1]-s1[1]
        dtheta = np.arctan2(np.sin(s2[2]-s1[2]), np.cos(s2[2]-s1[2]))
        return np.sqrt(dx**2 + dy**2) + 0.1 * abs(dtheta)
    
    def draw(self, screen: pygame.Surface, state, color: tuple[int,int,int] = cfg.RED):
        x, y, a = state

        
        # rotate rectangle around center 
        rect = make_rect((x, y), cfg.ROBOT_WIDTH_METERS, cfg.ROBOT_LENGTH_METERS, a)

        # rect = make_rect((x, y), 8, 8, a)
        # draw robot
        draw_shape(screen, rect, color)
        # print(f"drawing thingy at: {rect.exterior.coords.xy}")
        draw_shape(screen, rect, cfg.BLACK, 1)

    # def has_collided(self, state, grid: ObstacleEnvironment) -> bool:
    #     x, y, a = state
    #     # If it is inside a cell
    #     if grid.get_cell_val(x, y) is 1:
    #         return True
        
    #     def OBB_collision(rect1, rect2) -> bool:


        

    #     return True



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

    diff = Robot()

    state = [*grid_to_coords(6,7), 0]


    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                
        # virtual_screen.fill(cfg.WHITE)
        screen.fill(cfg.BLACK)

        state[2] += 2

        environment.draw_grid(virtual_screen)
        diff.draw(virtual_screen, state)
        diff.draw(virtual_screen, Robot.goal_state, cfg.GREEN)
        draw_screen(screen, virtual_screen)

        x, y, a = state

        
        # rotate rectangle around center 
        if make_rect((x, y), cfg.ROBOT_WIDTH_METERS, cfg.ROBOT_LENGTH_METERS, a).intersects(environment.world_geom):
            print("collides!!!")

        pygame.display.flip()
        clock.tick(30)



if __name__ == "__main__":
    main()