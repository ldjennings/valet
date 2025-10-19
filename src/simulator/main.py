import argparse
import numpy as np
import matplotlib as plt
import pygame
from .config import *
from .obstacle import *

def grid_to_coords(x:int, y: int):
    return (x + .5, y + .5)


def scale(points, scale = CELL_PIXEL_LEN):
    return (np.array(points) * scale).tolist()


def rotated_rect_points(x, y, w, h, angle_deg, origin=None):
    """
    Returns the 4 corner points of a rectangle rotated by angle_deg around origin.
    Uses NumPy for vectorized rotation.

    Args:
        x, y: rectangle center
        w, h: width and height
        angle_deg: rotation angle in degrees
        origin: rotation origin (defaults to rectangle center)
    """
    if origin is None:
        # origin = np.array([x + w / 2, y + h / 2])
        origin = np.array([x,y])

    hw, hh = w / 2, h / 2

    # corners in clockwise order
    corners = np.array([
        [x - hw, y - hh],  # top-left
        [x + hw, y - hh],  # top-right
        [x + hw, y + hh],  # bottom-right
        [x - hw, y + hh],  # bottom-left
    ], dtype=float)

    # translate corners relative to origin
    rel_corners = corners - origin

    # rotation matrix
    theta = np.radians(angle_deg)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s],
                  [s,  c]])

    # rotate and translate back
    rotated = rel_corners @ R.T + origin
    return rotated


def draw_screen(screen: pygame.Surface, virtual_screen: pygame.Surface):
    # Preserving aspect ratio
    window_width, window_height = screen.get_size()

    scale = min(window_width / VIRTUAL_SIZE[0], window_height / VIRTUAL_SIZE[1])

    # scaling the virtual surface to the actual screen size
    scaled_surface = pygame.transform.smoothscale(
        virtual_screen,
        (int(VIRTUAL_SIZE[0] * scale), int(VIRTUAL_SIZE[1] * scale))
    )
    offset = (
        (window_width - scaled_surface.get_width()) // 2,
        (window_height - scaled_surface.get_height()) // 2
    )
    screen.fill(BLACK)  # black bars
    screen.blit(scaled_surface, offset)

def draw_grid(screen: pygame.Surface, grid: np.ndarray):

    screen.fill(BLACK)
    
    for r in range(grid.shape[0]):
        for c in range(grid.shape[1]):
            rect_points =  scale([c, r, 1, 1,])
            rect = pygame.Rect(*rect_points)
            val = grid[r, c]
            if val == square.EMPTY.value: # nothing
                color = WHITE

            elif val == square.WALL.value:  # wall
                color = BLACK

            elif val == square.HERO.value: # player
                color = GREEN

            elif val == square.GOAL.value: # goal
                color = YELLOW
            
            elif val == square.ENEMY.value: # enemy bots
                color = RED

            else:
                color = BLUE # should not be called

            pygame.draw.rect(screen, color, rect) 
            pygame.draw.rect(screen, GRAY, rect, 1) # grid lines


def setup_sim(grid: np.ndarray,robot_length: int):
    # clearing space for start position
    grid[0:robot_length, 0:robot_length] = 0

    # clearing bottom right corner
    grid[-robot_length, -robot_length:] = 0



class Robot:
    def __init__(self, max_v=1.0, max_w=1.0):
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
    
    def draw(self, screen: pygame.Surface, state):

        pixel_state = scale(state)
        
        rect = rotated_rect_points(pixel_state[0], pixel_state[1], scale(.57), scale(.7) , state[2])
        print(rect)

        pygame.draw.polygon(screen, RED, rect.tolist())



    



def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="valet sim",
        description="simulator for RBE 550 assignment"
    )

    return parser.parse_args()


def main() -> None:
    args = parse_args()

    pygame.init()

    virtual_screen = pygame.Surface(VIRTUAL_SIZE)
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.RESIZABLE)
    clock = pygame.time.Clock()

    pygame.display.set_caption("Planner Sim")

    current_grid = populate_grid((NUM_ROWS, NUM_COLS), .2)
    setup_sim(current_grid, 2)
    # draw_grid(virtual_screen, current_grid)


    diff = Robot()

    state = [*grid_to_coords(6,7), 0]


    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        virtual_screen.fill(BLACK)
        screen.fill(BLACK)

        state[2] += 2
        draw_grid(virtual_screen, current_grid)
        diff.draw(virtual_screen, state)
        draw_screen(screen, virtual_screen)

        pygame.display.flip()
        clock.tick(30)

    # paths = Planner.simulate_path(diff, [0,0,0])
    # print(f"Count: {paths}, len: {len(paths)}")
    # print(paths)


if __name__ == "__main__":
    main()