import pygame
import pygame.gfxdraw

from typing import Generic
from bots import S, Bot
from shapely.geometry import linestring
import numpy as np
import config as cfg
from environment import ObstacleEnvironment

def scale(points, scale=cfg.METERS_TO_PIXELS):
    return (np.array(points) * scale).tolist()


def draw_shape(
    surface: pygame.Surface, geom, color, outline: bool = False, outline_color=None
):
    """
    Draw a Shapely geometry on a pygame surface.

    Args:
        surface: pygame.Surface to draw on
        geom: Shapely geometry (Polygon, LineString, MultiPolygon, etc.)
        color: RGB tuple
        width: line width (0 = filled)
    """
    if geom.geom_type == "Polygon":
        coords = scale(list(geom.exterior.coords))
        pygame.draw.polygon(surface, color, coords, 0)
        if outline:
            oc = outline_color if outline_color else color
            pygame.gfxdraw.aapolygon(surface, coords, oc)

    elif geom.geom_type == "Point":
        coords = scale(list(geom.coords))
        pygame.draw.circle(surface, color, coords, 1)
        if outline:
            oc = outline_color if outline_color else color
            pygame.gfxdraw.aacircle(surface, coords[0], coords[1], 3, oc)

    elif geom.geom_type == "LineString":
        coords = scale(list(geom.coords))
        pygame.draw.lines(surface, (0, 0, 0), False, coords, 3)

    elif geom.geom_type in ("MultiPolygon", "GeometryCollection"):
        for g in geom.geoms:
            draw_shape(surface, g, color, outline, outline_color)

    else:
        raise ValueError(f"unimplemented geometry type: {geom.geom_type}")


def draw_path(surface: pygame.Surface, path: list[S], color):

    coords = [p.position() for p in path]
    lines = linestring.LineString(coords)

    draw_shape(surface, lines, color, True)

def draw_visited(surface: pygame.Surface, visited_xy: list[tuple[float, float]], color):
    for x, y in visited_xy:
        px, py = int(x * cfg.METERS_TO_PIXELS), int(y * cfg.METERS_TO_PIXELS)
        pygame.draw.circle(surface, color, (px, py), 3)


def draw_grid(
    obstacles: ObstacleEnvironment,
    screen: pygame.Surface,
) -> None:
    draw_shape(screen, obstacles.enclosure_geom, cfg.WHITE, True, cfg.RED)
    for g in obstacles.obstacles.geometries:
        draw_shape(screen, g, cfg.BLACK, True, cfg.GRAY)


def draw_frame(
    surface: pygame.Surface,
    bot: Bot,
    state: S,
    goal: S,
    environment: ObstacleEnvironment,
    path: list[S] | None = None,
    visited_xy: list[tuple[float, float]] | None = None,
) -> None:
    # surface.fill(cfg.WHITE)

    draw_grid(environment, surface)

    if visited_xy:
        draw_visited(surface, visited_xy, cfg.LIGHT_BLUE)

    if path:
        draw_path(surface, path, cfg.GRAY)

    for _, _, g in bot.footprint(goal):
        draw_shape(surface, g, cfg.YELLOW, True, cfg.BLACK)
    for _, _, g in bot.footprint(state):
        draw_shape(surface, g, cfg.GREEN, True, cfg.BLACK)

def draw_to_screen(screen: pygame.Surface, virtual_screen: pygame.Surface):
    # Preserving aspect ratio
    window_width, window_height = screen.get_size()
    scale_factor = min(window_width / cfg.VIRTUAL_SIZE[0], window_height / cfg.VIRTUAL_SIZE[1])

    # scaling the virtual surface to the actual screen size
    scaled_surface = pygame.transform.smoothscale(
        virtual_screen,
        (int(cfg.VIRTUAL_SIZE[0] * scale_factor), int(cfg.VIRTUAL_SIZE[1] * scale_factor)),
    )
    # centering the screen
    offset = (
        (window_width - scaled_surface.get_width()) // 2,
        (window_height - scaled_surface.get_height()) // 2,
    )

    screen.fill(cfg.BLACK)
    screen.blit(scaled_surface, offset)

class Renderer(Generic[S]):
    def __init__(self, bot: Bot, goal: S, env: ObstacleEnvironment):
        self.bot = bot
        self.goal = goal
        self.env = env
        self.next_frame = pygame.Surface(cfg.VIRTUAL_SIZE)
        self.screen = pygame.display.set_mode(cfg.VIRTUAL_SIZE, pygame.RESIZABLE)
        pygame.display.set_caption("Planner Sim")

    def render(self, state: S, path: list[S] | None = None, visited_xy: list[tuple[float, float]] | None = None):
        draw_frame(self.next_frame, self.bot, state, self.goal, self.env, path, visited_xy)
        draw_to_screen(self.screen, self.next_frame)
        pygame.display.flip()
