"""
Pygame rendering for the simulator.

All drawing logic lives here: grid/obstacles, visited nodes, planned path,
bot footprints at current state and goal. The Renderer class owns the pygame
surfaces and exposes a single render() call per frame.
"""

import pygame
import pygame.gfxdraw

from typing import Generic
from bots import S, Bot
from bots.geometry import LineFootprint
from shapely.geometry import linestring
import numpy as np
import config as cfg
from environment import ObstacleEnvironment


# ── Drawing primitives ────────────────────────────────────────────────────────

def scale(points, scale=cfg.METERS_TO_PIXELS):
    """Convert world coordinates (meters) to pixel coordinates."""
    return (np.array(points) * scale).tolist()


def draw_shape(
    surface: pygame.Surface, geom, color, outline: bool = False, outline_color=None
):
    """
    Draw a Shapely geometry on a pygame surface.

    Args:
        surface: pygame.Surface to draw on
        geom: Shapely geometry (Polygon, LineString, MultiPolygon, etc.)
        color: RGB fill color tuple
        outline: if True, draw an antialiased outline
        outline_color: RGB outline color; defaults to color if not given
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
    """Draw the planned path as a polyline through each state's position."""
    coords = [p.position() for p in path]
    lines = linestring.LineString(coords)
    draw_shape(surface, lines, color, True)


def draw_visited(surface: pygame.Surface, visited_xy: list[tuple[float, float]], color):
    """Draw a dot at each visited node position (for A* debug visualisation)."""
    for x, y in visited_xy:
        px, py = int(x * cfg.METERS_TO_PIXELS), int(y * cfg.METERS_TO_PIXELS)
        pygame.draw.circle(surface, color, (px, py), 3)


# ── Composite frame drawing ───────────────────────────────────────────────────

def draw_grid(obstacles: ObstacleEnvironment, screen: pygame.Surface) -> None:
    """Draw the environment boundary and all obstacle cells."""
    draw_shape(screen, obstacles.enclosure_geom, cfg.WHITE, True, cfg.GRAY)
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
    """Compose a full frame: grid, visited nodes, path, goal, and current bot."""
    draw_grid(environment, surface)

    if visited_xy:
        draw_visited(surface, visited_xy, cfg.LIGHT_BLUE)

    if path:
        draw_path(surface, path, cfg.GRAY)

    for entry in bot.footprint(goal):
        if isinstance(entry, LineFootprint):
            pygame.draw.line(surface, cfg.BLACK, scale((entry.x0, entry.y0)), scale((entry.x1, entry.y1)), 3)
        else:
            _, _, g, _ = entry
            draw_shape(surface, g, cfg.YELLOW, True, cfg.BLACK)
    for entry in bot.footprint(state):
        if isinstance(entry, LineFootprint):
            pygame.draw.line(surface, cfg.BLACK, scale((entry.x0, entry.y0)), scale((entry.x1, entry.y1)), 3)
        else:
            _, _, g, _ = entry
            draw_shape(surface, g, cfg.GREEN, True, cfg.BLACK)


def draw_to_screen(screen: pygame.Surface, virtual_screen: pygame.Surface):
    """Scale the virtual surface to fit the window, preserving aspect ratio."""
    window_width, window_height = screen.get_size()
    scale_factor = min(window_width / cfg.VIRTUAL_SIZE[0], window_height / cfg.VIRTUAL_SIZE[1])

    scaled_surface = pygame.transform.smoothscale(
        virtual_screen,
        (int(cfg.VIRTUAL_SIZE[0] * scale_factor), int(cfg.VIRTUAL_SIZE[1] * scale_factor)),
    )
    offset = (
        (window_width - scaled_surface.get_width()) // 2,
        (window_height - scaled_surface.get_height()) // 2,
    )

    screen.fill(cfg.BLACK)
    screen.blit(scaled_surface, offset)


# ── Renderer ──────────────────────────────────────────────────────────────────

class Renderer(Generic[S]):
    """Owns the pygame display surfaces and drives per-frame rendering."""

    def __init__(self, bot: Bot, goal: S, env: ObstacleEnvironment):
        self.bot = bot
        self.goal = goal
        self.env = env
        self.next_frame = pygame.Surface(cfg.VIRTUAL_SIZE)
        self.screen = pygame.display.set_mode(cfg.VIRTUAL_SIZE, pygame.RESIZABLE)
        pygame.display.set_caption("Planner Sim")

    def render(self, state: S, path: list[S] | None = None, visited_xy: list[tuple[float, float]] | None = None):
        """Draw a frame for the given state and push it to the display."""
        draw_frame(self.next_frame, self.bot, state, self.goal, self.env, path, visited_xy)
        draw_to_screen(self.screen, self.next_frame)
        pygame.display.flip()
