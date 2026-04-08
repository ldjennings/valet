import pygame
import pygame.gfxdraw
from Bots.Bots import Bot
from simulator.obstacle import ObstacleEnvironment
from simulator.utils import scale
from Bots.BotState import S
from shapely.geometry import linestring
import simulator.config as cfg


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
        ValueError(f"unimplemented geometry type: {geom.geom_type}")


def draw_path(surface: pygame.Surface, path: list[S], color):
    def extract_xy(state: S):
        x, y, *_ = state
        return (x,y)


    coords = map(extract_xy, path)


    lines = linestring.LineString(coords)

    draw_shape(surface, lines, color, True)


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

def render(screen: pygame.Surface, next_frame: pygame.Surface, bot: Bot, state: S, goal: S, env: ObstacleEnvironment, path: list[S] | None):
    draw_frame(next_frame, bot, state, goal, env, path)
    draw_to_screen(screen, next_frame)
    pygame.display.flip()


# class Renderer()

