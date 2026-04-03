import pygame
import pygame.gfxdraw
from simulator.utils import scale


def draw_shape(surface: pygame.Surface, geom, color, outline: bool = False, outline_color=None):
    """
    Draw a Shapely geometry on a pygame surface.

    Args:
        surface: pygame.Surface to draw on
        geom: Shapely geometry (Polygon, LineString, MultiPolygon, etc.)
        color: RGB tuple
        width: line width (0 = filled)
    """
    if geom.geom_type == 'Polygon':
        coords = scale(list(geom.exterior.coords))
        pygame.draw.polygon(surface, color, coords, 0)
        if outline:
            oc = outline_color if outline_color else color
            pygame.gfxdraw.aapolygon(surface, coords, oc)
    elif geom.geom_type == 'Point':
        coords = scale(list(geom.coords))
        pygame.draw.circle(surface, color, coords, 1)
        if outline:
            oc = outline_color if outline_color else color
            pygame.gfxdraw.aacircle(surface, coords[0], coords[1],3, oc)
    elif geom.geom_type == 'LineString':
        coords = scale(list(geom.coords))
        pygame.draw.lines(surface, (0, 0, 0), False, coords, 3)
    elif geom.geom_type in ('MultiPolygon', 'GeometryCollection'):
        for g in geom.geoms:
            draw_shape(surface, g, color, outline, outline_color)
    else:
        ValueError(f"unimplemented geometry type: {geom.geom_type}")
    
