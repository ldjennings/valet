from typing import Generic
import simulator.config as cfg
from Bots import Bot, S
from environment.obstacle import ObstacleEnvironment
from simulator.draw import draw_grid, draw_path, draw_shape, draw_visited

import pygame

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