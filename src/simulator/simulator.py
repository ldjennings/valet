"""
Main simulation loop.

Orchestrates the bot, environment, planner, and rendering.
Each call to run() executes one full planning + animation session.
"""

import pygame
from dataclasses import dataclass
from typing import Generic

import config as cfg
from bots import S
from environment import ObstacleEnvironment
from planner import hybrid_astar, HybridConfig
from simulator.renderer import Renderer
from simulator.recorder import MP4Recorder, NoOpRecorder
from simulator.Bundle import BotBundle


class Simulator(Generic[S]):
    """Orchestrates planning, animation, and optional recording for one session."""

    def __init__(
        self,
        bundle: BotBundle[S],
        environment: ObstacleEnvironment,
        config: HybridConfig = HybridConfig(),
    ) -> None:
        self.bundle      = bundle
        self.environment = environment
        self.config      = config

    def run(self, manual: bool = False, record: bool = False) -> None:
        """Run the simulation loop.

        In planning mode, runs hybrid A* first then animates the resulting path.
        In manual mode, the bot is driven by keyboard input each frame.
        Recording captures every frame where a path exists and saves on exit.
        """
        bot   = self.bundle.bot
        state = self.bundle.start
        goal  = self.bundle.goal

        pygame.init()
        renderer = Renderer(bot, goal, self.environment)
        recorder = MP4Recorder(fps= cfg.FPS) if record else NoOpRecorder()
        clock    = pygame.time.Clock()

        renderer.render(state)

        path: list[S] | None = None
        visited_xy: list[tuple[float, float]] | None = None
        path_index = 0

        if not manual:
            result     = hybrid_astar(self.environment, bot, state, goal, self.config, debug=True)
            path       = result.path
            visited_xy = result.visited_xy or None
            if path is None:
                print("No path found.")

        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            if manual:
                next_state = bot.handle_input(state, 3.0)
                if self.environment.is_valid_state(bot.footprint(next_state)):
                    state = next_state
            else:
                if path and path_index < len(path):
                    state = path[path_index]
                    path_index += 1

            renderer.render(state, path, visited_xy)
            if path:
                recorder.capture(renderer.screen)

            clock.tick(cfg.FPS)

        if path:
            recorder.save()
