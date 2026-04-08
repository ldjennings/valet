# from Bots.BotState import BotState
from shapely.geometry.base import BaseGeometry
from simulator.obstacle import ObstacleEnvironment
from Bots.BotState import S
from Bots.Bots import Bot
from Planner.primitives import PrimitiveTable, Primitive
from .LatticeConfig import LatticeConfig
import numpy as np
import heapq

import math

from typing import Tuple


# based off of wikipedia article, particularly the pseudocode found here: https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode

def hybrid_astar(env: ObstacleEnvironment, bot: Bot, start: S, goal: S, config: LatticeConfig, prims: PrimitiveTable) -> list[S] | None:
    # Just doing euclidean distance, not considering heading at all. Works for every bot and is compliant
    #  with A* restrictions
    def h(s1: S, s2: S) -> float:
        x1, y1, *_ = s1
        x2, y2, *_ = s2
        return math.hypot(x2 - x1, y2 - y1)

    def path_is_valid(path: list[S]) -> bool:
        return all(
            env.is_valid_state(bot.footprint(s))
            for s in path
        )


    open_set:  list[Tuple[float, int, S]]   = []
    came_from: dict[S, S]                   = {}
    g_score:   dict[S, float]               = {start: 0.0}
    visited:   set[S]                       = set()
    counter                                 = 0

    heapq.heappush(open_set, (h(start, goal), counter, start))
    counter += 1 # increment after push


    while open_set:
        _, _, current = heapq.heappop(open_set)

        if current in visited:
            continue
        visited.add(current)

        if bot.is_terminal(current, goal, config):
            # TODO: last-mile connection to exact goal via BVP/Reeds-Shepp
            # for now just reconstruct path to nearest lattice node
            # reconstruct path
            # TODO: make this use primitives instead of just the points
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(current)
            path.reverse()
            return path


        for prim in prims.get(current):
            endpoint = prim.endpoint

            if not path_is_valid(prim.trajectory):
                continue

            tentative_g = g_score[current] + prim.cost

            if tentative_g < g_score.get(endpoint, float("inf")):
                came_from[endpoint] = current
                g_score[endpoint]   = tentative_g
                f                   = tentative_g + h(endpoint, goal)
                heapq.heappush(open_set, (f, counter, endpoint))
                counter            += 1                              # increment after push
    return None
