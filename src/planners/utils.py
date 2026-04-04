# # from Bots.BotState import BotState
# from shapely.geometry.base import BaseGeometry
# from simulator.obstacle import ObstacleEnvironment
# from Bots.BotState import S
# from Bots.Bots import Bot, check_collision
# import numpy as np
# import heapq

# from dataclasses import dataclass
# import math


# @dataclass
# class LatticeConfig:
#     spacing: float
#     terminal_radius: float
#     angular_spacing: float = math.pi / 4
#     angular_terminal: float = math.pi / 8
#     trailer_angular_terminal: float = math.pi / 6


# # based off of wikipedia article, particularly the pseudocode found here: https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode

# def astar(grid: ObstacleEnvironment, bot: Bot, start: S, goal: S, config: LatticeConfig) -> list[S] | None:
#     grid_width, grid_height = grid.dims
#     open_set = []
#     heapq.heappush(open_set, (0, start))
#     came_from = {}
#     g_score = {start: 0}

#     def h(p1, p2):
#         # Manhattan distance
#         return abs(p1[0]-p2[0]) + abs(p1[1]-p2[1])

#     while open_set:
#         _, current = heapq.heappop(open_set)

#         if bot.is_terminal(current, goal, config):
#             # TODO: last-mile connection to exact goal via BVP/Reeds-Shepp
#             # for now just reconstruct path to nearest lattice node
#             # reconstruct path
#             path = []
#             while current in came_from:
#                 path.append(current)
#                 current = came_from[current]
#             path.reverse()
#             return path


#         for neighbor in bot.primitives(current, config):
#             if :  # valid neighbor check
#                 tentative_g = g_score[current] + 1
#                 if tentative_g < g_score.get(neighbor, float("inf")):
#                     came_from[neighbor] = current
#                     g_score[neighbor] = tentative_g
#                     f_score = tentative_g + h(neighbor, goal)
#                     heapq.heappush(open_set, (f_score, neighbor))
#     return None
