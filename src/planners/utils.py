from simulator.BotState import BotState
from shapely.geometry.base import BaseGeometry
from simulator.obstacle import ObstacleEnvironment
import numpy as np
import heapq


# based off of wikipedia article, particularly the pseudocode found here: https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode

def astar(grid: ObstacleEnvironment, start: BotState, goal: BotState) -> list[BotState] | None:
    grid_width, grid_height = grid.dims
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}

    def h(p1, p2):
        # Manhattan distance
        return abs(p1[0]-p2[0]) + abs(p1[1]-p2[1])
    
    def get_neighbors(row: int, column: int) -> list[tuple[int,int]]:
        neighbors = []
        for dr, dc in [(1,0), (-1,0), (0,1), (0,-1)]:
            nr, nc = row + dr, column + dc
            neighbor = (nr, nc)

            if 0 <= nr < grid_width and 0 <= nc < grid_height:
                neighbors.append(neighbor)
        
        return neighbors

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            # reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        r, c = current
        for neighbor in get_neighbors(r,c):
            if grid[neighbor] != 1 and grid[neighbor] != 4:
                tentative_g = g_score[current] + 1
                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + h(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))
    return None


def point_robot(start: BotState, goal: BotState, geom: BaseGeometry, obstacles: ObstacleEnvironment):



    pass
    