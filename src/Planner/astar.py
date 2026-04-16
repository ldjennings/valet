# from Bots.BotState import BotState
from shapely.geometry.base import BaseGeometry
from environment.obstacle import ObstacleEnvironment
from Bots import S, Bot
from Planner.primitives import PrimitiveTable, Primitive
from .LatticeConfig import LatticeConfig
import numpy as np
import heapq
from dataclasses import dataclass, field
from typing import Generic
import math

from typing import Tuple

@dataclass(order=True)
class SearchNode(Generic[S]):
    f_cost: float                                                       # g + h, used for heap ordering
    g_cost: float               = field(compare=False)                  # cost so far
    state: S                    = field(compare=False)                  # continuous state
    parent: "SearchNode | None" = field(compare=False, default=None)
    trajectory: list[S]  | None = field(compare=False, default=None)    # trajectory FROM parent TO this node
 

def reconstruct_path(node: SearchNode[S], final_path: list[S] = []) -> list[S]:
    arcs = []
    while node.parent is not None:
        arcs.append(node.trajectory)
        node = node.parent
    arcs.reverse()
    path = []
    for arc in arcs:
        path.extend(arc if len(path) == 0 else arc[1:]) # unless at first node, all the paths will overlap at the end, so need this check
    
    # if we have a final path afterwards (usually one-shot closed-form trajectory), add that onto the end
    path.extend(final_path) 

    return path


def validate_path(obstacles: ObstacleEnvironment, bot: Bot, path: list[S]) -> bool:
    return all(
        obstacles.is_valid_state(bot.footprint(s))
        for s in path
    )


# based off of wikipedia article, particularly the pseudocode found here: https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode
# only really works with the point robots lmao, was not able to get it working with a Diff

def lattice_astar(env: ObstacleEnvironment, bot: Bot, start: S, goal: S, config: LatticeConfig, prims: PrimitiveTable) -> list[S] | None:
    # Just doing euclidean distance, not considering heading at all. Works for every bot and is compliant
    #  with A* restrictions
    def h(s1: S, s2: S) -> float:
        x1, y1, *_ = s1
        x2, y2, *_ = s2
        return math.hypot(x2 - x1, y2 - y1)

    start_node = SearchNode(f_cost=h(start, goal), g_cost=0.0, state=start)
    open_set:  list[SearchNode[S]] = [start_node]
    g_score:   dict[S, float]      = {start: 0.0}
    visited:   set[S]              = set()

    # heapq.heappush(open_set, (h(start, goal), counter, start))
    heapq.heappush(open_set, SearchNode(
        f_cost = h(start, goal),
        g_cost = g_score[start],
        state  = start,
    ))


    while open_set:
        node = heapq.heappop(open_set)
        current = node.state

        if current in visited:
            continue
        visited.add(current)

        if bot.is_terminal(current, goal, config):
            attempted_path = bot.generate_trajectory(current, goal)
            if attempted_path is not None and validate_path(env, bot, attempted_path):
                return reconstruct_path(node, attempted_path)

        # explore neighbors
        for prim in prims.get(current):
            endpoint = prim.endpoint

            if not validate_path(env, bot, prim.trajectory):
                continue

            tentative_g = node.g_cost + prim.cost

            if tentative_g < g_score.get(endpoint, float("inf")):
                g_score[endpoint]   = tentative_g
                f                   = tentative_g + h(endpoint, goal)
                heapq.heappush(open_set, SearchNode(
                    f_cost      = f, 
                    g_cost      = tentative_g,
                    state       = endpoint,
                    parent      = node,
                    trajectory  = prim.trajectory
                ))
    return None



# def hybrid_astar(env: ObstacleEnvironment, bot: Bot, start: S, goal: S) -> list[S] | None:
    # Just doing euclidean distance, not considering heading at all. Works for every bot and is compliant
    #  with A* restrictions
    def h(s1: S, s2: S) -> float:
        x1, y1, *_ = s1
        x2, y2, *_ = s2
        return math.hypot(x2 - x1, y2 - y1)
    
    def reconstruct_final_path(stop_state: S, origin_paths: dict[S, S]) -> list[S]:
        path = [goal]

        current_node = path[0]
        previous_node = stop_state
        while previous_node in came_from:
            traj = bot.generate_trajectory(previous_node, current_node)
            
            assert traj != None, "This should be guaranteed, if the primitives were correct"

            traj.reverse()
            path.extend(traj[1:])
            current_node = previous_node
            previous_node = origin_paths[previous_node]

        path.append(previous_node)
        path.reverse()
        print(len(path))

        # DEBUG CODE
        # for i in range(len(path) - 1):
        #     assert path[i] != path[i+1], f"Duplicate adjacent states at index {i}: {path[i]}"

        return path

    def path_is_valid(path: list[S]) -> bool:
        return all(
            env.is_valid_state(bot.footprint(s))
            for s in path
        )


    open_set:  list[Tuple[float, int, S]]   = []            # priority queue of possible nodes to expand next
    came_from: dict[S, S]                   = {}            # lets us reconstruct shortest path by knowing wh
    g_score:   dict[S, float]               = {start: 0.0}  #
    visited:   set[S]                       = set()         # set of visited nodes
    counter                                 = 0             # tiebreaker if two nodes have the same priority, chooses the fist added to the queue

    heapq.heappush(open_set, (h(start, goal), counter, start))
    counter += 1 # increment after push


    while open_set:
        _, _, current = heapq.heappop(open_set)

        if current in visited:
            continue
        visited.add(current)

        if bot.is_terminal(current, goal):
            # TODO: last-mile connection to exact goal via BVP/Reeds-Shepp
            # for now just reconstruct path to nearest lattice node
            # reconstruct path
            # TODO: make this use primitives instead of just the points
            attempted_path = bot.generate_trajectory(current, goal)
            if attempted_path is not None and path_is_valid(attempted_path):
                return reconstruct_final_path(current, came_from)

        # explore neighbors
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



