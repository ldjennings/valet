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

from typing import Tuple, TypeAlias

NodeKey: TypeAlias = tuple[int, ...]

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

def discretize(
        state: S,
        position_resolution: float,
        heading_resolution: float | None,
        trailer_resolution: float | None
    ) -> NodeKey:

    x, y, *rest = state
    key: NodeKey = (round(x / position_resolution), round(y / position_resolution))

    if rest and heading_resolution is not None:
        key += (round(rest[0] / heading_resolution),)

    if len(rest) > 1 and trailer_resolution is not None:
        key += (round(rest[1] / trailer_resolution),)

    return key




# based off of wikipedia article, particularly the pseudocode found here: https://en.wikipedia.org/wiki/A*_search_algorithm#Pseudocode
# only really works with the point robots lmao, was not able to get it working with a Diff

def lattice_astar(
        env: ObstacleEnvironment,
        bot: Bot,
        start: S,
        goal: S,
        config: LatticeConfig,
        prims: PrimitiveTable
    ) -> list[S] | None:

    # Just doing euclidean distance, not considering heading at all. Works for every bot and is compliant
    #  with A* restrictions
    def h(s1: S, s2: S) -> float:
        x1, y1, *_ = s1
        x2, y2, *_ = s2
        return math.hypot(x2 - x1, y2 - y1)

    LOG_INTERVAL = 500  # print a status line every N expansions

    x0, y0, *_ = start
    xg, yg, *_ = goal
    print(f"[lattice_astar] searching from ({x0:.2f}, {y0:.2f}) to ({xg:.2f}, {yg:.2f})")

    start_key = discretize(start, config.spacing, config.angular_spacing, config.angular_spacing)

    start_node: SearchNode[S]           = SearchNode(f_cost=h(start, goal), g_cost=0.0, state=start)
    open_set:   list[SearchNode[S]]     = [start_node]
    g_score:    dict[NodeKey, float]    = {start_key: 0.0}
    visited:    set[NodeKey]            = set()

    heapq.heappush(open_set, SearchNode(
        f_cost = h(start, goal),
        g_cost = g_score[start_key],
        state  = start,
    ))


    while open_set:

        node = heapq.heappop(open_set)
        current = node.state

        current_key = discretize(current, config.spacing, config.angular_spacing, config.angular_spacing)
        if current_key in visited:
            continue
        visited.add(current_key)

        if len(visited) % LOG_INTERVAL == 0:
            x, y, *_ = current
            print(f"[lattice_astar] expanded {len(visited):5d} nodes | "
                f"open = {len(open_set):5d} | "
                f"g = {node.g_cost:.2f} | "
                f"h = {h(current, goal):.2f} | "
                f"pos = ({x:.2f}, {y:.2f})")

        if bot.is_terminal(current, goal, config):
            attempted_path = bot.generate_trajectory(current, goal)

            if attempted_path is not None and validate_path(env, bot, attempted_path):
                print(f"[lattice_astar] found path: {len(visited)} expansions, "
                    f"{len(reconstruct_path(node, attempted_path))} states")

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

    print(f"[lattice_astar] no path found after {len(visited)} expansions")

    return None



# def hybrid_astar(env: ObstacleEnvironment, bot: Bot, start: S, goal: S) -> list[S] | None:
    # Just doing euclidean distance, not considering heading at all. Works for every bot and is compliant
    #  with A* restrictions
    def h(s1: S, s2: S) -> float:
        x1, y1, *_ = s1
        x2, y2, *_ = s2
        return math.hypot(x2 - x1, y2 - y1)



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



