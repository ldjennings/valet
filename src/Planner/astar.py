from environment.obstacle import ObstacleEnvironment
from Bots import S, Bot, TrailerBot
from Planner.primitives import PrimitiveTable, propagated_primitives
from .AstarConfig import GridConfig, HybridConfig
import heapq
from dataclasses import dataclass, field
from typing import Generic
from typing import TypeAlias

import math
NodeKey: TypeAlias = tuple[int, ...]

@dataclass
class PlanResult(Generic[S]):
    path:       list[S] | None
    visited_xy: list[tuple[float, float]]   # (x, y) of every expanded node, for debug viz


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
        # unless at root node, all the paths will overlap at the end, so need this check
        path.extend(arc if len(path) == 0 else arc[1:])

    # if we have a final path afterwards (usually from one-shot closed-form trajectory),
    # add that onto the end
    path.extend(final_path)

    return path


def validate_path(
    obstacles: ObstacleEnvironment, bot: Bot, path: list[S],
    coarse_step: int = 4, fine: bool = True,
) -> bool:
    # Phase 1: center-point + full footprint on sparse subset (endpoints + every Nth).
    for i in range(0, len(path), coarse_step):
        x, y, *_ = path[i]
        if not obstacles.is_point_free(x, y):
            return False
        if not obstacles.is_valid_state(bot.footprint(path[i])):
            return False

    # always check last state
    x, y, *_ = path[-1]
    if not obstacles.is_point_free(x, y):
        return False
    if not obstacles.is_valid_state(bot.footprint(path[-1])):
        return False

    if not fine:
        return True

    # Phase 2: fill in remaining states.
    for i in range(len(path)):
        if i % coarse_step == 0:
            continue
        if not obstacles.is_valid_state(bot.footprint(path[i])):
            return False

    return True


def smooth_path(
        path: list[S],
        bot: Bot,
        obstacles: ObstacleEnvironment,
        iterations: int = 100,
    ) -> list[S]:
    """
    Probabilistic path shortcutting. Repeatedly picks two random indices, attempts
    a direct connection via bot.generate_trajectory, and replaces the span if the
    shortcut is collision-free. Skipped entirely for TrailerBot since
    generate_trajectory ignores the trailer heading.
    """
    if isinstance(bot, TrailerBot):
        return path

    import random
    path = list(path)
    initial_len = len(path)
    shortcuts_applied = 0

    for i in range(iterations):
        if len(path) < 3:
            print(f"[smooth_path] path too short to shorten further, stopping at iteration {i}")
            break

        a = random.randint(0, len(path) - 2)
        b = random.randint(a + 1, len(path) - 1)

        shortcut = bot.generate_trajectory(path[a], path[b])
        if shortcut is None:
            continue

        if validate_path(obstacles, bot, shortcut):
            path = path[:a] + shortcut + path[b + 1:]
            shortcuts_applied += 1

    print(f"[smooth_path] {iterations} iterations | {shortcuts_applied} shortcuts applied | "
          f"{initial_len} -> {len(path)} states")

    return path


def discretize(state: S, config: GridConfig) -> NodeKey:
    x, y, *rest = state
    key: NodeKey = (round(x / config.spacing), round(y / config.spacing))

    if rest:
        heading = rest[0] % (2 * math.pi)   # normalize to [0, 2π)
        key += (round(heading / config.angular_spacing),)

    if len(rest) > 1:
        trailer_res = config.trailer_spacing if config.trailer_spacing is not None else config.angular_spacing
        trailer_heading = rest[1] % (2 * math.pi)   # normalize to [0, 2π)
        key += (round(trailer_heading / trailer_res),)

    return key



def hybrid_astar(
        env: ObstacleEnvironment,
        bot: Bot,
        start: S,
        goal: S,
        config: HybridConfig,
        debug: bool = False,
    ) -> PlanResult[S]:

    LOG_INTERVAL = 500  # print a status line every N expansions

    x0, y0, *_ = start
    xg, yg, *_ = goal
    print(f"[hybrid_astar] searching from ({x0:.2f}, {y0:.2f}) to ({xg:.2f}, {yg:.2f})")

    start_key = discretize(start, config)

    open_set:   list[SearchNode[S]]         = []
    g_score:    dict[NodeKey, float]        = {start_key: 0.0}
    visited:    set[NodeKey]                = set()
    visited_xy: list[tuple[float, float]]   = []

    heapq.heappush(open_set, SearchNode(
        f_cost = bot.heuristic(start, goal),
        g_cost = g_score[start_key],
        state  = start,
    ))


    while open_set:

        node = heapq.heappop(open_set)
        current = node.state

        current_key = discretize(current, config)
        if current_key in visited:
            continue
        visited.add(current_key)

        x, y, *_ = current
        if debug:
            visited_xy.append((x, y))

        if config.max_iterations is not None and len(visited) >= config.max_iterations:
            print(f"[hybrid_astar] hit max_iterations ({config.max_iterations}) — stopping")
            return PlanResult(path=None, visited_xy=visited_xy)

        if len(visited) % LOG_INTERVAL == 0:
            print(f"[hybrid_astar] expanded {len(visited):5d} nodes | "
                f"open = {len(open_set):5d} | "
                f"g = {node.g_cost:.2f} | "
                f"h = {bot.heuristic(current, goal):.2f} | "
                f"pos = ({x:.2f}, {y:.2f})")

        if bot.is_terminal(current, goal):
            attempted_path = bot.generate_trajectory(current, goal)

            if attempted_path is not None \
                and validate_path(env, bot, attempted_path, fine=config.fine_collision) \
                and bot.at_goal(attempted_path[-1], goal):

                print(f"[hybrid_astar] found path: {len(visited)} expansions, "
                    f"{len(reconstruct_path(node, attempted_path))} states")

                return PlanResult(
                    path=reconstruct_path(node, attempted_path),
                    visited_xy=visited_xy,
                )

        # explore neighbors
        for prim in propagated_primitives(bot, current, config, config.steering_granularity):
            endpoint = prim.endpoint
            endpoint_key = discretize(endpoint, config)

            if not validate_path(env, bot, prim.trajectory, fine=config.fine_collision):
                continue

            tentative_g = node.g_cost + prim.cost

            if tentative_g < g_score.get(endpoint_key, float("inf")):
                g_score[endpoint_key]   = tentative_g
                f                       = tentative_g + bot.heuristic(endpoint, goal)
                heapq.heappush(open_set, SearchNode(
                    f_cost      = f,
                    g_cost      = tentative_g,
                    state       = endpoint,
                    parent      = node,
                    trajectory  = prim.trajectory
                ))

    print(f"[hybrid_astar] no path found after {len(visited)} expansions")

    return PlanResult(path=None, visited_xy=visited_xy)
