# from Bots.BotState import BotState
from environment.obstacle import ObstacleEnvironment
from Bots import S, Bot, TrailerBot
from Planner.primitives import PrimitiveTable
from .AstarConfig import LatticeConfig, HybridConfig
import heapq
from dataclasses import dataclass, field
from typing import Generic
from typing import TypeAlias

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
        # unless at root node, all the paths will overlap at the end, so need this check
        path.extend(arc if len(path) == 0 else arc[1:])

    # if we have a final path afterwards (usually from one-shot closed-form trajectory),
    # add that onto the end
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




# only really works with the point robots lmao, was not able to get it working with a Diff

def lattice_astar(
        env: ObstacleEnvironment,
        bot: Bot,
        start: S,
        goal: S,
        config: LatticeConfig,
        prims: PrimitiveTable
    ) -> list[S] | None:

    LOG_INTERVAL = 500  # print a status line every N expansions

    x0, y0, *_ = start
    xg, yg, *_ = goal
    print(f"[lattice_astar] searching from ({x0:.2f}, {y0:.2f}) to ({xg:.2f}, {yg:.2f})")

    start_key = discretize(start, config.spacing, config.angular_spacing, config.angular_spacing)

    open_set:   list[SearchNode[S]]     = []
    g_score:    dict[NodeKey, float]    = {start_key: 0.0}
    visited:    set[NodeKey]            = set()

    heapq.heappush(open_set, SearchNode(
        f_cost = bot.heuristic(start, goal),
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
                f"h = {bot.heuristic(current, goal):.2f} | "
                f"pos = ({x:.2f}, {y:.2f})")

        if bot.is_terminal(current, goal):
            attempted_path = bot.generate_trajectory(current, goal)

            if attempted_path is not None and validate_path(env, bot, attempted_path):
                print(f"[lattice_astar] found path: {len(visited)} expansions, "
                    f"{len(reconstruct_path(node, attempted_path))} states")

                initial_path = reconstruct_path(node, attempted_path)

                return smooth_path(initial_path, bot, env)

        # explore neighbors
        for prim in prims.get(current):
            endpoint = prim.endpoint
            endpoint_key = discretize(endpoint, config.spacing, config.angular_spacing, config.angular_spacing)

            if not validate_path(env, bot, prim.trajectory):
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

    print(f"[lattice_astar] no path found after {len(visited)} expansions")

    return None



def hybrid_astar(
        env: ObstacleEnvironment,
        bot: Bot,
        start: S,
        goal: S,
        config: HybridConfig,
    ) -> list[S] | None:

    LOG_INTERVAL = 500  # print a status line every N expansions

    x0, y0, *_ = start
    xg, yg, *_ = goal
    print(f"[hybrid_astar] searching from ({x0:.2f}, {y0:.2f}) to ({xg:.2f}, {yg:.2f})")

    start_key = discretize(start, config.xy_spacing, config.angular_spacing, config.angular_spacing)

    open_set:   list[SearchNode[S]]     = []
    g_score:    dict[NodeKey, float]    = {start_key: 0.0}
    visited:    set[NodeKey]            = set()

    heapq.heappush(open_set, SearchNode(
        f_cost = bot.heuristic(start, goal),
        g_cost = g_score[start_key],
        state  = start,
    ))


    while open_set:

        node = heapq.heappop(open_set)
        current = node.state

        current_key = discretize(current, config.xy_spacing, config.angular_spacing, config.angular_spacing)
        if current_key in visited:
            continue
        visited.add(current_key)

        if len(visited) % LOG_INTERVAL == 0:
            x, y, *_ = current
            print(f"[hybrid_astar] expanded {len(visited):5d} nodes | "
                f"open = {len(open_set):5d} | "
                f"g = {node.g_cost:.2f} | "
                f"h = {bot.heuristic(current, goal):.2f} | "
                f"pos = ({x:.2f}, {y:.2f})")

        if bot.is_terminal(current, goal):
            attempted_path = bot.generate_trajectory(current, goal)

            if attempted_path is not None and validate_path(env, bot, attempted_path):
                print(f"[hybrid_astar] found path: {len(visited)} expansions, "
                    f"{len(reconstruct_path(node, attempted_path))} states")

                return reconstruct_path(node, attempted_path)

        # explore neighbors
        for prim in prims.get(current):
            endpoint = prim.endpoint
            endpoint_key = discretize(endpoint, config.xy_spacing, config.angular_spacing, config.angular_spacing)

            if not validate_path(env, bot, prim.trajectory):
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

    return None


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
