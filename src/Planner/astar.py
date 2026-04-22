from environment.obstacle import ObstacleEnvironment
from Bots import S, Bot
from Planner.Primitive import propagated_primitives
from Planner.postprocessing import smooth_path, resample_path
from Planner.AstarConfig import HybridConfig
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
    coarse_step: int = 4, fine_checking: bool = True,
) -> bool:
    # this is the most expensive method in the whole program, so steps have been taken to optimize it.

    # Phase 1: Check sparse subset (endpoint + every Nth).
    # Uses approximate (cached) footprints for speed during search.

    # always check last state, exit early if invalid
    if not obstacles.is_valid_state(bot.footprint(path[-1], approximate=True)):
        return False

    # check every nth state in trajectory
    for i in range(0, len(path), coarse_step):
        if not obstacles.is_valid_state(bot.footprint(path[i], approximate=True)):
            return False

    if fine_checking:
        # Optional Phase 2: check remaining states.
        for i in range(len(path)):
            if i % coarse_step == 0:
                continue
            if not obstacles.is_valid_state(bot.footprint(path[i], approximate= False)):
                return False

    return True


def discretize(state: S, config: HybridConfig) -> NodeKey:
    """
    Map a continuous state to a discrete grid cell for A* visited/g_score tracking.

    XY is rounded to the nearest spacing multiple.  Headings use floor + modulo
    so that 0 and 2π map to the same bin (round would create a spurious extra bin).
    """
    x, y, *rest = state
    key: NodeKey = (round(x / config.spacing), round(y / config.spacing))

    if rest:
        n_heading_bins = round(2 * math.pi / config.angular_spacing)
        heading = rest[0] % (2 * math.pi)
        key += (int(heading / config.angular_spacing) % n_heading_bins,)

    if len(rest) > 1:
        trailer_res = config.trailer_spacing if config.trailer_spacing is not None else config.angular_spacing
        n_trailer_bins = round(2 * math.pi / trailer_res)
        trailer_heading = rest[1] % (2 * math.pi)
        key += (int(trailer_heading / trailer_res) % n_trailer_bins,)

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

    x0, y0 = start.position()
    xg, yg = goal.position()
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

        p = current.position()
        if debug:
            visited_xy.append(p)

        if config.max_iterations is not None and len(visited) >= config.max_iterations:
            print(f"[hybrid_astar] hit max_iterations ({config.max_iterations}) — stopping")
            return PlanResult(path=None, visited_xy=visited_xy)

        if len(visited) % LOG_INTERVAL == 0:
            print(f"[hybrid_astar] expanded {len(visited):5d} nodes | "
                f"open = {len(open_set):5d} | "
                f"g = {node.g_cost:.2f} | "
                f"h = {bot.heuristic(current, goal):.2f} | "
               f"pos = ({p[0]:.2f}, {p[1]:.2f})")

        if bot.is_terminal(current, goal):
            attempted_path = bot.generate_trajectory(current, goal)

            if attempted_path is not None \
                and validate_path(env, bot, attempted_path, fine_checking=config.fine_collision) \
                and bot.at_goal(attempted_path[-1], goal):

                print(f"[hybrid_astar] found path: {len(visited)} expansions, "
                    f"{len(reconstruct_path(node, attempted_path))} states")

                raw_path = smooth_path(reconstruct_path(node, attempted_path), bot, env)
                return PlanResult(
                    path=resample_path(raw_path),
                    visited_xy=visited_xy,
                )

        # explore neighbors
        for prim in propagated_primitives(bot, current, config, config.steering_granularity):
            endpoint = prim.endpoint
            endpoint_key = discretize(endpoint, config)

            if not validate_path(env, bot, prim.trajectory, fine_checking=config.fine_collision):
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
