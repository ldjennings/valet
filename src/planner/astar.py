from dataclasses import dataclass, field
import math
from typing import Generic, cast, TypeAlias
import heapq

from bots import S, Bot
from bots.state import Rotateable
from utils import Position, angle_distance, center_distance, direction
from environment.obstacle import ObstacleEnvironment
from planner.postprocessing import smooth_path, resample_path


@dataclass
class HybridConfig():
    """Hybrid A* parameter configuration."""
    spacing:                float           = .5
    angular_spacing:        float           = math.pi / 6   # heading bins (6 instead of 8)
    trailer_spacing:        float | None    = None          # if None, falls back to angular_spacing
    steering_granularity:   int             = 2             # number of steering angles between 0 and max on each side
    reverse_cost:           float           = 0.2           # override: penalise reversals in hybrid search
    max_iterations:         int | None      = 7500          # cap on node expansions; None = unlimited
    fine_collision:         bool            = True          # False = only center-point + coarse checks (faster, less accurate)


@dataclass(frozen=True)
class Primitive(Generic[S]):
    trajectory: list[S]
    cost: float

    @property
    def start(self) -> S:
        return self.trajectory[0]

    @property
    def endpoint(self) -> S:
        return self.trajectory[-1]


    def __lt__(self, other: "Primitive") -> bool:
        return self.cost < other.cost


ROTATION_COST_WEIGHT = 0.5  # cost per radian of heading change; keeps rotate-in-place nonzero


def _is_reverse(traj: list[Rotateable]) -> bool:
    """True if the primitive moves opposite to the starting heading (reverse gear)."""
    start = traj[0]
    next = traj[1]

    x1, y1, h1 = start.pose()
    x2, y2, _ = next.pose()

    p1 = Position((x1,y1))
    p2 = Position((x2, y2))

    if center_distance(p1, p2) < 1e-6:
        return False  # pure rotation, not reverse


    # check to see if the angle between the current heading and the direction of the next state is
    # greater than 90 degrees
    return angle_distance(direction(p1, p2), h1) > math.pi / 2


def propagated_primitives(bot: Bot, state: S, config: HybridConfig, steering_granularity: int = 3) -> list[Primitive]:
    """
    Call bot.propagate() and wrap each (trajectory, arc_length) pair into a Primitive.

    arc_length (translational distance) comes from the bot analytically — exact for
    constant-curvature arcs, with no per-step summation loop.  The angular cost is
    computed here in O(1) from the endpoint heading difference, and the reverse
    penalty is applied if the primitive moves against the starting heading.
    """
    primitives = []
    for traj, arc_length in bot.propagate(state, config.spacing, config.angular_spacing, steering_granularity):
        if isinstance(traj[0], Rotateable):
            posed = cast(list[Rotateable], traj)
            heading_change = angle_distance(posed[-1].heading(), posed[0].heading())
            cost = arc_length + heading_change * ROTATION_COST_WEIGHT
            if _is_reverse(posed):
                cost += config.reverse_cost
        else:
            cost = arc_length

        primitives.append(Primitive(traj, cost))
    return primitives


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


def reconstruct_path(node: SearchNode[S], final_path: list[S] | None = None) -> list[S]:
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
    if final_path is not None:
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
