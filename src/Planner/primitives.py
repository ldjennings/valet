from Bots import S, PointState, Bot, PointBot
from typing import Generic, Iterator
from dataclasses import dataclass
import itertools
from Planner.AstarConfig import GridConfig
import math


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

def _arc_length(traj: list) -> float:
    """
    Arc length of a trajectory plus a weighted heading-change penalty.
    The rotation term ensures pure in-place rotations have nonzero cost,
    preventing the planner from spinning for free.
    Works for all state types via __iter__.
    """
    total = 0.0
    for s1, s2 in zip(traj, traj[1:]):
        x1, y1, *rest1 = s1
        x2, y2, *rest2 = s2
        total += math.hypot(x2 - x1, y2 - y1)
        if rest1 and rest2:
            total += abs((rest1[0] - rest2[0] + math.pi) % (2 * math.pi) - math.pi) * ROTATION_COST_WEIGHT
    return total


def _is_reverse(traj: list) -> bool:
    """True if the primitive moves opposite to the starting heading (reverse gear)."""
    x1, y1, *rest = traj[0]
    x2, y2, *_    = traj[1]
    if not rest:
        return False  # PointBot has no heading
    dx, dy = x2 - x1, y2 - y1
    if math.hypot(dx, dy) < 1e-6:
        return False  # pure rotation, not reverse
    return (dx * math.cos(rest[0]) + dy * math.sin(rest[0])) < 0


def propagated_primitives(bot: Bot, state: S, config: GridConfig, steering_granularity: int = 3) -> list[Primitive]:
    """
    Call bot.propagate() and wrap each trajectory into a Primitive with arc-length cost.
    Each bot internally computes per-control n_steps so every primitive displaces
    at least `config.spacing` in XY.
    """
    primitives = []
    for traj in bot.propagate(state, config.spacing, steering_granularity):
        cost = _arc_length(traj)
        if config.reverse_cost > 0 and _is_reverse(traj):
            cost += config.reverse_cost
        primitives.append(Primitive(traj, cost))
    return primitives



class PrimitiveTable(Generic[S]):

    def __init__(self, bot: Bot[S], cfg: GridConfig):
        """
        For each discretized heading, attempt to connect to all neighbor offsets
        at all target headings via bot.trajectory().
        Primitives that return None (kinematically infeasible) are silently skipped.
        """
        # headings = [
        #     i * cfg.angular_spacing
        #     for i in range(round(2 * math.pi / cfg.angular_spacing))
        # ]

        # # neighbor offsets — 1-ring by default, expand to 2-ring if paths are too jagged
        offsets = [
            (dx * cfg.spacing, dy * cfg.spacing)
            for dx, dy in itertools.product(range(-1, 2), repeat=2)
            if (dx, dy) != (0, 0)
        ]

        start = bot.make_state(0,0)

        # table: dict[float, list[Primitive[S]]] = {}

        # for h_from in headings:
        #     from_state = bot.make_state(0.0, 0.0, h_from)
        # primitives: dict[S, Primitive[S]] = {}
        primitives: list[Primitive[S]] = []

        for (dx, dy) in offsets:
            to_state   = bot.make_state(dx, dy)
            trajectory = bot.generate_trajectory(start, to_state)


            if trajectory is None:
                continue

            assert len(trajectory) > 1, f"Number of states in generated trajectory should be greater than one, so that a cost may be calculated"

            cost = sum(
                math.hypot(
                    trajectory[i+1].x - trajectory[i].x,  # type: ignore
                    trajectory[i+1].y - trajectory[i].y,  # type: ignore
                )
                for i in range(len(trajectory) - 1)
            )

            primitives.append(Primitive(trajectory=trajectory, cost=cost))


        self.table = primitives

    def get(self, state: S) -> Iterator[Primitive[S]]:
        """
        Yields primitives valid from the given state, translated to world position.
        Lazy — caller can short-circuit without paying translation cost for unused primitives.
        """
        x0, y0, *_ = state
        # heading     = self._heading_of(state)
        # snapped     = self._snap_heading(heading)
        # stored      = self._table.get(snapped, [])

        for prim in self.table:
            yield Primitive(
                trajectory = [self._translate(s, x0, y0) for s in prim.trajectory],
                cost       = prim.cost,
            )
    @staticmethod
    def _translate(s: S, dx: float, dy: float) -> S:
        """
        Offset a state's position by (dx, dy), preserving all other dimensions.
        Works for all state types because position is always the first two fields
        and all states are positional dataclasses.
        """
        x, y, *rest = s
        return type(s)(x + dx, y + dy, *rest)
