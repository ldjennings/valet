from Bots import S, Bot
from typing import Generic
from dataclasses import dataclass
from Planner.AstarConfig import HybridConfig
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


def propagated_primitives(bot: Bot, state: S, config: HybridConfig, steering_granularity: int = 3) -> list[Primitive]:
    """
    Call bot.propagate() and wrap each trajectory into a Primitive with arc-length cost.
    Each bot internally computes per-control n_steps so every primitive displaces
    at least `config.spacing` in XY and turns at least `config.angular_spacing` in heading.
    """
    primitives = []
    for traj in bot.propagate(state, config.spacing, config.angular_spacing, steering_granularity):
        cost = _arc_length(traj)
        if config.reverse_cost > 0 and _is_reverse(traj):
            cost += config.reverse_cost
        primitives.append(Primitive(traj, cost))
    return primitives
