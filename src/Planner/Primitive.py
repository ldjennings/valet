from Bots import S, Bot
from utils import angle_distance, angs, center_distance, heading, pos, trajectory_length, wrap_angle
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


def _is_reverse(traj: list) -> bool:
    """True if the primitive moves opposite to the starting heading (reverse gear)."""
    heading = angs(traj[0])

    if not heading:
        return False  # PointBot has no heading

    x1, y1 = pos(traj[0])
    x2, y2 = pos(traj[1])

    dx, dy = x2 - x1, y2 - y1
    if math.hypot(dx, dy) < 1e-6:
        return False  # pure rotation, not reverse
    return (dx * math.cos(heading[0]) + dy * math.sin(heading[0])) < 0


def propagated_primitives(bot: Bot, state: S, config: HybridConfig, steering_granularity: int = 3) -> list[Primitive]:
    """
    Call bot.propagate() and wrap each trajectory into a Primitive with arc-length cost.
    Each bot internally computes per-control n_steps so every primitive displaces
    at least `config.spacing` in XY and turns at least `config.angular_spacing` in heading.
    """
    primitives = []
    for traj in bot.propagate(state, config.spacing, config.angular_spacing, steering_granularity):

        cost = trajectory_length(traj, ROTATION_COST_WEIGHT)
        if  _is_reverse(traj):
            cost += config.reverse_cost

        primitives.append(Primitive(traj, cost))
    return primitives
