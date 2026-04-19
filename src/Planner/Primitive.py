from Bots import S, Bot, PointState
from utils import angle_distance, center_distance, direction, heading, trajectory_length
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
    start = traj[0]
    next = traj[1]

    if isinstance(start, PointState):
        return False  # PointBot has no heading

    if center_distance(start, next) < 1e-6:
        return False  # pure rotation, not reverse

    h = heading(start)
    assert h is not None

    # check to see if the angle between the current heading and the direction of the next state is
    # greater than 90 degrees
    return angle_distance(direction(start, next), h) > math.pi / 2


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
