"""
BotBundle groups a bot instance with its start and goal states.

Provides a single container that is passed through the planner and simulator,
keeping the bot and its states co-located and type-consistent.
"""

from bots.bots import Bot, PointBot, DiffBot, CarBot, TrailerBot
from bots.state import S, PointState, DiffState, CarState, TrailerState
from typing import Generic
from dataclasses import dataclass


@dataclass
class BotBundle(Generic[S]):
    """
    Container grouping a bot with its start and goal states.

    Generic over S so that the type checker can verify that bot, start, and goal
    all share the same state type — e.g., a BotBundle[CarState] cannot hold a DiffBot.
    """

    bot: Bot[S]
    start: S
    goal: S


def make_bot(
    geom: str, start: tuple[float, float], goal: tuple[float, float]
) -> BotBundle:
    """
    Factory that constructs a BotBundle from a geometry string and (x, y) positions.

    Start and goal headings default to 0; use BotBundle directly if non-zero
    headings are needed.

    Args:
        geom: One of "point", "diff", "car", or "trailer".
        start: (x, y) world position in meters for the start state.
        goal: (x, y) world position in meters for the goal state.

    Raises:
        ValueError: If geom is not a recognised robot type.
    """
    match geom:
        case "point":
            return BotBundle(
                PointBot(),
                PointState(start[0], start[1]), PointState(goal[0], goal[1])
            )

        case "diff":
            return BotBundle(
                DiffBot(),
                DiffState(start[0], start[1], 0),
                DiffState(goal[0], goal[1], 0),
            )

        case "car":
            return BotBundle(
                CarBot(),
                CarState(start[0], start[1], 0), CarState(goal[0], goal[1], 0)
            )

        case "trailer":
            return BotBundle(
                TrailerBot(),
                TrailerState(start[0], start[1], 0, 0),
                TrailerState(goal[0], goal[1], 0, 0),
            )
        case _:
            raise ValueError(geom)
