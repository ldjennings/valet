"""
BotBundle groups a bot instance with its start and goal states.

Provides a single container that is passed through the planner and simulator,
keeping the bot and its states co-located and type-consistent.
"""

from Bots.Bots import Bot, PointBot, DiffBot, CarBot, TrailerBot
from Bots.BotState import S, PointState, DiffState, CarState, TrailerState
from typing import Generic
from dataclasses import dataclass
import simulator.config as cfg


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
                PointBot(
                    goal_radius_tol     =  cfg.GOAL_RADIUS_TOLERANCE
                ), PointState(start[0], start[1]), PointState(goal[0], goal[1])
            )

        case "diff":
            return BotBundle(
                DiffBot(
                    length              = cfg.ROBOT_LENGTH_METERS,
                    width               = cfg.ROBOT_WIDTH_METERS,
                    goal_radius_tol     = cfg.GOAL_RADIUS_TOLERANCE,
                    goal_heading_tol    = cfg.GOAL_HEADING_TOLERANCE
                ),
                DiffState(start[0], start[1], 0),
                DiffState(goal[0], goal[1], 0),
            )

        case "car":
            return BotBundle(
                CarBot(
                    wheelbase           = cfg.CAR_WHEELBASE_METERS,
                    length              = cfg.CAR_LENGTH_METERS,
                    width               = cfg.CAR_WIDTH_METERS,
                    goal_radius_tol     = cfg.GOAL_RADIUS_TOLERANCE,
                    goal_heading_tol    = cfg.GOAL_HEADING_TOLERANCE
                ), CarState(start[0], start[1], 0), CarState(goal[0], goal[1], 0)
            )

        case "trailer":
            return BotBundle(
                TrailerBot(
                    wheelbase           = cfg.TRUCK_WHEELBASE_METERS,
                    length              = cfg.TRUCK_LENGTH_METERS,
                    width               = cfg.TRUCK_WIDTH_METERS,
                    hitch_distance      = cfg.TRUCK_HITCH_TO_TRAILER_AXLE,
                    trailer_length      = cfg.TRAILER_LENGTH_METERS,
                    trailer_width       = cfg.TRAILER_WIDTH_METERS,
                    goal_radius_tol     = cfg.GOAL_RADIUS_TOLERANCE,
                    goal_heading_tol    = cfg.GOAL_HEADING_TOLERANCE,
                    trailer_heading_tol = cfg.TRAILER_HEADING_TOLERANCE
                ),
                TrailerState(start[0], start[1], 0, 0),
                TrailerState(goal[0], goal[1], 0, 0),
            )
        case _:
            raise ValueError(geom)
