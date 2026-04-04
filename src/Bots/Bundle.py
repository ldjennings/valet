from Bots.Bots import Bot, PointBot, DiffBot, CarBot, TrailerBot
from Bots.BotState import S, PointState, DiffState, CarState, TrailerState
from typing import Generic
from dataclasses import dataclass


@dataclass
class BotBundle(Generic[S]):
    bot: Bot[S]  # a Bot whose methods all speak S
    start: S  # the current state, also S
    goal: S  # the goal state, S as well


def make_bot(
    geom: str, start: tuple[float, float], goal: tuple[float, float]
) -> BotBundle:
    match geom:
        case "point":
            return BotBundle(
                PointBot(), PointState(start[0], start[1]), PointState(goal[0], goal[1])
            )

        case "diff":
            return BotBundle(
                DiffBot(),
                DiffState(start[0], start[1], 0),
                DiffState(goal[0], goal[1], 0),
            )

        case "car":
            return BotBundle(
                CarBot(), CarState(start[0], start[1], 0), CarState(goal[0], goal[1], 0)
            )

        case "trailer":
            return BotBundle(
                TrailerBot(),
                TrailerState(start[0], start[1], 0, 0),
                TrailerState(goal[0], goal[1], 0, 0),
            )
        case _:
            raise ValueError(geom)
