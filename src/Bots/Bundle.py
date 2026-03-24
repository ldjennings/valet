from Bots.Bots import Bot, PointBot, DiffBot, CarBot, TrailerBot
from Bots.BotState import S, PointState, DiffState, CarState, TrailerState
from typing import Generic
from dataclasses import dataclass


@dataclass
class BotBundle(Generic[S]):
    bot: Bot[S]   # a Bot whose methods all speak S
    state: S      # the current state, also S


def make_bot(geom: str, x: float, y: float) -> BotBundle:
    match geom:
        case "point":   return BotBundle(PointBot(),   PointState(x, y))
        case "diff":    return BotBundle(DiffBot(),    DiffState(x, y, 0))
        case "car":     return BotBundle(CarBot(),     CarState(x, y, 0))
        case "trailer": return BotBundle(TrailerBot(), TrailerState(x, y, 0, 0))
        case _:         raise ValueError(geom)