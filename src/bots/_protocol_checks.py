from bots.state import CarState, PointTurnCapable, DiffState, Rotateable, TrailerState
from bots.bots import Bot, CarBot, DiffBot, PointBot, TrailerBot


def _state_checks() -> None:
    _: PointTurnCapable = DiffState(0.0, 0.0, 0.0)
    _: Rotateable       = DiffState(0.0, 0.0, 0.0)
    _: Rotateable       = CarState(0.0, 0.0, 0.0)
    _: Rotateable       = TrailerState(0.0, 0.0, 0.0, 0.0)


def _bot_checks() -> None:
    _: Bot  = PointBot()
    _: Bot  = DiffBot()
    _: Bot  = CarBot()
    _: Bot  = TrailerBot()
