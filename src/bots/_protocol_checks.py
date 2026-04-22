"""
Static protocol conformance checks.

These functions are never called at runtime. They exist so that the type
checker verifies that each state and bot class actually satisfies its protocol.
If a state or bot stops conforming, mypy/pyright will flag it here.
"""

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
