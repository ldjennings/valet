from Bots.BotState import CarState, PointState, PointTurnCapable, DiffState, Rotateable, TrailerState


def _static_checks() -> None:
    _: PointTurnCapable = DiffState(0.0, 0.0, 0.0)
    _: Rotateable = DiffState(0.0, 0.0, 0.0)
    _: Rotateable = CarState(0.0, 0.0, 0.0)
    _: Rotateable = TrailerState(0.0, 0.0, 0.0, 0.0)