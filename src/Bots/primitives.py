from .BotState import S
from typing import Generic
from dataclasses import dataclass

@dataclass(frozen=True)
class Primitive(Generic[S]):
    trajectory: tuple[S, ...]  # tuple not list so it's hashable
    cost: float

    @property
    def start(self) -> S:
        return self.trajectory[0]

    @property
    def endpoint(self) -> S:
        return self.trajectory[-1]