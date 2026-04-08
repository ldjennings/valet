from Bots.BotState import S, PointState
from typing import Generic, Iterator
from dataclasses import dataclass
from Bots.Bots import Bot, PointBot
import itertools
from Planner.LatticeConfig import LatticeConfig
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



class PrimitiveTable(Generic[S]):

    def __init__(self, bot: Bot[S], cfg: LatticeConfig):
        """
        For each discretized heading, attempt to connect to all neighbor offsets
        at all target headings via bot.trajectory().
        Primitives that return None (kinematically infeasible) are silently skipped.
        """
        # headings = [
        #     i * cfg.angular_spacing
        #     for i in range(round(2 * math.pi / cfg.angular_spacing))
        # ]

        # # neighbor offsets — 1-ring by default, expand to 2-ring if paths are too jagged
        offsets = [
            (dx * cfg.spacing, dy * cfg.spacing)
            for dx, dy in itertools.product(range(-1, 2), repeat=2)
            if (dx, dy) != (0, 0)
        ]
        print(len(offsets))

        start = bot.make_state(0,0)

        # table: dict[float, list[Primitive[S]]] = {}

        # for h_from in headings:
        #     from_state = bot.make_state(0.0, 0.0, h_from)
        # primitives: dict[S, Primitive[S]] = {}
        primitives: list[Primitive[S]] = []

        for (dx, dy) in offsets:
            to_state   = bot.make_state(dx, dy)
            trajectory = bot.generate_trajectory(start, to_state)


            if trajectory is None:
                continue

            assert len(trajectory) > 1, f"Number of states in generated trajectory should be greater than one, so that a cost may be calculated"

            cost = sum(
                math.hypot(
                    trajectory[i+1].x - trajectory[i].x,  # type: ignore
                    trajectory[i+1].y - trajectory[i].y,  # type: ignore
                )
                for i in range(len(trajectory) - 1)
            )

            # primitives.append(Primitive(trajectory=trajectory, cost=cost))
            primitives.append(Primitive(trajectory=trajectory, cost=cost))


        self.table = primitives

    def get(self, state: S) -> Iterator[Primitive[S]]:
        """
        Yields primitives valid from the given state, translated to world position.
        Lazy — caller can short-circuit without paying translation cost for unused primitives.
        """
        x0, y0, *_ = state
        # heading     = self._heading_of(state)
        # snapped     = self._snap_heading(heading)
        # stored      = self._table.get(snapped, [])

        for prim in self.table:
            yield Primitive(
                trajectory = [self._translate(s, x0, y0) for s in prim.trajectory],
                cost       = prim.cost,
            )
    @staticmethod
    def _translate(s: S, dx: float, dy: float) -> S:
        """
        Offset a state's position by (dx, dy), preserving all other dimensions.
        Works for all state types because position is always the first two fields
        and all states are positional dataclasses.
        """
        x, y, *rest = s
        return type(s)(x + dx, y + dy, *rest)

b = PointBot()


p = PrimitiveTable(b, LatticeConfig(.5))

collected = [p for p in p.get(PointState(-4,0))]
print(len(collected))

