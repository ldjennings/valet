from dataclasses import dataclass

@dataclass(frozen=True)
class BotState:
    x: float
    y: float
    heading: float
    trailer_heading: float

    def __iter__(self):
        yield self.x
        yield self.y
        yield self.heading
        yield self.trailer_heading

    def add_delta(self, delta, speed: float = 1):
        # delta is a tuple/list of same length as fields
        return BotState(
            x=self.x + delta[0] * speed,
            y=self.y + delta[1] * speed,
            heading=self.heading + delta[2] * speed,
            trailer_heading=self.trailer_heading + delta[3] * speed
        )