from dataclasses import dataclass
import math


@dataclass
class LatticeConfig:
    spacing: float
    terminal_radius: float
    angular_spacing: float = math.pi / 4
    angular_terminal: float = math.pi / 8
    trailer_angular_terminal: float = math.pi / 6  