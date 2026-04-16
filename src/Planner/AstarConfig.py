from dataclasses import dataclass, field
import math


@dataclass
class LatticeConfig:
    spacing: float
    angular_spacing: float = math.pi / 4
    terminal_radius: float = 25.0


@dataclass
class HybridConfig:
    xy_spacing: float
    angular_spacing: float = math.pi / 4
    terminal_radius: float = 25.0
    num_headings: int = 3
    reverse_added_cost: float = .2
