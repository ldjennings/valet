from dataclasses import dataclass
import math


@dataclass
class GridConfig:
    """Discretization grid shared by all grid-based planners."""
    spacing:         float          = 0.5
    angular_spacing: float          = math.pi / 4
    trailer_spacing: float | None   = None        # if None, falls back to angular_spacing


@dataclass
class HybridConfig(GridConfig):
    """Hybrid A*-specific parameters on top of the base grid config."""
    num_headings:  int   = 3
    reverse_cost:  float = 0.2
