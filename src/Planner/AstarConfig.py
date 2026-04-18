from dataclasses import dataclass
import math


@dataclass
class GridConfig:
    """Discretization grid shared by all grid-based planners."""
    spacing:         float          = 1.0
    angular_spacing: float          = math.pi / 4
    trailer_spacing: float | None   = None   # if None, falls back to angular_spacing
    reverse_cost:    float          = 0.0    # added to cost of any reverse primitive


@dataclass
class HybridConfig(GridConfig):
    """Hybrid A*-specific parameters on top of the base grid config."""
    angular_spacing:      float      = math.pi / 3 # override: coarser heading bins (6 instead of 8)
    steering_granularity: int        = 2          # number of steering angles between 0 and max on each side
    reverse_cost:         float      = 0.2        # override: penalise reversals in hybrid search
    max_iterations:       int | None = None      # cap on node expansions; None = unlimited
    fine_collision:       bool       = True        # False = only center-point + coarse checks (faster, less accurate)
