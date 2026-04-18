from dataclasses import dataclass
import math

@dataclass
class HybridConfig():
    """Hybrid A* parameters on top of the base grid config."""
    spacing:                float           = .5
    angular_spacing:        float           = math.pi / 6   # heading bins (6 instead of 8)
    trailer_spacing:        float | None    = None          # if None, falls back to angular_spacing
    steering_granularity:   int             = 2             # number of steering angles between 0 and max on each side
    reverse_cost:           float           = 0.2           # override: penalise reversals in hybrid search
    max_iterations:         int | None      = None          # cap on node expansions; None = unlimited
    fine_collision:         bool            = True          # False = only center-point + coarse checks (faster, less accurate)
