"""
Simulation constants for the Valet assignment.

All tunable parameters live here so implementation files stay clean.
"""

import math

# ── Grid / Environment ───────────────────────────────────────────────────
NUM_ROWS, NUM_COLS  = 12, 12
CELLS_TO_METERS     = 3                             # meters per grid cell
OBSTACLE_COVERAGE   = 0.1                          # fraction of cells filled

# ── Display ──────────────────────────────────────────────────────────────
VIRTUAL_SIZE        = (800, 800)                    # pixels
METERS_TO_PIXELS    = min(
    VIRTUAL_SIZE[0] // (NUM_COLS * CELLS_TO_METERS),
    VIRTUAL_SIZE[1] // (NUM_ROWS * CELLS_TO_METERS),
)

# ── Simulation ───────────────────────────────────────────────────────────
FPS = 30
DT  = 1 / FPS                                       # physics timestep (seconds)

# ── Differential drive robot ─────────────────────────────────────────────
ROBOT_LENGTH_METERS = 0.7                           # meters
ROBOT_WIDTH_METERS  = 0.57                          # meters

# ── Car ──────────────────────────────────────────────────────────────────
CAR_LENGTH_METERS   = 5.2                           # meters
CAR_WIDTH_METERS    = 1.8                           # meters
CAR_WHEELBASE_METERS = 2.8                          # meters

# ── Truck & Trailer ──────────────────────────────────────────────────────
TRUCK_LENGTH_METERS         = 5.4                   # meters
TRUCK_WIDTH_METERS          = 2.0                   # meters
TRUCK_WHEELBASE_METERS      = 3.4                   # meters
TRUCK_HITCH_TO_TRAILER_AXLE = 5.0                   # meters, hitch to trailer axle
TRAILER_LENGTH_METERS       = 4.5                   # meters
TRAILER_WIDTH_METERS        = 2.0                   # meters

# ── Goal tolerances ──────────────────────────────────────────────────────
GOAL_RADIUS_TOLERANCE    : float = 0.25             # meters
GOAL_HEADING_TOLERANCE   : float = math.pi / 12    # radians (~15°)
TRAILER_HEADING_TOLERANCE: float = math.pi / 48    # radians (~3.75°)

# ── Colors ───────────────────────────────────────────────────────────────
BLACK      = (0,   0,   0)
WHITE      = (255, 255, 255)
GREEN      = (0,   200, 0)
RED        = (200, 0,   0)
BLUE       = (0,   0,   200)
YELLOW     = (255, 255, 0)
GRAY       = (150, 150, 150)
LIGHT_BLUE = (100, 180, 255)
