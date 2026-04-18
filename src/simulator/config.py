import math

# screen/environment stuff
NUM_ROWS, NUM_COLS = 12, 12
CELL_SIZE = 15
NUM_ENEMIES = 100
VIRTUAL_SIZE = (800, 800)

# Conversion factors
CELLS_TO_METERS = 3
METERS_TO_PIXELS = min(
    VIRTUAL_SIZE[0] // (NUM_COLS * CELLS_TO_METERS),
    VIRTUAL_SIZE[1] // (NUM_ROWS * CELLS_TO_METERS),
)


goal_radius_tolerance: float = 0.25
# goal_radius_tolerance: float = 5
goal_heading_tolerance: float = math.pi / 12
trailer_heading_tolerance: float = math.pi / 32


# differential drive robot dimensions
ROBOT_WIDTH_METERS = 0.57
ROBOT_LENGTH_METERS = 0.7
ROBOT_WHEELBASE_METERS = ROBOT_WIDTH_METERS / 2

# Car dimensions
# CAR_WIDTH_METERS = 1.8
# CAR_LENGTH_METERS = 5.2
CAR_WIDTH_METERS = 1.5
CAR_LENGTH_METERS = 4.2
CAR_WHEELBASE_METERS = 2.8

# Truck dimensions
TRUCK_WIDTH_METERS = 2
TRUCK_LENGTH_METERS = 5.4
TRUCK_WHEELBASE_METERS = 3.4
TRUCK_HITCH_TO_TRAILER_AXLE = 5.0
TRAILER_WIDTH_METERS = 2.0
TRAILER_LENGTH_METERS = 4.5


# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 200, 0)
RED = (200, 0, 0)
BLUE = (0, 0, 200)
YELLOW = (255, 255, 0)
GRAY = (150, 150, 150)
LIGHT_BLUE = (100, 180, 255)
