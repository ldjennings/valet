"""
Robot kinematics, state representations, and collision geometry for the valet simulator.

Each bot type pairs a state dataclass (BotState) with a kinematics class (Bots)
that implements footprint generation, goal checking, and keyboard input handling.
"""

from Bots.Bots import Bot, PointBot, DiffBot, CarBot, TrailerBot, DT
from Bots.BotState import PointState, DiffState, CarState, TrailerState, S, center_distance, angle_distance_rad
