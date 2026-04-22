"""
Simulator package: rendering, recording, and the main simulation loop.

Public API:
    BotBundle / make_bot  — group a bot with its start and goal states
    Simulator             — runs the planning + animation loop
    Renderer              — draws each frame to a pygame surface
    MP4Recorder           — captures frames to an mp4 file
    NoOpRecorder          — drop-in replacement when recording is disabled
"""

from simulator.renderer import Renderer
from simulator.recorder import MP4Recorder, NoOpRecorder
from simulator.Bundle import BotBundle, make_bot
from simulator.simulator import Simulator