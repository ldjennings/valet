"""
Video recording helpers for the simulator.

Provides a Recorder protocol with two implementations: MP4Recorder writes
frames to an mp4 file via imageio, and NoOpRecorder is a silent drop-in
used when recording is disabled.
"""

from typing import Protocol
import pygame
import imageio
import numpy as np


# ── Frame extraction ──────────────────────────────────────────────────────────

def extract_image_data(surface: pygame.Surface) -> np.ndarray:
    """Convert a pygame surface to an (H, W, 3) uint8 numpy array."""
    return pygame.surfarray.array3d(surface).swapaxes(0, 1)


# ── Recorder implementations ──────────────────────────────────────────────────

class Recorder(Protocol):
    def capture(self, surface: pygame.Surface) -> None: ...
    def save(self) -> None: ...


class NoOpRecorder:
    """Drop-in recorder used when recording is disabled; all calls are no-ops."""

    def capture(self, surface: pygame.Surface) -> None:
        pass

    def save(self) -> None:
        pass


class MP4Recorder:
    """Captures pygame frames and writes them to an mp4 file on save()."""

    def __init__(self, path: str = "recording.mp4", fps: int = 30):
        self._path = path
        self._count = 0
        self._writer = imageio.get_writer(path, fps=fps)

    def capture(self, surface: pygame.Surface) -> None:
        """Append the current surface as the next frame."""
        frame = extract_image_data(surface)
        self._writer.append_data(frame)
        self._count += 1

    def save(self) -> None:
        """Flush and close the mp4 file."""
        self._writer.close()
        print(f"Saved {self._path} ({self._count} frames captured)")
