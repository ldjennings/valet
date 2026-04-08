from typing import Protocol
import pygame
import imageio
import numpy as np

def extract_image_data(surface: pygame.Surface) -> np.ndarray:
    return pygame.surfarray.array3d(surface).swapaxes(0, 1)

class Recorder(Protocol):
    def capture(self, surface: pygame.Surface) -> None: ...
    def save(self) -> None: ...


class NoOpRecorder:
    """Used when recording is disabled: all calls are silent no-ops."""

    def capture(self, surface: pygame.Surface) -> None:
        pass

    def save(self) -> None:
        pass


class MP4Recorder:
    """Records the progression into an mp4 file"""

    def __init__(
        self, path: str = "recording.mp4", fps: int = 30
    ):
        self._path = path
        self._count = 0
        self._writer = imageio.get_writer(path, fps=fps)

    def capture(self, surface: pygame.Surface) -> None:
        frame = extract_image_data(surface)
        self._writer.append_data(frame)
        self._count += 1

    def save(self) -> None:
        self._writer.close()
        print(f"Saved {self._path} ({self._count} frames captured)")
