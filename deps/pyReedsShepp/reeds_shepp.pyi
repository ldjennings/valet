# deps/pyReedsShepp/reeds_shepp.pyi

LEFT: int
STRAIGHT: int
RIGHT: int

class PyReedsSheppPath:
    """Reeds-Shepp path between two states for a vehicle with a minimum turning radius."""

    def __init__(
        self,
        q0: tuple[float, float, float],
        q1: tuple[float, float, float],
        turning_radius: float
    ) -> None: ...

    def distance(self) -> float:
        """Total length of the Reeds-Shepp curve."""
        ...

    def sample(self, step_size: float) -> list[tuple[float, float, float, float, float]]:
        """Uniformly sampled poses along the curve.

        Each element is (x, y, theta, curvature, signed_segment_length) where:
          curvature            = +1/rho for LEFT, -1/rho for RIGHT, 0 for STRAIGHT
          signed_segment_length = length of the current RS segment; negative = reverse gear
        """
        ...

    def type(self) -> tuple[tuple[int, float], ...]:
        """Segment types and lengths. Each element is (segment_type, length),
        where segment_type is LEFT, STRAIGHT, or RIGHT."""
        ...

def path_length(
    q0: tuple[float, float, float],
    q1: tuple[float, float, float],
    rho: float
) -> float:
    """Total length of the Reeds-Shepp curve from q0 to q1.

    Args:
        q0: Start state (x, y, theta) in meters and radians.
        q1: Goal state (x, y, theta) in meters and radians.
        rho: Minimum turning radius in meters.
    """
    ...

def path_sample(
    q0: tuple[float, float, float],
    q1: tuple[float, float, float],
    rho: float,
    step_size: float
) -> list[tuple[float, float, float, float, float]]:
    """Sample poses uniformly along the Reeds-Shepp curve from q0 to q1.

    Args:
        q0: Start state (x, y, theta) in meters and radians.
        q1: Goal state (x, y, theta) in meters and radians.
        rho: Minimum turning radius in meters.
        step_size: Distance between samples in meters.

    Returns:
        List of (x, y, theta, curvature, signed_segment_length) tuples where:
          curvature            = +1/rho for LEFT, -1/rho for RIGHT, 0 for STRAIGHT
          signed_segment_length = length of the current RS segment; negative = reverse gear
    """
    ...

def path_type(
    q0: tuple[float, float, float],
    q1: tuple[float, float, float],
    rho: float
) -> tuple[tuple[int, float], ...]:
    """Segment types and lengths for the Reeds-Shepp curve from q0 to q1.

    Args:
        q0: Start state (x, y, theta) in meters and radians.
        q1: Goal state (x, y, theta) in meters and radians.
        rho: Minimum turning radius in meters.

    Returns:
        Tuple of (segment_type, length) pairs, where segment_type is
        LEFT (1), STRAIGHT (2), or RIGHT (3).
    """
    ...