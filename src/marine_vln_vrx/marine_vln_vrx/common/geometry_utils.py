"""Geometry helpers shared by Marine-VLN nodes."""

from __future__ import annotations

import math
from typing import Tuple


def clamp(value: float, minimum: float, maximum: float) -> float:
    """Clamp value into [minimum, maximum]."""
    return max(minimum, min(maximum, value))


def wrap_to_pi(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return yaw angle in radians from quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw_rad: float) -> Tuple[float, float, float, float]:
    """Return quaternion (x, y, z, w) from yaw."""
    half = yaw_rad * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def distance_2d(x0: float, y0: float, x1: float, y1: float) -> float:
    """Euclidean distance in x-y plane."""
    return math.hypot(x1 - x0, y1 - y0)
