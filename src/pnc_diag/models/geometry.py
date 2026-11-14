"""Coordinate, angle, and orientation math utilities."""

from __future__ import annotations

import math


def normalize_angle(angle_rad: float) -> float:
    """Wrap an angle in radians into [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """Convert quaternion (x, y, z, w) to intrinsic ZYX Euler angles (roll, pitch, yaw) in rad."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm > 0.0:
        x, y, z, w = x / norm, y / norm, z / norm, w / norm

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Convert Euler angles (roll, pitch, yaw) in rad to quaternion (x, y, z, w)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w

