"""Coordinate conversion utilities: Unity (left-hand, Y-up) <-> ROS (right-hand, Z-up)."""

import math

import numpy as np


def unity_position_to_ros(
    x: float, y: float, z: float, scale: float = 1.0
) -> np.ndarray:
    """Convert Unity position (X-right, Y-up, Z-forward) to ROS (X-forward, Y-left, Z-up).

    Mapping:
        ros_x =  unity_z
        ros_y = -unity_x
        ros_z =  unity_y
    """
    return np.array([z * scale, -x * scale, y * scale])


def unity_quaternion_to_ros(
    x: float, y: float, z: float, w: float
) -> np.ndarray:
    """Convert Unity quaternion to ROS quaternion.

    Unity is left-handed Y-up, ROS is right-handed Z-up.
    Apply the same axis remapping as position to the quaternion vector part.

    Returns [qx, qy, qz, qw] in ROS convention.
    """
    # Vector part follows the same axis mapping as position
    ros_qx = z
    ros_qy = -x
    ros_qz = y
    ros_qw = -w  # handedness flip
    return np.array([ros_qx, ros_qy, ros_qz, ros_qw])


def unity_quaternion_to_pitch_roll(
    x: float, y: float, z: float, w: float
) -> tuple[float, float]:
    """Return Unity-local pitch (X) and roll (Z) using Unity's Z-X-Y order."""
    quaternion = np.array([x, y, z, w], dtype=float)
    norm = np.linalg.norm(quaternion)
    if norm == 0.0:
        return 0.0, 0.0

    quaternion /= norm
    x, y, z, w = quaternion

    matrix_10 = 2.0 * (x * y + z * w)
    matrix_11 = 1.0 - 2.0 * (x * x + z * z)
    matrix_12 = 2.0 * (y * z - x * w)

    pitch = math.asin(float(np.clip(-matrix_12, -1.0, 1.0)))
    roll = math.atan2(matrix_10, matrix_11)
    return pitch, roll
