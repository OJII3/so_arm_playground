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
