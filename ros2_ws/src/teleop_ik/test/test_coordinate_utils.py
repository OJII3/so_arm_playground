import math

import pytest

from teleop_ik.coordinate_utils import unity_quaternion_to_pitch_roll


def _axis_angle_quaternion(axis, angle):
    half = angle / 2.0
    scale = math.sin(half)
    return (
        axis[0] * scale,
        axis[1] * scale,
        axis[2] * scale,
        math.cos(half),
    )


def _multiply_quaternions(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def test_unity_local_pitch_maps_from_x_rotation():
    quaternion = _axis_angle_quaternion((1.0, 0.0, 0.0), 0.4)

    pitch, roll = unity_quaternion_to_pitch_roll(*quaternion)

    assert pitch == pytest.approx(0.4)
    assert roll == pytest.approx(0.0)


def test_unity_local_roll_maps_from_z_rotation():
    quaternion = _axis_angle_quaternion((0.0, 0.0, 1.0), -0.3)

    pitch, roll = unity_quaternion_to_pitch_roll(*quaternion)

    assert pitch == pytest.approx(0.0)
    assert roll == pytest.approx(-0.3)


def test_unity_local_yaw_is_ignored():
    quaternion = _axis_angle_quaternion((0.0, 1.0, 0.0), 0.5)

    pitch, roll = unity_quaternion_to_pitch_roll(*quaternion)

    assert pitch == pytest.approx(0.0)
    assert roll == pytest.approx(0.0)


def test_unity_local_pitch_and_roll_are_extracted_when_yaw_is_present():
    pitch = 0.4
    roll = -0.3
    yaw = 0.5
    quaternion = _multiply_quaternions(
        _axis_angle_quaternion((0.0, 1.0, 0.0), yaw),
        _multiply_quaternions(
            _axis_angle_quaternion((1.0, 0.0, 0.0), pitch),
            _axis_angle_quaternion((0.0, 0.0, 1.0), roll),
        ),
    )

    actual_pitch, actual_roll = unity_quaternion_to_pitch_roll(*quaternion)

    assert actual_pitch == pytest.approx(pitch)
    assert actual_roll == pytest.approx(roll)
