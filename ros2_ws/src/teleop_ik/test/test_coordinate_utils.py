import math

import pytest

from teleop_ik.coordinate_utils import unity_position_to_ros, unity_quaternion_to_ros


def test_unity_position_to_ros_axis_mapping():
    ros = unity_position_to_ros(1.0, 2.0, 3.0, scale=1.0)
    assert ros[0] == pytest.approx(3.0)
    assert ros[1] == pytest.approx(-1.0)
    assert ros[2] == pytest.approx(2.0)


def test_unity_position_to_ros_applies_scale():
    ros = unity_position_to_ros(1.0, 2.0, 3.0, scale=2.0)
    assert ros[0] == pytest.approx(6.0)
    assert ros[1] == pytest.approx(-2.0)
    assert ros[2] == pytest.approx(4.0)


def test_unity_quaternion_to_ros_flips_w_for_handedness():
    ros_q = unity_quaternion_to_ros(0.0, 0.0, 0.0, 1.0)
    assert ros_q[3] == pytest.approx(-1.0)
