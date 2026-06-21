import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import numpy as np
import pinocchio as pin
import pytest

from teleop_ik.ik_node import TeleopIKNode
import xacro


class _ParameterValue:
    def __init__(self, value):
        self.double_value = float(value)
        self.integer_value = int(value)


class _Parameter:
    def __init__(self, value):
        self._value = _ParameterValue(value)

    def get_parameter_value(self):
        return self._value


class _Logger:
    def warn(self, _message):
        pass


@pytest.fixture
def ik_node():
    urdf_path = os.path.join(
        get_package_share_directory("lerobot_description"),
        "urdf",
        "so101.urdf.xacro",
    )
    urdf_xml = xacro.process_file(urdf_path).toxml()

    node = TeleopIKNode.__new__(TeleopIKNode)
    node._model = pin.buildModelFromXML(urdf_xml)
    node._data = node._model.createData()
    node._ee_frame_id = node._model.getFrameId("gripper")
    node._arm_joint_ids = [
        node._model.getJointId(name)
        for name in ["1", "2", "3", "4", "5"]
    ]
    node._position_joint_ids = node._arm_joint_ids[:3]
    node._wrist_joint_ids = node._arm_joint_ids[3:]
    node._q_current = pin.neutral(node._model)
    node.get_parameter = lambda name: _Parameter(
        {
            "ik_damping": 1e-6,
            "ik_max_iterations": 100,
            "ik_tolerance": 1e-4,
            "position_scale": 1.0,
        }[name]
    )
    node.get_logger = lambda: _Logger()
    return node


def test_clamp_joints_uses_urdf_position_limits(ik_node):
    below = ik_node._model.lowerPositionLimit - 1.0
    above = ik_node._model.upperPositionLimit + 1.0

    assert np.array_equal(
        ik_node._clamp_joints(below),
        ik_node._model.lowerPositionLimit,
    )
    assert np.array_equal(
        ik_node._clamp_joints(above),
        ik_node._model.upperPositionLimit,
    )


def test_solve_ik_returns_none_when_target_does_not_converge(ik_node):
    unreachable_target = pin.SE3(np.eye(3), np.array([10.0, 10.0, 10.0]))

    assert ik_node._solve_ik(unreachable_target.translation, ik_node._q_current) is None


def test_solve_ik_converges_for_reachable_position_target(ik_node):
    pin.forwardKinematics(ik_node._model, ik_node._data, ik_node._q_current)
    pin.updateFramePlacements(ik_node._model, ik_node._data)
    initial_pose = ik_node._data.oMf[ik_node._ee_frame_id]
    target_position = initial_pose.translation + np.array([0.0, -0.01, 0.0])
    result = ik_node._solve_ik(target_position, ik_node._q_current)

    assert result is not None
    assert np.all(result >= ik_node._model.lowerPositionLimit)
    assert np.all(result <= ik_node._model.upperPositionLimit)
    pin.forwardKinematics(ik_node._model, ik_node._data, result)
    pin.updateFramePlacements(ik_node._model, ik_node._data)
    actual_position = ik_node._data.oMf[ik_node._ee_frame_id].translation
    assert np.linalg.norm(actual_position - target_position) < 1e-4


def test_solve_ik_keeps_wrist_joint_targets_fixed(ik_node):
    seed = ik_node._q_current.copy()
    seed[ik_node._model.joints[ik_node._model.getJointId("4")].idx_q] = 0.2
    seed[ik_node._model.joints[ik_node._model.getJointId("5")].idx_q] = -0.3
    pin.forwardKinematics(ik_node._model, ik_node._data, seed)
    pin.updateFramePlacements(ik_node._model, ik_node._data)
    target_position = (
        ik_node._data.oMf[ik_node._ee_frame_id].translation
        + np.array([0.0, -0.005, 0.0])
    )

    result = ik_node._solve_ik(target_position, seed)

    assert result is not None
    assert result[3] == pytest.approx(0.2)
    assert result[4] == pytest.approx(-0.3)


def test_target_pose_uses_previous_successful_solution_as_next_seed(ik_node):
    ik_node._active = True
    ik_node._unity_conversion = False
    ik_node._arm_init_pos = np.zeros(3)
    ik_node._unity_anchor_pos = np.zeros(3)
    ik_node._q_solution = ik_node._q_current.copy()
    ik_node._wrist_init_pos = np.array([0.1, -0.2])
    seeds = []

    def solve(_target_position, seed):
        seeds.append(seed.copy())
        result = seed.copy()
        result[0] += 0.05
        return result

    ik_node._solve_ik = solve
    ik_node._publish_arm_trajectory = lambda _q: None

    first = PoseStamped()
    first.pose.orientation.x = math.sin(0.4 / 2.0)
    first.pose.orientation.w = math.cos(0.4 / 2.0)
    ik_node._on_target_pose(first)

    second = PoseStamped()
    second.pose.orientation.z = math.sin(-0.3 / 2.0)
    second.pose.orientation.w = math.cos(-0.3 / 2.0)
    ik_node._on_target_pose(second)

    assert seeds[0][3] == pytest.approx(0.5)
    assert seeds[0][4] == pytest.approx(-0.2)
    assert seeds[1][0] == pytest.approx(0.05)
    assert seeds[1][3] == pytest.approx(0.1)
    assert seeds[1][4] == pytest.approx(-0.5)


import time

from teleop_ik.msg import TargetPoseWithInput  # type: ignore[attr-defined]


def _make_input(stick_x: float = 0.0, stick_y: float = 0.0, dt_sec: float = 0.0):
    """Build a TargetPoseWithInput with the given stick.

    dt_sec==0 → leave header.stamp at zero so fallback is used.
    """
    msg = TargetPoseWithInput()
    msg.pose.orientation.w = 1.0
    msg.stick_x = stick_x
    msg.stick_y = stick_y
    if dt_sec > 0.0:
        msg.header.stamp.sec = int(time.time())
        msg.header.stamp.nanosec = int((dt_sec - int(dt_sec)) * 1e9)
    return msg


def test_wrist_integrates_stick_per_message(ik_node):
    ik_node._active = True
    ik_node._unity_conversion = False
    ik_node._arm_init_pos = np.zeros(3)
    ik_node._unity_anchor_pos = np.zeros(3)
    ik_node._q_solution = ik_node._q_current.copy()
    ik_node._wrist_init_pos = np.array([0.0, 0.0])
    ik_node._integrated_stick = (0.0, 0.0)
    ik_node._last_msg_stamp = None
    # set parameters used by the new path
    ik_node.get_parameter = lambda name: _Parameter(
        {
            "ik_damping": 1e-6,
            "ik_max_iterations": 100,
            "ik_tolerance": 1e-4,
            "position_scale": 1.0,
            "stick_velocity_scale": 1.0,
            "stick_deadzone": 0.0,
            "stick_max_delta_per_msg": 10.0,
            "stick_fallback_dt": 0.1,
        }[name]
    )

    # First message: integrate (1.0, 0.5) at fallback_dt=0.1, scale=1.0
    ik_node._on_target_with_input(_make_input(stick_x=1.0, stick_y=0.5))
    assert ik_node._integrated_stick[0] == pytest.approx(0.1)
    assert ik_node._integrated_stick[1] == pytest.approx(0.05)


def test_wrist_resets_on_session_start(ik_node):
    ik_node._active = True
    ik_node._unity_conversion = False
    ik_node._arm_init_pos = np.zeros(3)
    ik_node._unity_anchor_pos = np.zeros(3)
    ik_node._q_solution = ik_node._q_current.copy()
    ik_node._integrated_stick = (1.23, -0.45)
    ik_node._last_msg_stamp = None
    ik_node.get_parameter = lambda name: _Parameter(
        {
            "ik_damping": 1e-6,
            "ik_max_iterations": 100,
            "ik_tolerance": 1e-4,
            "position_scale": 1.0,
            "stick_velocity_scale": 1.0,
            "stick_deadzone": 0.0,
            "stick_max_delta_per_msg": 10.0,
            "stick_fallback_dt": 0.1,
        }[name]
    )

    from std_msgs.msg import Bool
    b = Bool()
    b.data = True
    ik_node._on_active(b)
    assert ik_node._integrated_stick == (0.0, 0.0)
    assert ik_node._last_msg_stamp is None


def test_stick_deadzone_zeros_small_inputs(ik_node):
    ik_node._active = True
    ik_node._unity_conversion = False
    ik_node._arm_init_pos = np.zeros(3)
    ik_node._unity_anchor_pos = np.zeros(3)
    ik_node._q_solution = ik_node._q_current.copy()
    ik_node._wrist_init_pos = np.array([0.0, 0.0])
    ik_node._integrated_stick = (0.0, 0.0)
    ik_node._last_msg_stamp = None
    ik_node.get_parameter = lambda name: _Parameter(
        {
            "ik_damping": 1e-6,
            "ik_max_iterations": 100,
            "ik_tolerance": 1e-4,
            "position_scale": 1.0,
            "stick_velocity_scale": 1.0,
            "stick_deadzone": 0.1,
            "stick_max_delta_per_msg": 10.0,
            "stick_fallback_dt": 0.1,
        }[name]
    )

    # 0.05 < deadzone (0.1) → no integration
    ik_node._on_target_with_input(_make_input(stick_x=0.05, stick_y=-0.05))
    assert ik_node._integrated_stick == (0.0, 0.0)


def test_stick_huge_dt_clamps_to_fallback(ik_node):
    ik_node._active = True
    ik_node._unity_conversion = False
    ik_node._arm_init_pos = np.zeros(3)
    ik_node._unity_anchor_pos = np.zeros(3)
    ik_node._q_solution = ik_node._q_current.copy()
    ik_node._wrist_init_pos = np.array([0.0, 0.0])
    ik_node._integrated_stick = (0.0, 0.0)
    ik_node._last_msg_stamp = None
    ik_node.get_parameter = lambda name: _Parameter(
        {
            "ik_damping": 1e-6,
            "ik_max_iterations": 100,
            "ik_tolerance": 1e-4,
            "position_scale": 1.0,
            "stick_velocity_scale": 1.0,
            "stick_deadzone": 0.0,
            "stick_max_delta_per_msg": 10.0,
            "stick_fallback_dt": 0.1,
        }[name]
    )

    # First message: dt fallback should be used since _last_msg_stamp is None
    ik_node._on_target_with_input(_make_input(stick_x=1.0, stick_y=1.0))
    # With fallback_dt=0.1 and scale=1.0, the deltas are 0.1 each
    assert ik_node._integrated_stick[0] == pytest.approx(0.1)
    assert ik_node._integrated_stick[1] == pytest.approx(0.1)


def test_stick_max_delta_per_msg_clamp(ik_node):
    ik_node._active = True
    ik_node._unity_conversion = False
    ik_node._arm_init_pos = np.zeros(3)
    ik_node._unity_anchor_pos = np.zeros(3)
    ik_node._q_solution = ik_node._q_current.copy()
    ik_node._wrist_init_pos = np.array([0.0, 0.0])
    ik_node._integrated_stick = (0.0, 0.0)
    ik_node._last_msg_stamp = None
    ik_node.get_parameter = lambda name: _Parameter(
        {
            "ik_damping": 1e-6,
            "ik_max_iterations": 100,
            "ik_tolerance": 1e-4,
            "position_scale": 1.0,
            "stick_velocity_scale": 1.0,
            "stick_deadzone": 0.0,
            "stick_max_delta_per_msg": 0.05,  # tight cap
            "stick_fallback_dt": 1.0,           # large dt would otherwise blow past cap
        }[name]
    )

    # Stick=1.0 with dt=1.0 and cap=0.05 → integration must stay within cap
    ik_node._on_target_with_input(_make_input(stick_x=1.0, stick_y=1.0))
    assert ik_node._integrated_stick[0] == pytest.approx(0.05)
    assert ik_node._integrated_stick[1] == pytest.approx(0.05)
