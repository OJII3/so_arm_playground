import os

import numpy as np
import pinocchio as pin
import pytest
import xacro
from ament_index_python.packages import get_package_share_directory

from teleop_ik.ik_node import TeleopIKNode


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
    node._q_current = pin.neutral(node._model)
    node.get_parameter = lambda name: _Parameter(
        {
            "ik_damping": 1e-6,
            "ik_max_iterations": 100,
            "ik_tolerance": 1e-4,
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

    assert ik_node._solve_ik(unreachable_target) is None


def test_solve_ik_converges_for_reachable_position_target(ik_node):
    pin.forwardKinematics(ik_node._model, ik_node._data, ik_node._q_current)
    pin.updateFramePlacements(ik_node._model, ik_node._data)
    initial_pose = ik_node._data.oMf[ik_node._ee_frame_id]
    target_position = initial_pose.translation + np.array([0.0, -0.01, 0.0])
    target = pin.SE3(initial_pose.rotation, target_position)

    result = ik_node._solve_ik(target)

    assert result is not None
    assert np.all(result >= ik_node._model.lowerPositionLimit)
    assert np.all(result <= ik_node._model.upperPositionLimit)
    pin.forwardKinematics(ik_node._model, ik_node._data, result)
    pin.updateFramePlacements(ik_node._model, ik_node._data)
    actual_position = ik_node._data.oMf[ik_node._ee_frame_id].translation
    assert np.linalg.norm(actual_position - target_position) < 1e-4
