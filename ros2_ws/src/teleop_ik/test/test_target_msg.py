"""TargetPoseWithInput msg がビルド成果物から import できることを確認.

このテストは colcon build 完了後の環境でのみ pass する.
"""

from teleop_ik.msg import TargetPoseWithInput  # type: ignore[attr-defined]


def test_target_pose_with_input_instantiation():
    msg = TargetPoseWithInput()
    assert hasattr(msg, "header")
    assert hasattr(msg, "pose")
    assert hasattr(msg, "stick_x")
    assert hasattr(msg, "stick_y")
    assert msg.stick_x == 0.0
    assert msg.stick_y == 0.0


def test_target_pose_with_input_set_fields():
    msg = TargetPoseWithInput()
    msg.stick_x = 0.5
    msg.stick_y = -0.25
    assert msg.stick_x == 0.5
    assert msg.stick_y == -0.25
