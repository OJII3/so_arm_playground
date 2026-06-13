import os

from ament_index_python.packages import get_package_share_directory
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy

from teleop_ik.ik_node import TeleopIKNode


@pytest.fixture
def nodes():
    urdf_path = os.path.join(
        get_package_share_directory("lerobot_description"),
        "urdf",
        "so101.urdf.xacro",
    )
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            f"urdf_path:={urdf_path}",
        ]
    )
    ik_node = TeleopIKNode()
    probe = Node("qos_probe")
    yield ik_node, probe
    probe.destroy_node()
    ik_node.destroy_node()
    rclpy.shutdown()


def _subscription_reliability(probe, topic_name):
    endpoints = probe.get_subscriptions_info_by_topic(topic_name)
    assert len(endpoints) == 1
    return endpoints[0].qos_profile.reliability


def test_pose_subscription_is_best_effort(nodes):
    _, probe = nodes

    assert (
        _subscription_reliability(probe, "/teleop/target_pose")
        == ReliabilityPolicy.BEST_EFFORT
    )


@pytest.mark.parametrize("topic_name", ["/teleop/active", "/teleop/gripper"])
def test_control_subscriptions_remain_reliable(nodes, topic_name):
    _, probe = nodes

    assert (
        _subscription_reliability(probe, topic_name)
        == ReliabilityPolicy.RELIABLE
    )
