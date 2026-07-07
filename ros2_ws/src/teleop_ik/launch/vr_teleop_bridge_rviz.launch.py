"""Launch mock hardware + RViz for SoArmVR direct JointTrajectory control.

Extracted from vr_teleop_rviz.launch.py, removing teleop_ik_node.
SoArmVR connects directly via ROSettaDDS.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _check_file(path, label):
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"vr_teleop_bridge_rviz: {label} not found at {path!r}."
        )


def _preflight(context: LaunchContext, *args, **kwargs):
    paths = {
        "urdf_path": LaunchConfiguration("urdf_path").perform(context),
        "controllers_file": LaunchConfiguration("controllers_file").perform(context),
        "rviz_config": LaunchConfiguration("rviz_config").perform(context),
    }
    for label, path in paths.items():
        _check_file(path, label)
    return []


def generate_launch_description():
    lerobot_description_dir = get_package_share_directory("lerobot_description")
    lerobot_controller_dir = get_package_share_directory("lerobot_controller")
    teleop_ik_dir = get_package_share_directory("teleop_ik")

    default_urdf = os.path.join(
        lerobot_description_dir, "urdf", "so101_mock.urdf.xacro"
    )
    default_controllers = os.path.join(
        lerobot_controller_dir, "config", "so101_follower_controllers.yaml"
    )
    default_rviz = os.path.join(
        lerobot_description_dir, "rviz", "vr_teleop_rviz.rviz"
    )

    urdf_arg = DeclareLaunchArgument(
        "urdf_path", default_value=default_urdf
    )
    controllers_arg = DeclareLaunchArgument(
        "controllers_file", default_value=default_controllers
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz_config", default_value=default_rviz
    )

    preflight_check = OpaqueFunction(function=_preflight)

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("urdf_path")]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="follower",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="follower",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": False},
            LaunchConfiguration("controllers_file"),
        ],
        output="screen",
    )

    def _spawner(controller_name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                controller_name,
                "--controller-manager",
                "/follower/controller_manager",
            ],
        )

    spawners = TimerAction(
        period=2.0,
        actions=[
            _spawner("joint_state_broadcaster"),
            _spawner("arm_controller"),
            _spawner("gripper_controller"),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        output="screen",
    )

    return LaunchDescription([
        urdf_arg, controllers_arg, rviz_arg,
        preflight_check,
        robot_state_publisher,
        controller_manager,
        spawners,
        rviz,
    ])
