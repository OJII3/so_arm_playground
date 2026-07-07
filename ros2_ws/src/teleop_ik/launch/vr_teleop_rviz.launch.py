"""Launch the follower controller chain in mock-hardware mode plus teleop_ik + RViz.

No real Feetech servos and no Gazebo required. The SO-101 URDF is
loaded with mock_components/GenericSystem so controller_manager,
arm_controller, gripper_controller, and joint_state_broadcaster can
all be spawned against the same yaml used by the real hardware launch.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    default_params = os.path.join(
        teleop_ik_dir, "config", "teleop_ik_params.yaml"
    )

    urdf_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value=default_urdf,
        description="Path to SO-101 mock URDF/xacro file",
    )
    controllers_arg = DeclareLaunchArgument(
        "controllers_file",
        default_value=default_controllers,
        description="Path to follower controllers YAML",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz,
        description="Path to RViz configuration file",
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to teleop_ik parameter YAML",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("urdf_path")]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="follower",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager_node = Node(
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

    ik_launch = os.path.join(teleop_ik_dir, "launch", "teleop_ik.launch.py")
    teleop_ik = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ik_launch),
        launch_arguments={
            "params_file": LaunchConfiguration("params_file"),
            "urdf_path": LaunchConfiguration("urdf_path"),
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
    )

    teleop_ik_rviz = TimerAction(
        period=4.0,
        actions=[teleop_ik, rviz_node],
    )

    return LaunchDescription(
        [
            urdf_arg,
            controllers_arg,
            rviz_arg,
            params_arg,
            robot_state_publisher_node,
            controller_manager_node,
            spawners,
            teleop_ik_rviz,
        ]
    )
