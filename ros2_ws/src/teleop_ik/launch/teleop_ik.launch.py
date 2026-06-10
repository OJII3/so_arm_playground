"""Launch file for the teleop_ik node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    teleop_ik_dir = get_package_share_directory("teleop_ik")
    lerobot_description_dir = get_package_share_directory("lerobot_description")

    default_urdf = os.path.join(
        lerobot_description_dir, "urdf", "so101.urdf.xacro"
    )
    default_params = os.path.join(
        teleop_ik_dir, "config", "teleop_ik_params.yaml"
    )

    urdf_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value=default_urdf,
        description="Path to SO-101 URDF/xacro file",
    )

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to teleop_ik parameter YAML",
    )

    ik_node = Node(
        package="teleop_ik",
        executable="ik_node",
        name="teleop_ik_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"urdf_path": LaunchConfiguration("urdf_path")},
        ],
    )

    return LaunchDescription([
        urdf_arg,
        params_arg,
        ik_node,
    ])
