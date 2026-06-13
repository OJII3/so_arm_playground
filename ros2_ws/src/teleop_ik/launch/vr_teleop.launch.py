"""Launch the follower hardware and VR teleoperation IK node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    follower_dir = get_package_share_directory("lerobot_controller")
    teleop_ik_dir = get_package_share_directory("teleop_ik")

    follower_launch = os.path.join(
        follower_dir,
        "launch",
        "so101_follower_controller.launch.py",
    )
    ik_launch = os.path.join(teleop_ik_dir, "launch", "teleop_ik.launch.py")

    hardware_arguments = {
        "usb_port": "/dev/ttyACM0",
        "calib_json": os.path.join(
            follower_dir,
            "config",
            "follower_calib.json",
        ),
        "auto_zero_on_activate": "false",
        "apply_home_on_activate": "false",
        "home_j1_rad": "0.0",
        "home_j2_rad": "0.0",
        "home_j3_rad": "0.0",
        "home_j4_rad": "0.0",
        "home_j5_rad": "0.0",
        "home_j6_rad": "0.0",
    }
    params_file = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            teleop_ik_dir,
            "config",
            "teleop_ik_params.yaml",
        ),
        description="Path to teleop_ik parameter YAML",
    )

    follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(follower_launch),
        launch_arguments={
            "is_sim": "False",
            **{
                name: LaunchConfiguration(name)
                for name in hardware_arguments
            },
        }.items(),
    )
    ik = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ik_launch),
        launch_arguments={
            "params_file": LaunchConfiguration("params_file"),
        }.items(),
    )

    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(name, default_value=default)
                for name, default in hardware_arguments.items()
            ],
            params_file,
            follower,
            ik,
        ]
    )
