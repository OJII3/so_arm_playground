"""Launch the follower hardware (or mock) without teleop_ik_node.

SoArmVR publishes JointTrajectory directly via ROSettaDDS.
This launch only brings up the controller chain and optionally RViz.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    teleop_ik_dir = get_package_share_directory("teleop_ik")
    lerobot_controller_dir = get_package_share_directory("lerobot_controller")

    use_mock_arg = DeclareLaunchArgument(
        "use_mock",
        default_value="false",
        description="Use mock hardware (RViz) instead of real Feetech servos",
    )

    usb_port_arg = DeclareLaunchArgument(
        "usb_port",
        default_value="/dev/ttyACM0",
        description="USB port for Feetech servos (real hardware only)",
    )

    arm_topic_arg = DeclareLaunchArgument(
        "arm_topic",
        default_value="/follower/arm_controller/joint_trajectory",
        description="Topic name for arm JointTrajectory",
    )

    gripper_topic_arg = DeclareLaunchArgument(
        "gripper_topic",
        default_value="/follower/gripper_controller/joint_trajectory",
        description="Topic name for gripper JointTrajectory",
    )

    def launch_setup(context, *args, **kwargs):
        use_mock = LaunchConfiguration("use_mock").perform(context).lower() == "true"

        if use_mock:
            bridge_rviz_launch = os.path.join(
                teleop_ik_dir, "launch", "vr_teleop_bridge_rviz.launch.py"
            )
            return [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(bridge_rviz_launch),
                )
            ]
        else:
            follower_launch = os.path.join(
                lerobot_controller_dir,
                "launch",
                "so101_follower_controller.launch.py",
            )
            usb_port = LaunchConfiguration("usb_port")
            return [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(follower_launch),
                    launch_arguments={
                        "is_sim": "False",
                        "usb_port": usb_port,
                        "auto_zero_on_activate": "false",
                        "apply_home_on_activate": "false",
                    }.items(),
                )
            ]

    return LaunchDescription([
        use_mock_arg,
        usb_port_arg,
        arm_topic_arg,
        gripper_topic_arg,
        OpaqueFunction(function=launch_setup),
    ])
