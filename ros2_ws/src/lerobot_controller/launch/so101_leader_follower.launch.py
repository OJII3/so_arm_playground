import os
import json
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Arguments for Leader
    leader_usb_port_arg = DeclareLaunchArgument("leader_usb_port", default_value="/dev/ttyACM1")
    leader_calib_json_arg = DeclareLaunchArgument(
        "leader_calib_json", default_value=os.path.join(
                    get_package_share_directory("lerobot_controller"),
                    "config",
                    "leader_calib.json",
                ),
    )

    # Arguments for Follower
    follower_usb_port_arg = DeclareLaunchArgument("follower_usb_port", default_value="/dev/ttyACM0")
    follower_calib_json_arg = DeclareLaunchArgument(
        "follower_calib_json", default_value=os.path.join(
                    get_package_share_directory("lerobot_controller"),
                    "config",
                    "follower_calib.json",
                ),
    )

    def setup_nodes(context, *args, **kwargs):
        leader_usb_port = LaunchConfiguration("leader_usb_port").perform(context)
        leader_calib_path = LaunchConfiguration("leader_calib_json").perform(context)
        follower_usb_port = LaunchConfiguration("follower_usb_port").perform(context)
        follower_calib_path = LaunchConfiguration("follower_calib_json").perform(context)

        # Helper function to load calibration
        def load_calibration(calib_path):
            offsets = {"1": 0, "2": 0, "3": 0, "4": 0, "5": 0, "6": 0}
            ranges_min = {"1": 0, "2": 0, "3": 0, "4": 0, "5": 0, "6": 0}
            ranges_max = {"1": 4095, "2": 4095, "3": 4095, "4": 4095, "5": 4095, "6": 4095}
            try:
                p = Path(calib_path)
                if p.exists():
                    with p.open("r") as f:
                        data = json.load(f)

                    if isinstance(data, dict) and all(
                        isinstance(v, dict) and ("id" in v and "homing_offset" in v) for v in data.values()
                    ):
                        for name, cfg in data.items():
                            try:
                                jid = str(int(cfg.get("id")))
                                hoff = int(round(float(cfg.get("homing_offset", 0)))) % 4096
                                if jid in offsets:
                                    offsets[jid] = hoff
                                rmin = cfg.get("range_min")
                                rmax = cfg.get("range_max")
                                if (jid in ranges_min) and (rmin is not None):
                                    ranges_min[jid] = int(round(float(rmin)))
                                if (jid in ranges_max) and (rmax is not None):
                                    ranges_max[jid] = int(round(float(rmax)))
                            except Exception:
                                pass
            except Exception:
                pass
            return offsets, ranges_min, ranges_max

        leader_offsets, leader_ranges_min, leader_ranges_max = load_calibration(leader_calib_path)
        follower_offsets, follower_ranges_min, follower_ranges_max = load_calibration(follower_calib_path)

        # Leader URDF
        leader_urdf = os.path.join(
            get_package_share_directory("lerobot_description"),
            "urdf",
            "so101_hw_leader.urdf.xacro",
        )

        leader_robot_description = ParameterValue(
            Command(
                [
                    "xacro ",
                    leader_urdf,
                    " usb_port:=", leader_usb_port,
                    " auto_zero_on_activate:=false",
                    " apply_home_on_activate:=false",
                    " home_j1_rad:=0.0",
                    " home_j2_rad:=0.0",
                    " home_j3_rad:=0.0",
                    " home_j4_rad:=0.0",
                    " home_j5_rad:=0.0",
                    " home_j6_rad:=0.0",
                    " offset_j1:=", str(leader_offsets["1"]),
                    " offset_j2:=", str(leader_offsets["2"]),
                    " offset_j3:=", str(leader_offsets["3"]),
                    " offset_j4:=", str(leader_offsets["4"]),
                    " offset_j5:=", str(leader_offsets["5"]),
                    " offset_j6:=", str(leader_offsets["6"]),
                    " range_min_j1:=", str(leader_ranges_min["1"]),
                    " range_min_j2:=", str(leader_ranges_min["2"]),
                    " range_min_j3:=", str(leader_ranges_min["3"]),
                    " range_min_j4:=", str(leader_ranges_min["4"]),
                    " range_min_j5:=", str(leader_ranges_min["5"]),
                    " range_min_j6:=", str(leader_ranges_min["6"]),
                    " range_max_j1:=", str(leader_ranges_max["1"]),
                    " range_max_j2:=", str(leader_ranges_max["2"]),
                    " range_max_j3:=", str(leader_ranges_max["3"]),
                    " range_max_j4:=", str(leader_ranges_max["4"]),
                    " range_max_j5:=", str(leader_ranges_max["5"]),
                    " range_max_j6:=", str(leader_ranges_max["6"]),
                ]
            ),
            value_type=str,
        )

        # Follower URDF
        follower_urdf = os.path.join(
            get_package_share_directory("lerobot_description"),
            "urdf",
            "so101_hw.urdf.xacro",
        )

        follower_robot_description = ParameterValue(
            Command(
                [
                    "xacro ",
                    follower_urdf,
                    " usb_port:=", follower_usb_port,
                    " auto_zero_on_activate:=false",
                    " apply_home_on_activate:=false",
                    " home_j1_rad:=0.0",
                    " home_j2_rad:=0.0",
                    " home_j3_rad:=0.0",
                    " home_j4_rad:=0.0",
                    " home_j5_rad:=0.0",
                    " home_j6_rad:=0.0",
                    " offset_j1:=", str(follower_offsets["1"]),
                    " offset_j2:=", str(follower_offsets["2"]),
                    " offset_j3:=", str(follower_offsets["3"]),
                    " offset_j4:=", str(follower_offsets["4"]),
                    " offset_j5:=", str(follower_offsets["5"]),
                    " offset_j6:=", str(follower_offsets["6"]),
                    " range_min_j1:=", str(follower_ranges_min["1"]),
                    " range_min_j2:=", str(follower_ranges_min["2"]),
                    " range_min_j3:=", str(follower_ranges_min["3"]),
                    " range_min_j4:=", str(follower_ranges_min["4"]),
                    " range_min_j5:=", str(follower_ranges_min["5"]),
                    " range_min_j6:=", str(follower_ranges_min["6"]),
                    " range_max_j1:=", str(follower_ranges_max["1"]),
                    " range_max_j2:=", str(follower_ranges_max["2"]),
                    " range_max_j3:=", str(follower_ranges_max["3"]),
                    " range_max_j4:=", str(follower_ranges_max["4"]),
                    " range_max_j5:=", str(follower_ranges_max["5"]),
                    " range_max_j6:=", str(follower_ranges_max["6"]),
                ]
            ),
            value_type=str,
        )

        # Leader nodes
        leader_robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="leader",
            parameters=[{"robot_description": leader_robot_description}],
        )

        leader_controller_manager = Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace="leader",
            parameters=[
                {"robot_description": leader_robot_description, "use_sim_time": False},
                os.path.join(
                    get_package_share_directory("lerobot_controller"),
                    "config",
                    "so101_leader_controllers.yaml",
                ),
            ],
        )

        leader_joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/leader/controller_manager",
            ],
        )

        # Follower nodes
        follower_robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="follower",
            parameters=[{"robot_description": follower_robot_description}],
        )

        follower_controller_manager = Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace="follower",
            parameters=[
                {"robot_description": follower_robot_description, "use_sim_time": False},
                os.path.join(
                    get_package_share_directory("lerobot_controller"),
                    "config",
                    "so101_follower_controllers.yaml",
                ),
            ],
        )

        follower_joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/follower/controller_manager",
            ],
        )

        follower_arm_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller", "--controller-manager", "/follower/controller_manager"],
        )

        follower_gripper_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gripper_controller", "--controller-manager", "/follower/controller_manager"],
        )

        # Joint state to trajectory converter (leader -> follower)
        joint_state_to_trajectory = Node(
            package="lerobot_controller",
            executable="joint_state_to_trajectory",
            name="joint_state_to_trajectory",
            parameters=[{
                "input_topic": "/leader/joint_states",
                "arm_output_topic": "/follower/arm_controller/joint_trajectory",
                "gripper_output_topic": "/follower/gripper_controller/joint_trajectory",
                "arm_joints": ["1", "2", "3", "4", "5"],
                "gripper_joints": ["6"],
                "time_from_start_sec": 0.1,
            }],
        )

        return [
            leader_robot_state_publisher,
            leader_controller_manager,
            leader_joint_state_broadcaster_spawner,
            follower_robot_state_publisher,
            follower_controller_manager,
            follower_joint_state_broadcaster_spawner,
            follower_arm_controller_spawner,
            follower_gripper_controller_spawner,
            joint_state_to_trajectory,
        ]

    return LaunchDescription(
        [
            leader_usb_port_arg,
            leader_calib_json_arg,
            follower_usb_port_arg,
            follower_calib_json_arg,
            OpaqueFunction(function=setup_nodes),
        ]
    )
