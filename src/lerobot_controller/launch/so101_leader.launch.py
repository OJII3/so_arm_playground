import os
import json
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Arguments
    usb_port_arg = DeclareLaunchArgument("usb_port", default_value="/dev/ttyACM1")
    calib_json_arg = DeclareLaunchArgument(
        "calib_json", default_value=os.path.join(
                    get_package_share_directory("lerobot_controller"),
                    "config",
                    "leader_calib.json",
                ),
    )
    auto_zero_arg = DeclareLaunchArgument("auto_zero_on_activate", default_value="false")
    apply_home_arg = DeclareLaunchArgument("apply_home_on_activate", default_value="false")
    home_j1_arg = DeclareLaunchArgument("home_j1_rad", default_value="0.0")
    home_j2_arg = DeclareLaunchArgument("home_j2_rad", default_value="0.0")
    home_j3_arg = DeclareLaunchArgument("home_j3_rad", default_value="0.0")
    home_j4_arg = DeclareLaunchArgument("home_j4_rad", default_value="0.0")
    home_j5_arg = DeclareLaunchArgument("home_j5_rad", default_value="0.0")
    home_j6_arg = DeclareLaunchArgument("home_j6_rad", default_value="0.0")

    def setup_nodes(context, *args, **kwargs):
        # Read args at runtime
        usb_port_val = LaunchConfiguration("usb_port").perform(context)
        calib_path = LaunchConfiguration("calib_json").perform(context)
        auto_zero_val = LaunchConfiguration("auto_zero_on_activate").perform(context)
        apply_home_val = LaunchConfiguration("apply_home_on_activate").perform(context)
        home_j1 = LaunchConfiguration("home_j1_rad").perform(context)
        home_j2 = LaunchConfiguration("home_j2_rad").perform(context)
        home_j3 = LaunchConfiguration("home_j3_rad").perform(context)
        home_j4 = LaunchConfiguration("home_j4_rad").perform(context)
        home_j5 = LaunchConfiguration("home_j5_rad").perform(context)
        home_j6 = LaunchConfiguration("home_j6_rad").perform(context)

        so101_hw_leader_urdf = os.path.join(
            get_package_share_directory("lerobot_description"),
            "urdf",
            "so101_hw_leader.urdf.xacro",
        )

        # default offsets (ticks) and ranges
        offsets = {"1": 0, "2": 0, "3": 0, "4": 0, "5": 0, "6": 0}
        ranges_min = {"1": 0, "2": 0, "3": 0, "4": 0, "5": 0, "6": 0}
        ranges_max = {"1": 4095, "2": 4095, "3": 4095, "4": 4095, "5": 4095, "6": 4095}
        try:
            p = Path(calib_path)
            if p.exists():
                with p.open("r") as f:
                    data = json.load(f)

                # 1) Support example_calib.json format: top-level joints with id and homing_offset
                #    e.g. {"shoulder_pan": {"id":1, "homing_offset":1453, ...}, ...}
                if isinstance(data, dict) and all(
                    isinstance(v, dict) and ("id" in v and "homing_offset" in v) for v in data.values()
                ):
                    # Map by servo id -> homing_offset (ticks, modulo 4096)
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
                else:
                    # 2) Generic formats: offsets map or unit-specified values
                    unit = None
                    if isinstance(data, dict):
                        unit = data.get("unit") or data.get("offsets_unit")
                    src = data.get("offsets", data) if isinstance(data, dict) else {}
                    if isinstance(src, dict):
                        for k in list(offsets.keys()):
                            if k in src:
                                val = src[k]
                                try:
                                    if unit:
                                        u = str(unit).lower()
                                        if u in ("tick", "ticks", "step", "steps"):
                                            offsets[k] = int(round(float(val))) % 4096
                                        elif u in ("rad", "radian", "radians"):
                                            offsets[k] = int(round(float(val) * 4096.0 / (2.0 * 3.141592653589793))) % 4096
                                        elif u in ("deg", "degree", "degrees"):
                                            offsets[k] = int(round(float(val) * 4096.0 / 360.0)) % 4096
                                        else:
                                            offsets[k] = int(round(float(val))) % 4096
                                    else:
                                        fv = float(val)
                                        if abs(fv) <= 2 * 3.141592653589793 and (abs(fv) < 10 and (fv != int(fv))):
                                            offsets[k] = int(round(fv * 4096.0 / (2.0 * 3.141592653589793))) % 4096
                                        elif abs(fv) <= 360.0:
                                            offsets[k] = int(round(fv * 4096.0 / 360.0)) % 4096
                                        else:
                                            offsets[k] = int(round(fv)) % 4096
                                except Exception:
                                    pass
        except Exception:
            pass

        robot_description = ParameterValue(
            Command(
                [
                    "xacro ",
                    so101_hw_leader_urdf,
                    " usb_port:=",
                    usb_port_val,
                    " auto_zero_on_activate:=",
                    auto_zero_val,
                    " apply_home_on_activate:=",
                    apply_home_val,
                    " home_j1_rad:=", home_j1,
                    " home_j2_rad:=", home_j2,
                    " home_j3_rad:=", home_j3,
                    " home_j4_rad:=", home_j4,
                    " home_j5_rad:=", home_j5,
                    " home_j6_rad:=", home_j6,
                    " offset_j1:=",
                    str(offsets["1"]),
                    " offset_j2:=",
                    str(offsets["2"]),
                    " offset_j3:=",
                    str(offsets["3"]),
                    " offset_j4:=",
                    str(offsets["4"]),
                    " offset_j5:=",
                    str(offsets["5"]),
                    " offset_j6:=",
                    str(offsets["6"]),
                    " range_min_j1:=", str(ranges_min["1"]),
                    " range_min_j2:=", str(ranges_min["2"]),
                    " range_min_j3:=", str(ranges_min["3"]),
                    " range_min_j4:=", str(ranges_min["4"]),
                    " range_min_j5:=", str(ranges_min["5"]),
                    " range_min_j6:=", str(ranges_min["6"]),
                    " range_max_j1:=", str(ranges_max["1"]),
                    " range_max_j2:=", str(ranges_max["2"]),
                    " range_max_j3:=", str(ranges_max["3"]),
                    " range_max_j4:=", str(ranges_max["4"]),
                    " range_max_j5:=", str(ranges_max["5"]),
                    " range_max_j6:=", str(ranges_max["6"]),
                ]
            ),
            value_type=str,
        )

        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="leader",
            parameters=[{"robot_description": robot_description}],
        )

        controller_manager = Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace="leader",
            parameters=[
                {"robot_description": robot_description, "use_sim_time": False},
                os.path.join(
                    get_package_share_directory("lerobot_controller"),
                    "config",
                    "so101_leader_controllers.yaml",
                ),
            ],
        )

        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "/leader/controller_manager",
            ],
        )

        return [
            controller_manager,
            TimerAction(period=2.0, actions=[robot_state_publisher_node]),
            joint_state_broadcaster_spawner,
        ]

    return LaunchDescription(
        [
            usb_port_arg,
            calib_json_arg,
            auto_zero_arg,
            apply_home_arg,
            home_j1_arg, home_j2_arg, home_j3_arg, home_j4_arg, home_j5_arg, home_j6_arg,
            OpaqueFunction(function=setup_nodes),
        ]
    )
