import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription


LAUNCH_PATH = (
    Path(__file__).parents[1] / "launch" / "vr_teleop.launch.py"
)


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("vr_teleop_launch", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_vr_teleop_launch_includes_follower_and_ik_launches():
    launch_description = _load_launch_module().generate_launch_description()
    includes = [
        entity
        for entity in launch_description.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]

    assert len(includes) == 2


def test_vr_teleop_launch_exposes_hardware_arguments():
    launch_description = _load_launch_module().generate_launch_description()
    argument_names = {
        entity.name
        for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }

    assert {
        "usb_port",
        "calib_json",
        "auto_zero_on_activate",
        "apply_home_on_activate",
        "home_j1_rad",
        "home_j2_rad",
        "home_j3_rad",
        "home_j4_rad",
        "home_j5_rad",
        "home_j6_rad",
        "params_file",
    } <= argument_names
