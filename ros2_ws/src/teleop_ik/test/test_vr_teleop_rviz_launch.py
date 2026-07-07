import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node


LAUNCH_PATH = (
    Path(__file__).parents[1] / "launch" / "vr_teleop_rviz.launch.py"
)


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("vr_teleop_rviz_launch", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _flatten(entities):
    for entity in entities:
        if isinstance(entity, TimerAction):
            yield from _flatten(entity.actions)
        else:
            yield entity


def _visit_nodes(nodes):
    ctx = LaunchContext()
    for n in nodes:
        try:
            n.visit(ctx)
        except Exception:
            pass
    return nodes


def test_vr_teleop_rviz_launch_exposes_arguments():
    launch_description = _load_launch_module().generate_launch_description()
    argument_names = {
        entity.name
        for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }

    assert {
        "urdf_path",
        "controllers_file",
        "rviz_config",
        "params_file",
    } <= argument_names


def test_vr_teleop_rviz_launch_spawns_required_nodes():
    launch_description = _load_launch_module().generate_launch_description()
    nodes = [
        entity
        for entity in _flatten(launch_description.entities)
        if isinstance(entity, Node)
    ]
    node_kinds = {(n.node_package, n.node_executable) for n in nodes}

    assert ("robot_state_publisher", "robot_state_publisher") in node_kinds
    assert ("controller_manager", "ros2_control_node") in node_kinds
    assert ("rviz2", "rviz2") in node_kinds


def test_vr_teleop_rviz_launch_includes_teleop_ik_launch():
    launch_description = _load_launch_module().generate_launch_description()
    includes = [
        entity
        for entity in _flatten(launch_description.entities)
        if isinstance(entity, IncludeLaunchDescription)
    ]

    assert len(includes) == 1
    ctx = LaunchContext()
    for inc in includes:
        try:
            inc.visit(ctx)
        except Exception:
            pass
    assert any("teleop_ik.launch.py" in entity.launch_description_source.location for entity in includes)
    assert "PythonLaunchDescriptionSource" in type(
        includes[0].launch_description_source
    ).__name__


def test_vr_teleop_rviz_launch_namespaces_follower():
    launch_description = _load_launch_module().generate_launch_description()
    nodes = [
        entity
        for entity in _flatten(launch_description.entities)
        if isinstance(entity, Node)
    ]

    assert all(
        node.expanded_node_namespace == "/follower"
        for node in _visit_nodes(nodes)
        if (node.node_package, node.node_executable) in {
            ("robot_state_publisher", "robot_state_publisher"),
            ("controller_manager", "ros2_control_node"),
        }
    )
