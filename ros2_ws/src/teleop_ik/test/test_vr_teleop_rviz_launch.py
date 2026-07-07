import importlib.util
from pathlib import Path

import pytest

from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch_ros.actions import Node


LAUNCH_PATH = (
    Path(__file__).parents[1] / "launch" / "vr_teleop_rviz.launch.py"
)

EXPECTED_NODE_NAMESPACE = "follower"
EXPECTED_CONTROLLER_MANAGER_URI = "/follower/controller_manager"
EXPECTED_SPAWNER_PERIOD_SEC = 2.0
EXPECTED_TELEOP_IK_RVIZ_PERIOD_SEC = 4.0
EXPECTED_SPAWNER_CONTROLLERS = {
    "joint_state_broadcaster",
    "arm_controller",
    "gripper_controller",
}
EXPECTED_TOP_LEVEL_NODE_KINDS = {
    ("robot_state_publisher", "robot_state_publisher"),
    ("controller_manager", "ros2_control_node"),
    ("controller_manager", "spawner"),
    ("rviz2", "rviz2"),
}


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


def _argv_strings(node, context):
    """Return a Node's argv as a list of plain strings.

    Tries the post-visit `final_cmd` first (substitutions expanded). Falls
    back to flattening the pre-visit `cmd` (which contains substitution
    objects) by stringifying each token. The fallback is enough for
    fixed-argument spawners and rviz2 because their argv doesn't depend on
    launch-time substitutions for the args we care about.
    """
    final = node.process_description.final_cmd
    if final is not None:
        return list(final)
    tokens = []
    for sub_list in node.process_description.cmd:
        parts = []
        for sub in sub_list:
            if isinstance(sub, str):
                parts.append(sub)
            else:
                # TextSubstitution exposes its literal text via `.text`.
                text = getattr(sub, "text", None)
                if text is not None:
                    parts.append(text)
                else:
                    parts.append(repr(sub))
        tokens.append("".join(parts))
    return tokens


def _spawner_controller_name(node):
    tokens = _argv_strings(node, None)
    for token in tokens[1:]:
        if not token.startswith("--"):
            return token
    return None


def _spawner_controller_manager_uri(node):
    tokens = _argv_strings(node, None)
    for i, token in enumerate(tokens):
        if token == "--controller-manager" and i + 1 < len(tokens):
            return tokens[i + 1]
    return None


def _safe_visit(action, context):
    """Visit a launch action to expand its substitutions.

    Node.visit() can raise RuntimeError (parameter validation), AttributeError
    (no asyncio loop in this synthetic context), or similar non-fatal errors
    during substitution expansion. We only need enough expansion to verify the
    launch structure, so swallow the known transient errors and let the
    public `expanded_node_namespace` getter return its (possibly sentinel)
    value.
    """
    try:
        action.visit(context)
    except (RuntimeError, AttributeError, TypeError):
        pass


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

    missing = EXPECTED_TOP_LEVEL_NODE_KINDS - node_kinds
    assert not missing, f"Missing top-level nodes: {missing}"


def test_vr_teleop_rviz_launch_spawns_required_controllers():
    launch_description = _load_launch_module().generate_launch_description()
    spawner_nodes = [
        entity
        for entity in _flatten(launch_description.entities)
        if isinstance(entity, Node)
        and entity.node_package == "controller_manager"
        and entity.node_executable == "spawner"
    ]
    spawned_controllers = {
        name for name in (_spawner_controller_name(n) for n in spawner_nodes) if name
    }

    missing = EXPECTED_SPAWNER_CONTROLLERS - spawned_controllers
    assert not missing, f"Missing spawner controllers: {missing}"


def test_vr_teleop_rviz_launch_spawner_targets_follower_controller_manager():
    launch_description = _load_launch_module().generate_launch_description()
    spawner_nodes = [
        entity
        for entity in _flatten(launch_description.entities)
        if isinstance(entity, Node)
        and entity.node_package == "controller_manager"
        and entity.node_executable == "spawner"
    ]

    assert spawner_nodes, "no controller_manager spawner nodes found"
    for node in spawner_nodes:
        uri = _spawner_controller_manager_uri(node)
        assert uri == EXPECTED_CONTROLLER_MANAGER_URI, (
            f"spawner {node.name!r} targets wrong controller-manager URI: {uri!r}"
        )


def test_vr_teleop_rviz_launch_timer_periods():
    launch_description = _load_launch_module().generate_launch_description()
    timer_actions = [
        e for e in launch_description.entities if isinstance(e, TimerAction)
    ]

    spawners_timer = next(
        (e for e in timer_actions if e.period == EXPECTED_SPAWNER_PERIOD_SEC),
        None,
    )
    teleop_ik_rviz_timer = next(
        (e for e in timer_actions if e.period == EXPECTED_TELEOP_IK_RVIZ_PERIOD_SEC),
        None,
    )

    assert spawners_timer is not None, (
        f"TimerAction(period={EXPECTED_SPAWNER_PERIOD_SEC}) for spawners not found"
    )
    assert teleop_ik_rviz_timer is not None, (
        f"TimerAction(period={EXPECTED_TELEOP_IK_RVIZ_PERIOD_SEC}) for teleop_ik + RViz not found"
    )


def test_vr_teleop_rviz_launch_rviz_uses_dash_d_config_arg():
    launch_description = _load_launch_module().generate_launch_description()
    rviz_nodes = [
        entity
        for entity in _flatten(launch_description.entities)
        if isinstance(entity, Node) and entity.node_package == "rviz2"
    ]

    assert rviz_nodes, "no rviz2 node found"
    rviz = rviz_nodes[0]
    tokens = _argv_strings(rviz, None)
    assert tokens, "rviz2 node has no argv tokens"
    # The executable is wrapped in an ExecutableInPackage substitution, so
    # the pre-visit token is something like
    # "PathJoinSubstitution('ExecInPkg(pkg='rviz2', exec='rviz2')')".
    # Just check that -d is one of the arguments.
    assert "-d" in tokens, (
        f"rviz2 must be invoked with -d, got tokens={tokens!r}"
    )


def test_vr_teleop_rviz_launch_includes_teleop_ik_launch():
    launch_description = _load_launch_module().generate_launch_description()
    includes = [
        entity
        for entity in _flatten(launch_description.entities)
        if isinstance(entity, IncludeLaunchDescription)
    ]

    assert len(includes) == 1
    ctx = LaunchContext()
    _safe_visit(includes[0], ctx)
    assert "teleop_ik.launch.py" in includes[0].launch_description_source.location
    assert "PythonLaunchDescriptionSource" in type(
        includes[0].launch_description_source
    ).__name__


def test_vr_teleop_rviz_launch_forwards_args_to_teleop_ik_include():
    launch_description = _load_launch_module().generate_launch_description()
    includes = [
        entity
        for entity in _flatten(launch_description.entities)
        if isinstance(entity, IncludeLaunchDescription)
    ]

    assert len(includes) == 1
    forwarded = dict(includes[0].launch_arguments)
    assert forwarded.get("params_file") is not None, "params_file not forwarded"
    assert forwarded.get("urdf_path") is not None, "urdf_path not forwarded"


def test_vr_teleop_rviz_launch_namespaces_follower():
    launch_description = _load_launch_module().generate_launch_description()
    nodes = [
        entity
        for entity in _flatten(launch_description.entities)
        if isinstance(entity, Node)
    ]

    ctx = LaunchContext()
    for n in nodes:
        _safe_visit(n, ctx)

    checked = [
        n
        for n in nodes
        if (n.node_package, n.node_executable)
        in {
            ("robot_state_publisher", "robot_state_publisher"),
            ("controller_manager", "ros2_control_node"),
        }
    ]
    assert checked, "expected robot_state_publisher and controller_manager nodes to be present"
    assert all(
        n.expanded_node_namespace == f"/{EXPECTED_NODE_NAMESPACE}" for n in checked
    ), "robot_state_publisher / controller_manager must be in /follower namespace"


def test_vr_teleop_rviz_launch_includes_preflight_check():
    launch_description = _load_launch_module().generate_launch_description()
    preflight = [
        e for e in launch_description.entities if isinstance(e, OpaqueFunction)
    ]
    assert preflight, "expected an OpaqueFunction for preflight file checks"


def test_vr_teleop_rviz_launch_preflight_passes_with_default_paths():
    launch_description = _load_launch_module().generate_launch_description()
    preflight = next(
        e for e in launch_description.entities if isinstance(e, OpaqueFunction)
    )

    ctx = LaunchContext()
    # DeclareLaunchArgument.visit() registers default values in the context
    # so LaunchConfiguration.perform() can resolve them.
    for entity in launch_description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            _safe_visit(entity, ctx)

    # Default paths come from the install/ tree; they should resolve to
    # existing files. If the workspace is not built (no install/), this
    # raises FileNotFoundError and the test fails loudly.
    result = preflight.execute(ctx)
    assert result in (None, [])


def test_vr_teleop_rviz_launch_preflight_raises_on_missing_path():
    launch_description = _load_launch_module().generate_launch_description()
    preflight = next(
        e for e in launch_description.entities if isinstance(e, OpaqueFunction)
    )

    ctx = LaunchContext()
    # Register overrides that point at non-existent files for every
    # preflight-checked path.
    for name in ("urdf_path", "controllers_file", "rviz_config", "params_file"):
        ctx.launch_configurations[name] = "/nonexistent/__missing__.fake"

    with pytest.raises(FileNotFoundError) as excinfo:
        preflight.execute(ctx)
    assert "vr_teleop_rviz" in str(excinfo.value)
