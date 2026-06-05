from __future__ import annotations

from .config import CoordinateMap, Workspace
from .types import ControllerPose, RobotTarget


def pose_to_target(pose: ControllerPose, coordinate_map: CoordinateMap, workspace: Workspace) -> RobotTarget:
    translated = tuple(p + offset for p, offset in zip(pose.position, coordinate_map.origin_offset))
    scaled = tuple(v * coordinate_map.scale for v in translated)
    x = _axis_value(coordinate_map.robot_x, scaled)
    y = _axis_value(coordinate_map.robot_y, scaled)
    z = _axis_value(coordinate_map.robot_z, scaled)
    clamped = (
        _clamp(x, workspace.x_min, workspace.x_max),
        _clamp(y, workspace.y_min, workspace.y_max),
        _clamp(z, workspace.z_min, workspace.z_max),
    )
    return RobotTarget(
        position=clamped,
        orientation_xyzw=pose.orientation_xyzw,
        gripper=pose.trigger * 100.0,
    )


def _axis_value(spec: str, values: tuple[float, float, float]) -> float:
    sign = -1.0 if spec.startswith("-") else 1.0
    axis = spec[1:] if spec.startswith("-") else spec
    index = {"godot_x": 0, "godot_y": 1, "godot_z": 2}[axis]
    return sign * values[index]


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)
