from __future__ import annotations

from dataclasses import dataclass


JointAction = dict[str, float]


@dataclass(frozen=True)
class ControllerPose:
    seq: int
    timestamp_usec: int
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    trigger: float
    grip: float
    enabled: bool


@dataclass(frozen=True)
class RobotTarget:
    position: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    gripper: float
