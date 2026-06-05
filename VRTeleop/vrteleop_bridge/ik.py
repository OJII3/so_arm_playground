from __future__ import annotations

import math

from .config import IKConfig
from .types import JointAction, RobotTarget


JOINT_NAMES = (
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
)


class IKSolver:
    def solve(self, target: RobotTarget) -> JointAction:
        raise NotImplementedError


class GeometricSO101IKSolver(IKSolver):
    def __init__(self, config: IKConfig) -> None:
        self.config = config

    def solve(self, target: RobotTarget) -> JointAction:
        x, y, z = target.position
        yaw = math.atan2(y, x)
        radial = math.hypot(x, y) - self.config.wrist_m
        height = z - self.config.base_height_m

        shoulder, elbow = _solve_two_link(
            radial,
            height,
            self.config.upper_arm_m,
            self.config.forearm_m,
        )
        pitch, _roll, _yaw = _quat_to_euler_xyz(target.orientation_xyzw)
        wrist_flex = pitch - shoulder - elbow
        wrist_roll = _roll

        action = {
            "shoulder_pan.pos": math.degrees(yaw),
            "shoulder_lift.pos": math.degrees(shoulder),
            "elbow_flex.pos": math.degrees(elbow),
            "wrist_flex.pos": math.degrees(wrist_flex),
            "wrist_roll.pos": math.degrees(wrist_roll),
            "gripper.pos": target.gripper,
        }
        return {
            name: _clamp(value, *self.config.joint_limits_deg[name])
            for name, value in action.items()
        }


class RateLimitedIKSolver(IKSolver):
    def __init__(self, inner: IKSolver, max_step_deg: float) -> None:
        self.inner = inner
        self.max_step_deg = max_step_deg
        self._previous: JointAction | None = None

    def solve(self, target: RobotTarget) -> JointAction:
        raw = self.inner.solve(target)
        if self._previous is None:
            self._previous = raw
            return raw
        limited = {
            name: self._previous[name] + _clamp(raw[name] - self._previous[name], -self.max_step_deg, self.max_step_deg)
            for name in raw
        }
        self._previous = limited
        return limited


def _solve_two_link(x: float, z: float, link_a: float, link_b: float) -> tuple[float, float]:
    distance = math.hypot(x, z)
    reachable = _clamp(distance, abs(link_a - link_b) + 1e-6, link_a + link_b - 1e-6)
    cos_elbow = (reachable * reachable - link_a * link_a - link_b * link_b) / (2.0 * link_a * link_b)
    elbow = math.acos(_clamp(cos_elbow, -1.0, 1.0))
    shoulder_offset = math.atan2(link_b * math.sin(elbow), link_a + link_b * math.cos(elbow))
    shoulder = math.atan2(z, x) - shoulder_offset
    return shoulder, elbow


def _quat_to_euler_xyz(q: tuple[float, float, float, float]) -> tuple[float, float, float]:
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return pitch, roll, yaw


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)
