from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class Workspace:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float


@dataclass(frozen=True)
class CoordinateMap:
    robot_x: str
    robot_y: str
    robot_z: str
    origin_offset: tuple[float, float, float]
    scale: float


@dataclass(frozen=True)
class IKConfig:
    base_height_m: float
    upper_arm_m: float
    forearm_m: float
    wrist_m: float
    joint_limits_deg: dict[str, tuple[float, float]]


@dataclass(frozen=True)
class BridgeConfig:
    udp_host: str
    udp_port: int
    recv_timeout_sec: float
    control_hz: float
    max_step_deg: float
    workspace: Workspace
    coordinate_map: CoordinateMap
    ik: IKConfig


def load_config(path: str | Path) -> BridgeConfig:
    data = json.loads(Path(path).read_text())
    return parse_config(data)


def parse_config(data: dict[str, Any]) -> BridgeConfig:
    workspace = Workspace(**data["workspace"])
    coord = data["coordinate_map"]
    coordinate_map = CoordinateMap(
        robot_x=coord["robot_x"],
        robot_y=coord["robot_y"],
        robot_z=coord["robot_z"],
        origin_offset=tuple(coord["origin_offset"]),
        scale=float(coord["scale"]),
    )
    ik_data = data["ik"]
    limits = {
        name: (float(value[0]), float(value[1]))
        for name, value in ik_data["joint_limits_deg"].items()
    }
    ik = IKConfig(
        base_height_m=float(ik_data["base_height_m"]),
        upper_arm_m=float(ik_data["upper_arm_m"]),
        forearm_m=float(ik_data["forearm_m"]),
        wrist_m=float(ik_data["wrist_m"]),
        joint_limits_deg=limits,
    )
    return BridgeConfig(
        udp_host=data["udp_host"],
        udp_port=int(data["udp_port"]),
        recv_timeout_sec=float(data["recv_timeout_sec"]),
        control_hz=float(data["control_hz"]),
        max_step_deg=float(data["max_step_deg"]),
        workspace=workspace,
        coordinate_map=coordinate_map,
        ik=ik,
    )
