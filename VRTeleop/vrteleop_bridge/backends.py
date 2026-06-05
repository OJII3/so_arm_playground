from __future__ import annotations

import math
from contextlib import AbstractContextManager

from .ik import JOINT_NAMES
from .types import JointAction


class CommandBackend(AbstractContextManager["CommandBackend"]):
    def send(self, action: JointAction) -> None:
        raise NotImplementedError

    def __enter__(self) -> "CommandBackend":
        return self

    def __exit__(self, *exc: object) -> None:
        return None


class DryRunBackend(CommandBackend):
    def send(self, action: JointAction) -> None:
        formatted = ", ".join(f"{name}={action[name]:.2f}" for name in JOINT_NAMES)
        print(formatted)


class LeRobotSO101Backend(CommandBackend):
    def __init__(self, port: str, robot_id: str, max_relative_target: float) -> None:
        self.port = port
        self.robot_id = robot_id
        self.max_relative_target = max_relative_target
        self._robot = None

    def __enter__(self) -> "LeRobotSO101Backend":
        from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

        config = SOFollowerRobotConfig(
            robot_type="so101_follower",
            id=self.robot_id,
            port=self.port,
            disable_torque_on_disconnect=True,
            max_relative_target=self.max_relative_target,
            use_degrees=True,
        )
        self._robot = SOFollower(config)
        self._robot.connect()
        return self

    def __exit__(self, *exc: object) -> None:
        if self._robot is not None:
            self._robot.disconnect()
            self._robot = None

    def send(self, action: JointAction) -> None:
        if self._robot is None:
            raise RuntimeError("LeRobot backend is not connected.")
        self._robot.send_action(action)


class MujocoBackend(CommandBackend):
    def __init__(self, mjcf_path: str, step_count: int = 4) -> None:
        self.mjcf_path = mjcf_path
        self.step_count = step_count
        self._mujoco = None
        self._model = None
        self._data = None
        self._actuator_ids: dict[str, int] = {}

    def __enter__(self) -> "MujocoBackend":
        import mujoco

        self._mujoco = mujoco
        self._model = mujoco.MjModel.from_xml_path(self.mjcf_path)
        self._data = mujoco.MjData(self._model)
        self._actuator_ids = {
            name: mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in JOINT_NAMES
        }
        missing = [name for name, actuator_id in self._actuator_ids.items() if actuator_id < 0]
        if missing:
            raise RuntimeError(f"MuJoCo model is missing actuators: {', '.join(missing)}")
        return self

    def send(self, action: JointAction) -> None:
        if self._mujoco is None or self._model is None or self._data is None:
            raise RuntimeError("MuJoCo backend is not open.")
        for name in JOINT_NAMES:
            value = action[name]
            if name == "gripper.pos":
                low, high = self._model.actuator_ctrlrange[self._actuator_ids[name]]
                value = low + (high - low) * (value / 100.0)
            else:
                value = math.radians(value)
            self._data.ctrl[self._actuator_ids[name]] = value
        for _ in range(self.step_count):
            self._mujoco.mj_step(self._model, self._data)
