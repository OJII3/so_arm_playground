from __future__ import annotations

import argparse
import time

from .backends import DryRunBackend, LeRobotSO101Backend, MujocoBackend
from .config import load_config
from .coordinates import pose_to_target
from .ik import GeometricSO101IKSolver, RateLimitedIKSolver
from .pose_receiver import UdpPoseReceiver


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Quest 3 OpenXR pose to SO-101 command bridge.")
    parser.add_argument("--config", default="config/default.json")
    parser.add_argument("--backend", choices=("dry-run", "real-lerobot", "mujoco"), default="dry-run")
    parser.add_argument("--port", help="Serial port for SO-101 follower.")
    parser.add_argument("--robot-id", default="so101_follower")
    parser.add_argument("--mjcf", help="MuJoCo XML path.")
    args = parser.parse_args(argv)

    config = load_config(args.config)
    solver = RateLimitedIKSolver(GeometricSO101IKSolver(config.ik), config.max_step_deg)
    backend = _make_backend(args, config.max_step_deg)
    period = 1.0 / config.control_hz

    print(f"Listening on udp://{config.udp_host}:{config.udp_port}; backend={args.backend}")
    with UdpPoseReceiver(config.udp_host, config.udp_port, config.recv_timeout_sec) as receiver, backend:
        while True:
            started = time.monotonic()
            pose = receiver.recv()
            if pose is not None and pose.enabled:
                target = pose_to_target(pose, config.coordinate_map, config.workspace)
                action = solver.solve(target)
                backend.send(action)
            elapsed = time.monotonic() - started
            if elapsed < period:
                time.sleep(period - elapsed)


def _make_backend(args: argparse.Namespace, max_step_deg: float):
    if args.backend == "dry-run":
        return DryRunBackend()
    if args.backend == "real-lerobot":
        if not args.port:
            raise SystemExit("--port is required for --backend real-lerobot")
        return LeRobotSO101Backend(args.port, args.robot_id, max_step_deg)
    if args.backend == "mujoco":
        if not args.mjcf:
            raise SystemExit("--mjcf is required for --backend mujoco")
        return MujocoBackend(args.mjcf)
    raise SystemExit(f"Unsupported backend: {args.backend}")
