from vrteleop_bridge.config import parse_config
from vrteleop_bridge.coordinates import pose_to_target
from vrteleop_bridge.ik import GeometricSO101IKSolver, JOINT_NAMES
from vrteleop_bridge.types import ControllerPose


def test_pose_to_target_maps_and_clamps_position() -> None:
    config = parse_config(
        {
            "udp_host": "127.0.0.1",
            "udp_port": 50530,
            "recv_timeout_sec": 0.5,
            "control_hz": 30,
            "max_step_deg": 4.0,
            "workspace": {
                "x_min": 0.08,
                "x_max": 0.32,
                "y_min": -0.22,
                "y_max": 0.22,
                "z_min": 0.02,
                "z_max": 0.32,
            },
            "coordinate_map": {
                "robot_x": "-godot_z",
                "robot_y": "-godot_x",
                "robot_z": "godot_y",
                "origin_offset": [0.0, -1.0, 0.0],
                "scale": 0.35,
            },
            "ik": {
                "base_height_m": 0.045,
                "upper_arm_m": 0.115,
                "forearm_m": 0.135,
                "wrist_m": 0.055,
                "joint_limits_deg": {name: [-180.0, 180.0] for name in JOINT_NAMES},
            },
        }
    )
    pose = ControllerPose(
        seq=1,
        timestamp_usec=1,
        position=(0.1, 1.2, -0.6),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        trigger=0.5,
        grip=1.0,
        enabled=True,
    )
    target = pose_to_target(pose, config.coordinate_map, config.workspace)
    assert target.position == (0.21, -0.034999999999999996, 0.06999999999999998)
    assert target.gripper == 50.0


def test_geometric_solver_returns_all_so101_action_keys() -> None:
    config = parse_config(
        {
            "udp_host": "127.0.0.1",
            "udp_port": 50530,
            "recv_timeout_sec": 0.5,
            "control_hz": 30,
            "max_step_deg": 4.0,
            "workspace": {
                "x_min": 0.08,
                "x_max": 0.32,
                "y_min": -0.22,
                "y_max": 0.22,
                "z_min": 0.02,
                "z_max": 0.32,
            },
            "coordinate_map": {
                "robot_x": "-godot_z",
                "robot_y": "-godot_x",
                "robot_z": "godot_y",
                "origin_offset": [0.0, -1.0, 0.0],
                "scale": 0.35,
            },
            "ik": {
                "base_height_m": 0.045,
                "upper_arm_m": 0.115,
                "forearm_m": 0.135,
                "wrist_m": 0.055,
                "joint_limits_deg": {
                    "shoulder_pan.pos": [-100.0, 100.0],
                    "shoulder_lift.pos": [-95.0, 95.0],
                    "elbow_flex.pos": [-110.0, 110.0],
                    "wrist_flex.pos": [-100.0, 100.0],
                    "wrist_roll.pos": [-180.0, 180.0],
                    "gripper.pos": [0.0, 100.0],
                },
            },
        }
    )
    solver = GeometricSO101IKSolver(config.ik)
    action = solver.solve(
        pose_to_target(
            ControllerPose(
                seq=1,
                timestamp_usec=1,
                position=(0.0, 1.1, -0.5),
                orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
                trigger=1.0,
                grip=1.0,
                enabled=True,
            ),
            config.coordinate_map,
            config.workspace,
        )
    )
    assert set(action) == set(JOINT_NAMES)
    assert action["gripper.pos"] == 100.0
