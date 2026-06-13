# VR Teleop Safety Design

## Goal

SoArmVR から SO-101 follower 実機までの必要ノードを一つの launch で起動し、
Pose の QoS を SoArmVR と一致させ、IK 非収束時に危険な関節指令を送らない。

## Architecture

- `teleop_ik/launch/vr_teleop.launch.py` が follower controller launch と
  `teleop_ik_node` を起動する。
- `/teleop/target_pose` の subscription だけを BestEffort / Volatile にする。
  `/teleop/active` と `/teleop/gripper` は Reliable のまま維持する。
- IK の各反復では URDF の関節上下限へ clamp する。
- 最大反復数までに位置誤差が許容値以下にならなければ `None` を返し、
  JointTrajectory を publish しない。

## Launch Interface

`vr_teleop.launch.py` は follower 実機起動に必要な引数を公開し、
`so101_follower_controller.launch.py` へ渡す。

- `usb_port`
- `calib_json`
- `auto_zero_on_activate`
- `apply_home_on_activate`
- `home_j1_rad` から `home_j6_rad`
- `params_file`

VR テレオペ用途なので follower の `is_sim` は `False` に固定する。

## Testing

- IK node の endpoint QoS を実行時に検査する。
- URDF 制限を越える値が clamp されることを単体テストする。
- IK 非収束時に `None` が返ることを単体テストする。
- 統合 launch の構文と公開引数を検証する。
- `colcon build` と対象パッケージのテストを実行する。
