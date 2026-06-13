# VR Teleop Safety Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** SoArmVR から follower 実機までを一つの launch で起動し、QoS と IK 指令を安全にする。

**Architecture:** `teleop_ik` に統合 launch と Python 単体テストを追加する。IK node は Pose のみ BestEffort で購読し、URDF 制限へ clamp した上で、非収束時には指令を生成しない。

**Tech Stack:** ROS 2 Jazzy, rclpy, launch, Pinocchio, pytest, colcon

---

### Task 1: IK 安全動作

**Files:**
- Create: `ros2_ws/src/teleop_ik/test/test_ik_node.py`
- Modify: `ros2_ws/src/teleop_ik/teleop_ik/ik_node.py`

- [ ] clamp と非収束時 `None` の失敗テストを追加する。
- [ ] 対象テストを実行し、非収束時に配列が返るため失敗することを確認する。
- [ ] 非収束時に `None` を返し、関節制限 clamp を維持する。
- [ ] 対象テストを再実行して成功を確認する。
- [ ] 変更をコミットする。

### Task 2: Pose QoS

**Files:**
- Create: `ros2_ws/src/teleop_ik/test/test_qos.py`
- Modify: `ros2_ws/src/teleop_ik/teleop_ik/ik_node.py`

- [ ] Pose subscription が BestEffort であることを検査する失敗テストを追加する。
- [ ] 対象テストを実行して失敗を確認する。
- [ ] Pose subscription のみ BestEffort / Volatile に変更する。
- [ ] 対象テストを再実行して成功を確認する。
- [ ] 変更をコミットする。

### Task 3: VR テレオペ統合 launch

**Files:**
- Create: `ros2_ws/src/teleop_ik/launch/vr_teleop.launch.py`
- Create: `ros2_ws/src/teleop_ik/test/test_vr_teleop_launch.py`

- [ ] follower controller と IK node の起動を検査する失敗テストを追加する。
- [ ] 対象テストを実行して launch 不在による失敗を確認する。
- [ ] follower 実機 launch を include し、IK node を起動する統合 launch を追加する。
- [ ] 対象テストと launch 引数表示を実行して成功を確認する。
- [ ] 変更をコミットする。

### Task 4: 統合検証

**Files:**
- Modify: `ros2_ws/README.md`

- [ ] README に VR テレオペ起動コマンドを追加する。
- [ ] `colcon build --symlink-install` を実行する。
- [ ] `teleop_ik` の対象テストを実行する。
- [ ] QoS endpoint と launch 起動構成を動的に確認する。
- [ ] 変更をコミットし、push と PR 作成を行う。
