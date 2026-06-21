# SoArmVR: アンカー投影 & スティック駆動手首 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Unity 側でアンカーをヨーのみ保持する投影に変更し、右スティックの入力を `teleop_ik/msg/TargetPoseWithInput` 経由で ROS 2 に渡し、IK 側でスティックを dt 積分して joint 4/5 を駆動する。既存の `/teleop/target_pose` (PoseStamped) は廃止する。

**Architecture:**
- 単一の新規 ROS msg `TargetPoseWithInput` (Header + Pose + float32 stick_x / float32 stick_y) を `teleop_ik` パッケージに追加し、`/teleop/target` で通信する。
- アンカーは Unity 側で `Quaternion.LookRotation(水平投影 forward, Vector3.up)` を使い、ヨーのみ抽出する。
- Unity 側は右コントローラの `primary2DAxis` を sample に乗せて publish。
- ROS 側は `header.stamp` から dt を求め、deadzone → 1 メッセージあたり上限クランプ → 積分の順で `stick` を処理し、`wrist_init_pos + (stick_x → j5, stick_y → j4)` を joint 4/5 目標として position IK (joint 1〜3) に渡す。

**Tech Stack:** ROS 2 Jazzy (ament_python + rosidl), Pinocchio, Unity (C#, Input System, ROSettaDDS), pytest, ROSettaDDS genmsg.

**Spec:** `docs/superpowers/specs/2026-06-21-soarmvr-anchor-projection-stick-wrist-design.md`

---

## File Structure

### New files (ROS 2)
- `ros2_ws/src/teleop_ik/msg/TargetPoseWithInput.msg` — 新規 ROS msg
- `ros2_ws/src/teleop_ik/CMakeLists.txt` — rosidl + Python モジュール / launch / config / エントリポイントのインストール定義
- `ros2_ws/src/teleop_ik/test/test_target_msg.py` — msg 取り込みテスト
- (生成物) `install/teleop_ik/lib/python3.12/site-packages/teleop_ik/msg/_target_pose_with_input.py` ほか

> 実装着手時に判明した ROS 2 Jazzy 制約: `geometry_msgs` には `Vector2` が存在しないため、
> `stick` は `Vector2` でなく `float32 stick_x` / `float32 stick_y` の 2 フィールドで持つ。
> さらに `ament_python` パッケージでは `rosidl_generate_interfaces` を呼べないため、
> `teleop_ik` を `ament_cmake` ビルドタイプへ移行する。

### New files (Unity)
- `SoArmVR/Assets/_SoArmVR/Msgs/teleop_ik/msg/TargetPoseWithInput.msg` — Unity 用ミラーファイル
- (生成物) `SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/TeleopIk/TargetPoseWithInput.cs` およびシリアライザ

### Modified files (ROS 2)
- `ros2_ws/src/teleop_ik/package.xml` — rosidl 依存追加
- `ros2_ws/src/teleop_ik/setup.py` — 変更不要 (CMakeLists と並置)
- `ros2_ws/src/teleop_ik/teleop_ik/ik_node.py` — スティック積分化、新 msg 購読、`unity_quaternion_to_pitch_roll` 削除
- `ros2_ws/src/teleop_ik/teleop_ik/gamepad_node.py` — 新 msg publish 化
- `ros2_ws/src/teleop_ik/teleop_ik/coordinate_utils.py` — `unity_quaternion_to_pitch_roll` 削除
- `ros2_ws/src/teleop_ik/config/teleop_ik_params.yaml` — スティック系パラメータ追加
- `ros2_ws/src/teleop_ik/test/test_ik_node.py` — テスト追加・更新
- `ros2_ws/src/teleop_ik/test/test_coordinate_utils.py` — pitch/roll 関連テスト削除

### Modified files (Unity)
- `SoArmVR/Assets/_SoArmVR/Input/SoArmTeleoperation.inputactions` — `Stick` アクション追加
- `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationSample.cs` — `Vector2 stick` 追加(Unity 内部では `Vector2` として扱い、publish 時に `float stick_x/stick_y` に展開)
- `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationAnchor.cs` — ヨー抽出の `Place` 実装
- `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationSession.cs` — `_stickAction` 読み取り
- `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/RosTeleoperationSink.cs` — 新 msg publish 化
- `SoArmVR/Assets/_SoArmVR/Prefabs/Teleoperation.prefab` — `_stickAction` 結線

---

## Task 1: 新規 ROS msg `TargetPoseWithInput` 定義を追加

**Files:**
- Create: `ros2_ws/src/teleop_ik/msg/TargetPoseWithInput.msg`
- Create: `ros2_ws/src/teleop_ik/CMakeLists.txt`
- Modify: `ros2_ws/src/teleop_ik/package.xml`

- [ ] **Step 1: msg ファイルを作成**

`ros2_ws/src/teleop_ik/msg/TargetPoseWithInput.msg` を以下の内容で作成:

```
std_msgs/Header header
geometry_msgs/Pose pose
float32 stick_x
float32 stick_y
```

- [ ] **Step 2: CMakeLists.txt を作成**

`ros2_ws/src/teleop_ik/CMakeLists.txt` を以下の内容で作成(ament_cmake に移行し、msg 生成 + Python モジュール / launch / config / エントリポイントをインストール):

```cmake
cmake_minimum_required(VERSION 3.16)
project(teleop_ik)

find_package(Python3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

string(REGEX REPLACE "^.*/lib/" "lib/" python_site_packages "${Python3_SITELIB}")

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/TargetPoseWithInput.msg
  DEPENDENCIES geometry_msgs std_msgs
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  ${PROJECT_NAME}
  DESTINATION ${python_site_packages}
)

set(node_scripts
  ik_node teleop_ik.ik_node:main
  gamepad_node teleop_ik.gamepad_node:main
)
foreach(script_entry ${node_scripts})
  list(POP_FRONT script_entry script_name script_module)
  set(script_path "${CMAKE_CURRENT_BINARY_DIR}/${script_name}")
  file(WRITE "${script_path}"
"#!/usr/bin/env python3
from ${script_module} import main
if __name__ == '__main__':
    main()
")
  install(PROGRAMS "${script_path}" DESTINATION lib/${PROJECT_NAME})
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_target_msg
    test/test_target_msg.py
    APPEND_ENV "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${python_site_packages}"
  )
endif()

ament_package()
```

> 既存の `setup.py` は不要になるため削除する(`ament_cmake` への完全移行)。

- [ ] **Step 3: `package.xml` を更新**

`ros2_ws/src/teleop_ik/package.xml` を以下に置換:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>teleop_ik</name>
  <version>0.0.1</version>
  <description>VR teleop IK node for SO-101 arm using Pinocchio</description>
  <maintainer email="ojii3dev@gmail.com">OJII3</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>trajectory_msgs</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>joy</exec_depend>
  <exec_depend>lerobot_controller</exec_depend>
  <exec_depend>lerobot_description</exec_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 4: 旧 `setup.py` を削除**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
git rm ros2_ws/src/teleop_ik/setup.py
```

> 削除する理由: `ament_cmake` 移行により CMakeLists.txt が Python モジュールと
> エントリポイントのインストールを担うため、`setup.py` は冗長。

- [ ] **Step 5: colcon build 通過確認**

```bash
cd ros2_ws
nix develop ../.#ros --command bash -lc 'colcon build --packages-select teleop_ik --cmake-args -DBUILD_TESTING=OFF'
```

期待結果: `Starting >>> teleop_ik` → `Finished >>> teleop_ik` がエラー無しで完了。

- [ ] **Step 6: コミット**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
git add ros2_ws/src/teleop_ik/msg/TargetPoseWithInput.msg \
        ros2_ws/src/teleop_ik/CMakeLists.txt \
        ros2_ws/src/teleop_ik/package.xml
git rm ros2_ws/src/teleop_ik/setup.py
git commit -m "feat(teleop_ik): add TargetPoseWithInput msg and migrate to ament_cmake"
```

> メモ: 既存の `e14b740` / `1fef7d9` / `e26a253` / `dda7736` でこのタスクの主要な
> 作業は既にコミット済み(Task 2 着手時に implementer が先回り)。本タスクは
> 設計ドキュメントの更新と `setup.py` 削除のクリーンアップの位置づけ。

---

## Task 2: msg 取り込みテストを追加

**Files:**
- Create: `ros2_ws/src/teleop_ik/test/test_target_msg.py`

- [ ] **Step 1: テストを作成**

`ros2_ws/src/teleop_ik/test/test_target_msg.py`:

```python
"""TargetPoseWithInput msg がビルド成果物から import できることを確認.

このテストは colcon build 完了後の環境でのみ pass する.
"""

from teleop_ik.msg import TargetPoseWithInput  # type: ignore[attr-defined]


def test_target_pose_with_input_instantiation():
    msg = TargetPoseWithInput()
    assert hasattr(msg, "header")
    assert hasattr(msg, "pose")
    assert hasattr(msg, "stick_x")
    assert hasattr(msg, "stick_y")
    assert msg.stick_x == 0.0
    assert msg.stick_y == 0.0


def test_target_pose_with_input_set_fields():
    msg = TargetPoseWithInput()
    msg.stick_x = 0.5
    msg.stick_y = -0.25
    assert msg.stick_x == 0.5
    assert msg.stick_y == -0.25
```

- [ ] **Step 2: テストを colcon test で実行**

```bash
cd ros2_ws
nix develop ../.#ros --command bash -lc 'colcon build --packages-select teleop_ik && colcon test --packages-select teleop_ik --pytest-args test_target_msg.py'
```

期待結果: `test_target_pose_with_input_instantiation` および `test_target_pose_with_input_set_fields` が pass。

- [ ] **Step 3: コミット**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
git add ros2_ws/src/teleop_ik/test/test_target_msg.py
git commit -m "test(teleop_ik): verify TargetPoseWithInput msg is importable"
```

---

## Task 3: スティック系パラメータを yaml に追加

**Files:**
- Modify: `ros2_ws/src/teleop_ik/config/teleop_ik_params.yaml`

- [ ] **Step 1: yaml に追記**

`ros2_ws/src/teleop_ik/config/teleop_ik_params.yaml` を以下に置換:

```yaml
teleop_ik_node:
  ros__parameters:
    # urdf_path is set via launch argument
    end_effector_frame: "gripper"
    position_scale: 1.0
    ik_damping: 1.0e-6
    ik_max_iterations: 100
    ik_tolerance: 1.0e-4
    trajectory_time_from_start: 0.1
    unity_conversion: true
    # Stick-driven wrist integration
    stick_velocity_scale: 1.5       # rad/sec at full stick deflection
    stick_deadzone: 0.1             # input magnitude below this is zero
    stick_max_delta_per_msg: 0.2    # rad; safety cap per message
    stick_fallback_dt: 0.0111       # sec; used when header.stamp is missing/invalid
```

- [ ] **Step 2: yaml の構文チェック (任意)**

```bash
cd ros2_ws
nix develop ../.#ros --command bash -lc 'python3 -c "import yaml; yaml.safe_load(open(\"src/teleop_ik/config/teleop_ik_params.yaml\"))"'
```

期待結果: エラー無しで終了。

- [ ] **Step 3: コミット**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
git add ros2_ws/src/teleop_ik/config/teleop_ik_params.yaml
git commit -m "feat(teleop_ik): add stick integration parameters"
```

---

## Task 4: `coordinate_utils.unity_quaternion_to_pitch_roll` を削除 (TDD 反転)

**Files:**
- Modify: `ros2_ws/src/teleop_ik/test/test_coordinate_utils.py`
- Modify: `ros2_ws/src/teleop_ik/teleop_ik/coordinate_utils.py`

> TDD としては「削除ターゲットが存在しない」ことをテストで担保する代わりに、関連テストを先に削除してから本体を削除する。

- [ ] **Step 1: 既存テストから pitch/roll 関連を削除**

`ros2_ws/src/teleop_ik/test/test_coordinate_utils.py` を以下に置換:

```python
import math

import pytest

from teleop_ik.coordinate_utils import unity_position_to_ros, unity_quaternion_to_ros


def test_unity_position_to_ros_axis_mapping():
    ros = unity_position_to_ros(1.0, 2.0, 3.0, scale=1.0)
    assert ros[0] == pytest.approx(3.0)
    assert ros[1] == pytest.approx(-1.0)
    assert ros[2] == pytest.approx(2.0)


def test_unity_position_to_ros_applies_scale():
    ros = unity_position_to_ros(1.0, 2.0, 3.0, scale=2.0)
    assert ros[0] == pytest.approx(6.0)
    assert ros[1] == pytest.approx(-2.0)
    assert ros[2] == pytest.approx(4.0)


def test_unity_quaternion_to_ros_flips_w_for_handedness():
    ros_q = unity_quaternion_to_ros(0.0, 0.0, 0.0, 1.0)
    assert ros_q[3] == pytest.approx(-1.0)
```

- [ ] **Step 2: テストが pass することを確認 (新テストは通る、削除対象は無い)**

```bash
cd ros2_ws
nix develop ../.#ros --command bash -lc 'colcon build --packages-select teleop_ik && colcon test --packages-select teleop_ik --pytest-args test_coordinate_utils.py'
```

期待結果: pass (3 件)。

- [ ] **Step 3: `coordinate_utils.py` から `unity_quaternion_to_pitch_roll` を削除**

`ros2_ws/src/teleop_ik/teleop_ik/coordinate_utils.py` を以下に置換:

```python
"""Coordinate conversion utilities: Unity (left-hand, Y-up) <-> ROS (right-hand, Z-up)."""

import math

import numpy as np


def unity_position_to_ros(
    x: float, y: float, z: float, scale: float = 1.0
) -> np.ndarray:
    """Convert Unity position (X-right, Y-up, Z-forward) to ROS (X-forward, Y-left, Z-up).

    Mapping:
        ros_x =  unity_z
        ros_y = -unity_x
        ros_z =  unity_y
    """
    return np.array([z * scale, -x * scale, y * scale])


def unity_quaternion_to_ros(
    x: float, y: float, z: float, w: float
) -> np.ndarray:
    """Convert Unity quaternion to ROS quaternion.

    Unity is left-handed Y-up, ROS is right-handed Z-up.
    Apply the same axis remapping as position to the quaternion vector part.

    Returns [qx, qy, qz, qw] in ROS convention.
    """
    # Vector part follows the same axis mapping as position
    ros_qx = z
    ros_qy = -x
    ros_qz = y
    ros_qw = -w  # handedness flip
    return np.array([ros_qx, ros_qy, ros_qz, ros_qw])
```

- [ ] **Step 4: テスト再実行で pass を確認**

```bash
cd ros2_ws
nix develop ../.#ros --command bash -lc 'colcon build --packages-select teleop_ik && colcon test --packages-select teleop_ik --pytest-args test_coordinate_utils.py'
```

期待結果: pass。

- [ ] **Step 5: コミット**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
git add ros2_ws/src/teleop_ik/teleop_ik/coordinate_utils.py \
        ros2_ws/src/teleop_ik/test/test_coordinate_utils.py
git commit -m "refactor(teleop_ik): drop unity_quaternion_to_pitch_roll (replaced by stick integration)"
```

---

## Task 5: スティック積分ロジックの TDD — 失敗するテストを追加

**Files:**
- Modify: `ros2_ws/src/teleop_ik/test/test_ik_node.py`

- [ ] **Step 1: テストファイル末尾に新規テストを追加**

`ros2_ws/src/teleop_ik/test/test_ik_node.py` の末尾に以下を追記:

```python
import time

from teleop_ik.ik_node import TeleopIKNode
from teleop_ik.msg import TargetPoseWithInput  # type: ignore[attr-defined]


def _make_input(stick_x: float = 0.0, stick_y: float = 0.0, dt_sec: float = 0.0):
    """Build a TargetPoseWithInput with the given stick.

    dt_sec==0 → leave header.stamp at zero so fallback is used.
    """
    msg = TargetPoseWithInput()
    msg.pose.orientation.w = 1.0
    msg.stick_x = stick_x
    msg.stick_y = stick_y
    if dt_sec > 0.0:
        msg.header.stamp.sec = int(time.time())
        msg.header.stamp.nanosec = int((dt_sec - int(dt_sec)) * 1e9)
    return msg


def test_wrist_integrates_stick_per_message(ik_node):
    ik_node._active = True
    ik_node._unity_conversion = False
    ik_node._arm_init_pos = np.zeros(3)
    ik_node._unity_anchor_pos = np.zeros(3)
    ik_node._q_solution = ik_node._q_current.copy()
    ik_node._wrist_init_pos = np.array([0.0, 0.0])
    ik_node._integrated_stick = (0.0, 0.0)
    ik_node._last_msg_stamp = None
    # set parameters used by the new path
    ik_node.get_parameter = lambda name: _Parameter(
        {
            "ik_damping": 1e-6,
            "ik_max_iterations": 100,
            "ik_tolerance": 1e-4,
            "position_scale": 1.0,
            "stick_velocity_scale": 1.0,
            "stick_deadzone": 0.0,
            "stick_max_delta_per_msg": 10.0,
            "stick_fallback_dt": 0.1,
        }[name]
    )

    # First message: integrate (1.0, 0.5) at fallback_dt=0.1, scale=1.0
    ik_node._on_target_with_input(_make_input(stick_x=1.0, stick_y=0.5))
    assert ik_node._integrated_stick[0] == pytest.approx(0.1)
    assert ik_node._integrated_stick[1] == pytest.approx(0.05)


def test_wrist_resets_on_session_start(ik_node):
    ik_node._active = True
    ik_node._unity_conversion = False
    ik_node._arm_init_pos = np.zeros(3)
    ik_node._unity_anchor_pos = np.zeros(3)
    ik_node._q_solution = ik_node._q_current.copy()
    ik_node._integrated_stick = (1.23, -0.45)
    ik_node._last_msg_stamp = None
    ik_node.get_parameter = lambda name: _Parameter(
        {
            "ik_damping": 1e-6,
            "ik_max_iterations": 100,
            "ik_tolerance": 1e-4,
            "position_scale": 1.0,
            "stick_velocity_scale": 1.0,
            "stick_deadzone": 0.0,
            "stick_max_delta_per_msg": 10.0,
            "stick_fallback_dt": 0.1,
        }[name]
    )

    # Simulate an active→inactive→active round trip
    msg_active = _make_input(stick_x=0.0, stick_y=0.0, dt_sec=0.0)
    msg_active.data = True
    from std_msgs.msg import Bool

    b = Bool()
    b.data = True
    ik_node._on_active(b)
    assert ik_node._integrated_stick == (0.0, 0.0)
    assert ik_node._last_msg_stamp is None


def test_stick_deadzone_zeros_small_inputs(ik_node):
    ik_node._active = True
    ik_node._unity_conversion = False
    ik_node._arm_init_pos = np.zeros(3)
    ik_node._unity_anchor_pos = np.zeros(3)
    ik_node._q_solution = ik_node._q_current.copy()
    ik_node._wrist_init_pos = np.array([0.0, 0.0])
    ik_node._integrated_stick = (0.0, 0.0)
    ik_node._last_msg_stamp = None
    ik_node.get_parameter = lambda name: _Parameter(
        {
            "ik_damping": 1e-6,
            "ik_max_iterations": 100,
            "ik_tolerance": 1e-4,
            "position_scale": 1.0,
            "stick_velocity_scale": 1.0,
            "stick_deadzone": 0.1,
            "stick_max_delta_per_msg": 10.0,
            "stick_fallback_dt": 0.1,
        }[name]
    )

    # 0.05 < deadzone (0.1) → no integration
    ik_node._on_target_with_input(_make_input(stick_x=0.05, stick_y=-0.05))
    assert ik_node._integrated_stick == (0.0, 0.0)


def test_stick_huge_dt_clamps_to_fallback(ik_node):
    ik_node._active = True
    ik_node._unity_conversion = False
    ik_node._arm_init_pos = np.zeros(3)
    ik_node._unity_anchor_pos = np.zeros(3)
    ik_node._q_solution = ik_node._q_current.copy()
    ik_node._wrist_init_pos = np.array([0.0, 0.0])
    ik_node._integrated_stick = (0.0, 0.0)
    ik_node._last_msg_stamp = None
    ik_node.get_parameter = lambda name: _Parameter(
        {
            "ik_damping": 1e-6,
            "ik_max_iterations": 100,
            "ik_tolerance": 1e-4,
            "position_scale": 1.0,
            "stick_velocity_scale": 1.0,
            "stick_deadzone": 0.0,
            "stick_max_delta_per_msg": 10.0,
            "stick_fallback_dt": 0.1,
        }[name]
    )

    # First message: dt fallback should be used since _last_msg_stamp is None
    ik_node._on_target_with_input(_make_input(stick_x=1.0, stick_y=1.0))
    # With fallback_dt=0.1 and scale=1.0, the deltas are 0.1 each
    assert ik_node._integrated_stick[0] == pytest.approx(0.1)
    assert ik_node._integrated_stick[1] == pytest.approx(0.1)


def test_stick_max_delta_per_msg_clamp(ik_node):
    ik_node._active = True
    ik_node._unity_conversion = False
    ik_node._arm_init_pos = np.zeros(3)
    ik_node._unity_anchor_pos = np.zeros(3)
    ik_node._q_solution = ik_node._q_current.copy()
    ik_node._wrist_init_pos = np.array([0.0, 0.0])
    ik_node._integrated_stick = (0.0, 0.0)
    ik_node._last_msg_stamp = None
    ik_node.get_parameter = lambda name: _Parameter(
        {
            "ik_damping": 1e-6,
            "ik_max_iterations": 100,
            "ik_tolerance": 1e-4,
            "position_scale": 1.0,
            "stick_velocity_scale": 1.0,
            "stick_deadzone": 0.0,
            "stick_max_delta_per_msg": 0.05,  # tight cap
            "stick_fallback_dt": 1.0,           # large dt would otherwise blow past cap
        }[name]
    )

    # Stick=1.0 with dt=1.0 and cap=0.05 → integration must stay within cap
    ik_node._on_target_with_input(_make_input(stick_x=1.0, stick_y=1.0))
    assert ik_node._integrated_stick[0] == pytest.approx(0.05)
    assert ik_node._integrated_stick[1] == pytest.approx(0.05)
```

> 注意: `import time` と `from std_msgs.msg import Bool` は他のタスクでテストファイル冒頭に既出の import と衝突しないよう、ファイル先頭の既存 import ブロックに含めてもよい。重複しても pytest は通るが、lint で警告される可能性がある。その場合は既存 import ブロックの末尾に追加する。

- [ ] **Step 2: テストが失敗することを確認**

```bash
cd ros2_ws
nix develop ../.#ros --command bash -lc 'colcon build --packages-select teleop_ik && colcon test --packages-select teleop_ik --pytest-args test_ik_node.py -k "stick or wrist_resets"'
```

期待結果: 全 5 件が `AttributeError` などで fail(まだ `_integrated_stick` / `_on_target_with_input` が未実装のため)。

- [ ] **Step 3: コミット(Red)**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
git add ros2_ws/src/teleop_ik/test/test_ik_node.py
git commit -m "test(teleop_ik): add failing tests for stick-driven wrist integration"
```

---

## Task 6: スティック積分ロジックを実装 (Green)

**Files:**
- Modify: `ros2_ws/src/teleop_ik/teleop_ik/ik_node.py`

- [ ] **Step 1: import を差し替え**

`ros2_ws/src/teleop_ik/teleop_ik/ik_node.py` 冒頭の import を以下に置換:

```python
"""IK node for VR teleop of the SO-101 arm.

Subscribes to VR controller pose+stick and gripper commands, solves IK using
Pinocchio, and publishes JointTrajectory commands.
"""

import subprocess

from builtin_interfaces.msg import Duration
import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64

from teleop_ik.coordinate_utils import unity_position_to_ros
from teleop_ik.msg import TargetPoseWithInput
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
```

- [ ] **Step 2: パラメータ宣言を追加**

`__init__` の `self.declare_parameter("unity_conversion", True)` の直後に以下を追加:

```python
        # -- Stick integration parameters --
        self.declare_parameter("stick_velocity_scale", 1.5)
        self.declare_parameter("stick_deadzone", 0.1)
        self.declare_parameter("stick_max_delta_per_msg", 0.2)
        self.declare_parameter("stick_fallback_dt", 0.0111)
```

- [ ] **Step 3: 状態変数を追加**

`__init__` の `# -- Session state --` 直下、`self._active = False` の前に以下を追加:

```python
        # -- Stick integration state --
        self._integrated_stick: tuple[float, float] = (0.0, 0.0)
        self._last_msg_stamp = None  # rclpy.time.Time or None
```

- [ ] **Step 4: セッション開始/終了処理を更新**

`_start_session` の中身を以下に置換:

```python
    def _start_session(self) -> None:
        """Start a teleop session: capture current EE position via FK and reset stick state."""
        pin.forwardKinematics(self._model, self._data, self._q_current)
        pin.updateFramePlacements(self._model, self._data)
        self._arm_init_pos = self._data.oMf[self._ee_frame_id].translation.copy()
        self._unity_anchor_pos = None  # Will be set on first target_pose
        self._q_solution = self._q_current.copy()
        self._wrist_init_pos = np.array(
            [
                self._q_current[self._model.joints[jid].idx_q]
                for jid in self._wrist_joint_ids
            ]
        )
        self._integrated_stick = (0.0, 0.0)
        self._last_msg_stamp = None
        self._active = True
        self.get_logger().info(
            f"Session started. EE init pos: {self._arm_init_pos}"
        )
```

- [ ] **Step 5: 購詮とコールバック名を差し替え**

`__init__` 内の購詮ブロックのうち、`PoseStamped` を購詮している箇所を以下に置換:

```python
        self.create_subscription(
            TargetPoseWithInput,
            "/teleop/target",
            self._on_target_with_input,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
```

- [ ] **Step 6: `_on_target_pose` を `_on_target_with_input` に置換**

`_on_target_pose` メソッド全体を以下に置換:

```python
    def _on_target_with_input(self, msg: TargetPoseWithInput) -> None:
        """Receive target pose + stick from Unity and solve IK."""
        if (
            not self._active
            or self._arm_init_pos is None
            or self._q_solution is None
            or self._wrist_init_pos is None
        ):
            return

        scale = self.get_parameter("position_scale").get_parameter_value().double_value

        p = msg.pose.position
        o = msg.pose.orientation  # noqa: F841 (kept for future use; wrist uses stick)

        if self._unity_conversion:
            ros_pos = unity_position_to_ros(p.x, p.y, p.z, scale)
        else:
            ros_pos = np.array([p.x, p.y, p.z]) * scale

        # On first pose, record anchor
        if self._unity_anchor_pos is None:
            self._unity_anchor_pos = ros_pos.copy()
            self.get_logger().info(f"Anchor set: {self._unity_anchor_pos}")
            self._last_msg_stamp = self._stamp_to_time(msg.header.stamp)
            return

        # --- Stick integration ---
        stick_scale = (
            self.get_parameter("stick_velocity_scale")
            .get_parameter_value()
            .double_value
        )
        deadzone = (
            self.get_parameter("stick_deadzone")
            .get_parameter_value()
            .double_value
        )
        max_delta = (
            self.get_parameter("stick_max_delta_per_msg")
            .get_parameter_value()
            .double_value
        )
        fallback_dt = (
            self.get_parameter("stick_fallback_dt")
            .get_parameter_value()
            .double_value
        )

        now = self._stamp_to_time(msg.header.stamp)
        if self._last_msg_stamp is None or now is None:
            delta_t = fallback_dt
        else:
            delta_t = now - self._last_msg_stamp
            if delta_t <= 0.0 or delta_t > 0.5:
                delta_t = fallback_dt
        self._last_msg_stamp = now

        # Compute delta from anchor
        delta = ros_pos - self._unity_anchor_pos
        target_pos = self._arm_init_pos + delta

        vx, vy = self._apply_stick_deadzone(
            float(msg.stick_x), float(msg.stick_y), deadzone
        )
        # Per-message cap on the *resulting* delta
        cap_v = max_delta / max(stick_scale * delta_t, 1e-6)
        vx = float(np.clip(vx, -cap_v, cap_v))
        vy = float(np.clip(vy, -cap_v, cap_v))
        delta_vx = vx * stick_scale * delta_t
        delta_vy = vy * stick_scale * delta_t
        self._integrated_stick = (
            self._integrated_stick[0] + delta_vx,
            self._integrated_stick[1] + delta_vy,
        )

        q_seed = self._q_solution.copy()
        # stick_y → joint 4 (pitch), stick_x → joint 5 (roll)
        q_seed[self._model.joints[self._wrist_joint_ids[0]].idx_q] = (
            self._wrist_init_pos[0] + self._integrated_stick[1]
        )
        q_seed[self._model.joints[self._wrist_joint_ids[1]].idx_q] = (
            self._wrist_init_pos[1] + self._integrated_stick[0]
        )
        q_seed = self._clamp_joints(q_seed)

        q_result = self._solve_ik(target_pos, q_seed)
        if q_result is not None:
            self._q_solution = q_result
            self._publish_arm_trajectory(q_result)
```

- [ ] **Step 7: ヘルパーメソッドを追加**

`_solve_ik` の直後に以下を追加(クラス内に追加するので `_solve_ik` と同じインデント):

```python
    def _apply_stick_deadzone(
        self, x: float, y: float, deadzone: float
    ) -> tuple[float, float]:
        """Apply radial deadzone then rescale to preserve full-range feel."""
        mag = float(np.hypot(x, y))
        if mag < deadzone or deadzone >= 1.0:
            return 0.0, 0.0
        scale = (mag - deadzone) / (mag * (1.0 - deadzone))
        return x * scale, y * scale

    def _stamp_to_time(self, stamp) -> float | None:
        """Convert a builtin_interfaces/Time to a float seconds, or None if invalid."""
        try:
            sec = int(stamp.sec)
            nsec = int(stamp.nanosec)
        except (AttributeError, TypeError, ValueError):
            return None
        if sec == 0 and nsec == 0:
            # Treat uninitialized stamp as invalid → caller falls back.
            return None
        return float(sec) + float(nsec) * 1e-9
```

- [ ] **Step 8: テストを実行して Green を確認**

```bash
cd ros2_ws
nix develop ../.#ros --command bash -lc 'colcon build --packages-select teleop_ik && colcon test --packages-select teleop_ik --pytest-args test_ik_node.py'
```

期待結果: Task 5 で追加した 5 件が pass、既存テストのうち `test_solve_ik_keeps_wrist_joint_targets_fixed` および `test_clamp_joints_*` が変わらず pass。`test_target_pose_uses_previous_successful_solution_as_next_seed` は Task 7 で更新する。

- [ ] **Step 9: コミット (Green)**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
git add ros2_ws/src/teleop_ik/teleop_ik/ik_node.py
git commit -m "feat(teleop_ik): integrate stick input to drive joints 4/5"
```

---

## Task 7: 既存 ik_node テストを新仕様に更新

**Files:**
- Modify: `ros2_ws/src/teleop_ik/test/test_ik_node.py`

- [ ] **Step 1: 既存テスト `test_target_pose_uses_previous_successful_solution_as_next_seed` を更新**

`test_ik_node.py` の `test_target_pose_uses_previous_successful_solution_as_next_seed` 関数を以下に置換:

```python
def test_target_uses_previous_successful_solution_as_next_seed(ik_node):
    ik_node._active = True
    ik_node._unity_conversion = False
    ik_node._arm_init_pos = np.zeros(3)
    ik_node._unity_anchor_pos = np.zeros(3)
    ik_node._q_solution = ik_node._q_current.copy()
    ik_node._wrist_init_pos = np.array([0.1, -0.2])
    ik_node._integrated_stick = (0.0, 0.0)
    ik_node._last_msg_stamp = None
    seeds = []

    def solve(_target_position, seed):
        seeds.append(seed.copy())
        result = seed.copy()
        result[0] += 0.05
        return result

    ik_node._solve_ik = solve
    ik_node._publish_arm_trajectory = lambda _q: None
    ik_node.get_parameter = lambda name: _Parameter(
        {
            "ik_damping": 1e-6,
            "ik_max_iterations": 100,
            "ik_tolerance": 1e-4,
            "position_scale": 1.0,
            "stick_velocity_scale": 0.0,  # disable stick for this test
            "stick_deadzone": 0.0,
            "stick_max_delta_per_msg": 10.0,
            "stick_fallback_dt": 0.0,
        }[name]
    )

    first = PoseStamped()
    first.pose.orientation.w = 1.0
    ik_node._on_target_pose(first)
    # Anchor set, no IK solved this iteration.

    second = PoseStamped()
    second.pose.orientation.w = 1.0
    ik_node._on_target_pose(second)

    # First non-anchor msg: wrist init still used (stick_x=stick_y=0)
    assert seeds[0][3] == pytest.approx(0.1)
    assert seeds[0][4] == pytest.approx(-0.2)
    # Second msg uses the previous solution as seed
    assert seeds[1][0] == pytest.approx(0.05)
    assert seeds[1][3] == pytest.approx(0.1)
    assert seeds[1][4] == pytest.approx(-0.2)
```

- [ ] **Step 2: テスト実行で pass を確認**

```bash
cd ros2_ws
nix develop ../.#ros --command bash -lc 'colcon build --packages-select teleop_ik && colcon test --packages-select teleop_ik --pytest-args test_ik_node.py'
```

期待結果: 全件 pass。

- [ ] **Step 3: コミット**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
git add ros2_ws/src/teleop_ik/test/test_ik_node.py
git commit -m "test(teleop_ik): align previous-solution test with stick-driven wrist"
```

---

## Task 8: `gamepad_node` を新 msg publish に移行

**Files:**
- Modify: `ros2_ws/src/teleop_ik/teleop_ik/gamepad_node.py`

- [ ] **Step 1: import を更新**

`ros2_ws/src/teleop_ik/teleop_ik/gamepad_node.py` 冒頭の import を以下に置換:

```python
"""Gamepad teleop node: converts Joy input to IK target pose via velocity control."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float64

from teleop_ik.msg import TargetPoseWithInput

# PS3 DualShock 3 default mapping (Linux kernel driver)
# Axes: 0=LX, 1=LY, 2=RX, 3=RY
# Buttons: 0=Cross, 1=Circle, 2=Triangle, 3=Square,
#          4=L1, 5=R1, 6=L2, 7=R2, 8=Select, 9=Start
```

- [ ] **Step 2: パラメータと購詮は据え置き、publisher を `TargetPoseWithInput` に変更**

`_pose_pub` の publisher 作成箇所を以下に置換:

```python
        self._target_pub = self.create_publisher(
            TargetPoseWithInput, "/teleop/target", 10
        )
        self._active_pub = self.create_publisher(Bool, "/teleop/active", 10)
        self._gripper_pub = self.create_publisher(Float64, "/teleop/gripper", 10)
```

- [ ] **Step 3: タイマーコールバック内の publish ブロックを置換**

`_timer_callback` 内の `# Publish target pose (ROS frame, relative to session start)` 以降の `pose` 組み立て・publish 部分を以下に置換:

```python
        # Publish target pose + stick (stick_x=stick_y=0 for gamepad; wrist stays neutral)
        msg = TargetPoseWithInput()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = self._target_x
        msg.pose.position.y = self._target_y
        msg.pose.position.z = self._target_z
        msg.pose.orientation.w = 1.0
        msg.stick_x = 0.0
        msg.stick_y = 0.0
        self._target_pub.publish(msg)

        self._gripper_pub.publish(Float64(data=self._gripper_value))
```

- [ ] **Step 4: 静的検証 (`python -m py_compile`)**

```bash
cd ros2_ws
nix develop ../.#ros --command bash -lc 'python3 -m py_compile src/teleop_ik/teleop_ik/gamepad_node.py'
```

期待結果: エラー無しで終了。

- [ ] **Step 5: コミット**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
git add ros2_ws/src/teleop_ik/teleop_ik/gamepad_node.py
git commit -m "feat(teleop_ik): publish TargetPoseWithInput from gamepad_node"
```

---

## Task 9: Unity 用 `TargetPoseWithInput.msg` ミラーファイルを追加

**Files:**
- Create: `SoArmVR/Assets/_SoArmVR/Msgs/teleop_ik/msg/TargetPoseWithInput.msg`

- [ ] **Step 1: ミラーファイルを作成**

`SoArmVR/Assets/_SoArmVR/Msgs/teleop_ik/msg/TargetPoseWithInput.msg`:

```
std_msgs/Header header
geometry_msgs/Pose pose
float32 stick_x
float32 stick_y
```

> ROS 側と完全一致させる(2026-06-13 設計書 §3.2 のミラーパターン)。

- [ ] **Step 2: コミット(まだ生成しない)**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
git add SoArmVR/Assets/_SoArmVR/Msgs/teleop_ik/
git commit -m "feat(soarmvr): add TargetPoseWithInput.msg mirror for genmsg"
```

---

## Task 10: Unity 側 ROSettaDDS メッセージを生成

**Files:**
- Create: `SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/TeleopIk/TargetPoseWithInput.cs`(+ メタ)
- Create: `SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/TeleopIk/TargetPoseWithInputSerializer.cs`(+ メタ)

- [ ] **Step 1: genmsg を実行**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
nix develop .#soarmvr --command bash -lc '\
  dotnet run --project <ROSettaDDS>/tools/rosettadds-genmsg -- \
    --input  SoArmVR/Assets/Msgs \
    --output SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs'
```

> 注: `<ROSettaDDS>` はローカルクローンパス。`SoArmVR/README` 等で運用パスを確認し置換する。

期待結果: `SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/TeleopIk/TargetPoseWithInput.cs` と Serializer が生成される(2026-06-13 設計書 §3.2 と同じパターン)。

- [ ] **Step 2: 生成物の存在確認**

```bash
ls SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/TeleopIk/
```

期待結果: `TargetPoseWithInput.cs` と `TargetPoseWithInputSerializer.cs`(および `.meta`)が並ぶ。

- [ ] **Step 3: Unity コンソールエラーがないことを `uloop` で確認**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
uloop refresh-assemblies
uloop console --tail 200
```

期待結果: 生成型が見つからない旨のエラーや cs コンパイルエラーが無いこと。

- [ ] **Step 4: コミット**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
git add SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/TeleopIk/
git commit -m "feat(soarmvr): generate TargetPoseWithInput C# types from .msg"
```

---

## Task 11: `TeleoperationSample` に `stick` フィールドを追加

**Files:**
- Modify: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationSample.cs`

- [ ] **Step 1: フィールドを追加**

`TeleoperationSample.cs` の構造体内、`public float gripper;` の直後に以下を追加:

```csharp
        /// <summary>右コントローラの primary2DAxis(右親指スティック)の生入力 (-1..1, x/y)。</summary>
        public Vector2 stick;
```

- [ ] **Step 2: コミット**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
git add SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationSample.cs
git commit -m "feat(soarmvr): add stick field to TeleoperationSample"
```

---

## Task 12: `TeleoperationAnchor.Place` をヨー抽出に改修

**Files:**
- Modify: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationAnchor.cs`

- [ ] **Step 1: `Place` メソッドを置換**

`Place` メソッドを以下に置換:

```csharp
        /// <summary>
        /// 指定のワールド位置とワールド回転を基準にアンカーを設置する。
        /// アンカーの up 軸は常にワールド up 軸に一致させ、ヨーのみを保持する
        /// (コントローラの pitch/roll は捨てる)。
        /// </summary>
        public void Place(Vector3 worldPosition, Quaternion worldRotation)
        {
            Vector3 fwd = Vector3.ProjectOnPlane(worldRotation * Vector3.forward, Vector3.up);
            if (fwd.sqrMagnitude < 1e-6f)
            {
                // 真上/真下を向きすぎて水平成分がほぼゼロのときはワールド forward にフォールバック
                fwd = Vector3.ProjectOnPlane(Vector3.forward, Vector3.up);
            }
            transform.SetPositionAndRotation(worldPosition, Quaternion.LookRotation(fwd.normalized, Vector3.up));
            IsPlaced = true;
            SetVisualVisible(true);
        }
```

- [ ] **Step 2: 静的検証 (`uloop` でコンパイルエラー確認)**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
uloop refresh-assemblies
uloop console --tail 200
```

期待結果: コンパイルエラー無し。

- [ ] **Step 3: コミット**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
git add SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationAnchor.cs
git commit -m "feat(soarmvr): project anchor to keep only yaw (up = world up)"
```

---

## Task 13: Input Actions に `Stick` を追加

**Files:**
- Modify: `SoArmVR/Assets/_SoArmVR/Input/SoArmTeleoperation.inputactions`

- [ ] **Step 1: アクション定義を追加**

`SoArmTeleoperation.inputactions` の `Teleoperation` マップの `actions` 配列、`Gripper` の直後に以下を追加:

```json
                {
                    "name": "Stick",
                    "type": "Value",
                    "id": "<新規UUID, ulg --uuid などで生成して差し替え>",
                    "expectedControlType": "Vector2",
                    "processors": "",
                    "interactions": "",
                    "initialStateCheck": true
                }
```

- [ ] **Step 2: バインディングを追加**

同ファイルの `bindings` 配列、`Gripper` の直後に以下を追加:

```json
                {
                    "name": "",
                    "id": "<新規UUID, ulg --uuid などで生成して差し替え>",
                    "path": "<XRController>{RightHand}/primary2DAxis",
                    "interactions": "",
                    "processors": "",
                    "groups": "",
                    "action": "Stick",
                    "isComposite": false,
                    "isPartOfComposite": false
                }
```

- [ ] **Step 3: Unity が inputactions を取り込み直すのを確認**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
uloop refresh-assemblies
uloop console --tail 200
```

期待結果: 取り込みエラー無し。

- [ ] **Step 4: コミット**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
git add SoArmVR/Assets/_SoArmVR/Input/SoArmTeleoperation.inputactions
git commit -m "feat(soarmvr): add Stick (primary2DAxis) action for right controller"
```

---

## Task 14: `TeleoperationSession` でスティックを読み取り sample に詰める

**Files:**
- Modify: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationSession.cs`

- [ ] **Step 1: `[SerializeField]` フィールドを追加**

`TeleoperationSession.cs` の `[Header("Input")]` ブロックの末尾(`_gripperAction` の直後)に以下を追加:

```csharp
        [SerializeField, Tooltip("スティック入力(右親指スティック -1..1)")]
        InputActionProperty _stickAction;
```

- [ ] **Step 2: `Awake/OnEnable/OnDisable` を更新**

`OnEnable` を以下に置換:

```csharp
        void OnEnable()
        {
            _teleoperateAction.action?.Enable();
            _gripperAction.action?.Enable();
            _stickAction.action?.Enable();
        }
```

`OnDisable` を以下に置換:

```csharp
        void OnDisable()
        {
            _teleoperateAction.action?.Disable();
            _gripperAction.action?.Disable();
            _stickAction.action?.Disable();
            if (_active)
                EndSession();
        }
```

- [ ] **Step 3: `PushSample` 内で sample に stick を詰める**

`PushSample` 内の sample 生成箇所に `stick` を追加。`var sample = new TeleoperationSample { ... };` ブロックを以下に置換:

```csharp
            var sample = new TeleoperationSample
            {
                timestampMs = System.DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
                id = _sampleId++,
                position = localPosition,
                rotation = localRotation,
                gripper = _gripperAction.action != null ? _gripperAction.action.ReadValue<float>() : 0f,
                stick = _stickAction.action != null ? _stickAction.action.ReadValue<Vector2>() : Vector2.zero,
            };
```

> 注意: `Vector2` は `UnityEngine` 名前空間。`using UnityEngine;` が既にファイル先頭にあるはず(元コードに記載あり)。

- [ ] **Step 4: コミット**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
git add SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationSession.cs
git commit -m "feat(soarmvr): read right-stick input into TeleoperationSample"
```

---

## Task 15: `RosTeleoperationSink` を新 msg publish 化

**Files:**
- Modify: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/RosTeleoperationSink.cs`

- [ ] **Step 1: using を追加**

`RosTeleoperationSink.cs` 冒頭の using ブロックに以下を追加:

```csharp
using ROSettaDDS.Msgs.TeleopIk;
using RosTargetPoseWithInput = ROSettaDDS.Msgs.TeleopIk.TargetPoseWithInput;
```

> 名前空間 `ROSettaDDS.Msgs.TeleopIk` は Task 10 の生成物(パッケージ名 `teleop_ik` → パスカルケース `TeleopIk`)に合わせる。実際の生成物の namespace を Task 10 完了時に確認し、必要なら修正する。

- [ ] **Step 2: フィールドを `TargetPoseWithInput` publisher に差し替え**

`Publisher<PoseStamped> _posePub;` を以下に置換:

```csharp
        Publisher<RosTargetPoseWithInput> _targetPub;
```

- [ ] **Step 3: `InitParticipant` の publisher 作成を置換**

`_posePub` を作成している 5 引数版の `CreatePublisher<PoseStamped>(...)` ブロックを以下に置換:

```csharp
            _targetPub = _participant.CreatePublisher<RosTargetPoseWithInput>(
                "/teleop/target",
                TargetPoseWithInputSerializer.Instance,
                ReliabilityQos.BestEffort,
                DurabilityQos.Volatile,
                RosTargetPoseWithInput.DdsTypeName);
```

- [ ] **Step 4: `OnDestroy` を更新**

`_posePub?.Dispose();` を `_targetPub?.Dispose();` に置換し、`_posePub = null;` も `_targetPub = null;` に置換。

- [ ] **Step 5: `Push` / `PublishPose` を新 msg publish 化**

`Push` メソッドを以下に置換:

```csharp
        public void Push(in TeleoperationSample sample)
        {
            // async void に in パラメータを直接渡せないため値コピーする
            var s = sample;
            PublishTarget(s);
            PublishGripper(s.gripper);
        }
```

`PublishPose` メソッドを `PublishTarget` にリネームし、中身を以下に置換:

```csharp
        async void PublishTarget(TeleoperationSample sample)
        {
            if (_targetPub == null) return;

            var now = System.DateTimeOffset.UtcNow;
            var stamp = new RosTime((int)now.ToUnixTimeSeconds(), (uint)(now.Millisecond * 1_000_000));

            var msg = new RosTargetPoseWithInput(
                new Header(stamp, "teleop"),
                new RosPose(
                    new Point(sample.position.x, sample.position.y, sample.position.z),
                    new RosQuaternion(sample.rotation.x, sample.rotation.y, sample.rotation.z, sample.rotation.w)
                ),
                sample.stick.x,
                sample.stick.y
            );

            try
            {
                await _targetPub.PublishAsync(msg);
            }
            catch (System.ObjectDisposedException) { }
        }
```

- [ ] **Step 6: コンパイル確認**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
uloop refresh-assemblies
uloop console --tail 200
```

期待結果: `TargetPoseWithInput` 型未解決やシグネチャ不一致のエラーが無いこと。

- [ ] **Step 7: コミット**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
git add SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/RosTeleoperationSink.cs
git commit -m "feat(soarmvr): publish TargetPoseWithInput from RosTeleoperationSink"
```

---

## Task 16: Prefab に `_stickAction` を結線

**Files:**
- Modify: `SoArmVR/Assets/_SoArmVR/Prefabs/Teleoperation.prefab`

> Prefab YAML の手編集はミスが起きやすいので、Step 1 で `uloop` 経由(または手動)で `_stickAction` を参照設定する。CLI 経由が難しい場合は、Step 2 の手編集手順で差し替える。

- [ ] **Step 1: 可能なら `uloop` 経由で設定**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
uloop set-input-action --prefab SoArmVR/Assets/_SoArmVR/Prefabs/Teleoperation.prefab \
  --component TeleoperationSession --field _stickAction --action Stick
```

期待結果: prefab が更新され、`_stickAction` の `m_Reference` に InputAction 参照が入る。

> 注: 上記 CLI が存在しない/別仕様なら次の Step 2 へ。

- [ ] **Step 2: CLI が無い場合の手編集**

1. Unity Editor を開いて `SoArmVR/Assets/_SoArmVR/Prefabs/Teleoperation.prefab` を選択
2. `Teleoperation Session` オブジェクトの `TeleoperationSession` コンポーネントを表示
3. Inspector の `Stick Action` フィールドに、`SoArmTeleoperation` Input Action Asset の `Teleoperation` マップ / `Stick` アクションをドラッグ&ドロップ
4. prefab を保存

- [ ] **Step 3: 変更をコミット**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
git add SoArmVR/Assets/_SoArmVR/Prefabs/Teleoperation.prefab
git commit -m "feat(soarmvr): wire _stickAction in Teleoperation.prefab"
```

---

## Task 17: 最終検証

**Files:** (検証のみ、変更なし)

- [ ] **Step 1: ROS 側フルテスト**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground/ros2_ws
nix develop ../.#ros --command bash -lc 'colcon build --packages-select teleop_ik && colcon test --packages-select teleop_ik'
```

期待結果: 全テスト pass。

- [ ] **Step 2: Unity コンソール確認**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
uloop refresh-assemblies
uloop console --tail 200
```

期待結果: コンパイルエラー・ランタイムエラー無し。

- [ ] **Step 3: 差分確認**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
git status
git log --oneline feat/unity-sticks-pub ^main | head -30
```

期待結果: Task 1〜16 のコミットが並ぶ。意図しない差分が残っていないこと(`Meta Quest 3.asset` の未コミット変更は本変更とは別件なので、必要に応じて別 PR で扱う)。

- [ ] **Step 4: ドキュメント更新 (任意)**

`ros2_ws/README.md` の VR テレオペ launch の説明文を読み、必要なら「`/teleop/target_pose` → `/teleop/target`」「`TargetPoseWithInput` 経由のスティック駆動手首」への言及を更新する。

該当箇所: `ros2_ws/README.md` 126-132 行目周辺。

必要なら差分例:

```diff
- VR テレオペ launch は follower 実機 controller と `teleop_ik_node` をまとめて起動する。
- `/teleop/target_pose` は SoArmVR に合わせて BestEffort で購読し、IK が収束しない場合は
+ VR テレオペ launch は follower 実機 controller と `teleop_ik_node` をまとめて起動する。
+ `/teleop/target` (TargetPoseWithInput) は SoArmVR に合わせて BestEffort で購読し、IK が収束しない場合は
   関節指令を publish しない。IK の各反復と出力値は URDF の関節上下限に clamp される。
- SoArmVR の相対回転はローカル pitch を joint 4、ローカル roll を joint 5 に対応させ、
- yaw は使用しない。joint 4・5 を回転目標に固定した上で、joint 1〜3 のみを位置 IK で解く。
+ アンカーはヨーのみ保持し、up = world up に固定する。手首は右スティックの入力を
+ 速度として ROS 側で積分し、joint 4 (pitch) と joint 5 (roll) を駆動する。
+ joint 4・5 を回転目標に固定した上で、joint 1〜3 のみを位置 IK で解く。
```

変更した場合:

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
git add ros2_ws/README.md
git commit -m "docs(ros2_ws): describe new /teleop/target topic and stick-driven wrist"
```

- [ ] **Step 5: プッシュ & PR 作成**

```bash
cd /home/ojii3/src/github.com/ojii3/so_arm_playground
git push -u origin feat/unity-sticks-pub
gh pr create --base main --head feat/unity-sticks-pub \
  --title "feat(soarmvr/teleop_ik): anchor yaw projection and stick-driven wrist" \
  --body-file - <<'EOF'
## Summary

- VR アンカーをヨーのみ保持(up = world up)する投影に変更
- スティック入力を新規 ROS msg `teleop_ik/msg/TargetPoseWithInput` で `/teleop/target` に publish
- IK 側でスティックを dt 積分して joint 4/5 を駆動
- 旧 `/teleop/target_pose` (PoseStamped) と `unity_quaternion_to_pitch_roll` を廃止
- gamepad_node も新 msg に移行

## Test plan

- [x] `colcon build --packages-select teleop_ik`
- [x] `colcon test --packages-select teleop_ik`
- [x] `uloop refresh-assemblies` 後の Unity コンソールにエラー無し
- [ ] (可能なら実機 LAN 環境) `ros2 topic echo /teleop/target` の疎通確認

## Spec / Plan

- spec: docs/superpowers/specs/2026-06-21-soarmvr-anchor-projection-stick-wrist-design.md
- plan: docs/superpowers/plans/2026-06-21-soarmvr-anchor-projection-stick-wrist.md
EOF
```

---

## Self-Review Checklist

実装着手前にこの計画と spec を突き合わせた自己確認:

- [x] §1 ゴール(3 つの変更) — Task 1〜8, 9〜16 で全てカバー
- [x] §2 確定方針 — 各決定事項が Task に対応
- [x] §3.1 データフロー — Task 6 / 8 / 14 / 15 で実装
- [x] §3.2 コンポーネント責務 — Task 4〜7, 8, 9〜16 に対応
- [x] §4.1 アンカー投影 — Task 12
- [x] §4.2 Input Actions — Task 13 / 14
- [x] §4.3 新規 msg — Task 1 / 2
- [x] §4.4 Unity 側 Sink — Task 9 / 10 / 15
- [x] §4.5 IK ノード — Task 5 / 6
- [x] §4.6 gamepad_node — Task 8
- [x] §4.7 coordinate_utils — Task 4
- [x] §5 Launch / Interop — 影響なし
- [x] §6.1 テスト — Task 2 / 5 / 7
- [x] §6.2 検証チェックリスト — Task 17

タイプ・シグネチャ整合性:
- メソッド名 `_on_target_with_input` は Task 5 / 6 / 7 で一貫
- フィールド `_integrated_stick`, `_last_msg_stamp` は Task 5 / 6 で一貫
- ヘルパー `_apply_stick_deadzone`, `_stamp_to_time` は Task 6 内で完結
- 名前空間 `ROSettaDDS.Msgs.TeleopIk` は Task 10 / 15 で一貫(実装時に Task 10 の生成物 namespace を確認し Task 15 の using を微調整する旨を Task 15 Step 1 の注記に明記済み)
