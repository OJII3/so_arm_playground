# VR Teleop RViz (mock hardware) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** SO-101 の VR テレオペ (`teleop_ik_node` + `SoArmVR`) を, 実機 Feetech driver / Gazebo 無しで RViz 上に可視化できる launch 経路を追加する. 実機 launch (`vr_teleop.launch.py`) の構造は変えず, mock hardware を使った RViz 専用 launch を `teleop_ik` パッケージに新設する.

**Architecture:** `ros2_control` の `mock_components/GenericSystem` を hardware layer として使う `so101_mock.urdf.xacro` を `lerobot_description` に追加し, 既存 `so101_follower_controllers.yaml` をそのまま使い回す. 新 launch `vr_teleop_rviz.launch.py` が `robot_state_publisher` + `ros2_control_node` + 3 つの controller spawner + `teleop_ik_node` (既存 launch を include) + `rviz2` をまとめて起動する. `teleop_ik_node` 本体 / 既存 `vr_teleop.launch.py` / 既存 URDF (Gazebo / Feetech) には一切手を入れない.

**Tech Stack:** ROS 2 Jazzy, `ros2_control`, `mock_components`, `controller_manager`, `joint_trajectory_controller`, `joint_state_broadcaster`, `xacro`, `rviz2`.

---

## File Structure

この計画で作成・変更するファイル:

- 新規: `ros2_ws/src/lerobot_description/urdf/so101_ros2_control_mock.xacro`
- 新規: `ros2_ws/src/lerobot_description/urdf/so101_mock.urdf.xacro`
- 変更: `ros2_ws/src/lerobot_description/package.xml` (mock_components 依存追加)
- 新規: `ros2_ws/src/teleop_ik/config/vr_teleop_rviz.rviz`
- 新規: `ros2_ws/src/lerobot_description/rviz/vr_teleop_rviz.rviz`  ← 上の代わりに description 側を使う
- 新規: `ros2_ws/src/teleop_ik/launch/vr_teleop_rviz.launch.py`
- 新規: `ros2_ws/src/teleop_ik/test/test_vr_teleop_rviz_launch.py`
- 変更: `ros2_ws/README.md` (起動コマンド追記)
- 変更: `README.md` (必要に応じて代表コマンド更新)

タスク 1〜3 は description パッケージへの追加, タスク 4〜5 は teleop_ik パッケージへの追加, タスク 6 でドキュメント更新.

---

## Task 1: mock 用 ros2_control xacro を追加

**Files:**
- Create: `ros2_ws/src/lerobot_description/urdf/so101_ros2_control_mock.xacro`

- [ ] **Step 1: ファイル作成**

`ros2_ws/src/lerobot_description/urdf/so101_ros2_control_mock.xacro` を以下の内容で作成する:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

  <!--
    ros2_control mock hardware for RViz-only VR teleop testing.
    Mirrors so101_ros2_control.xacro (Gazebo) and
    so101_ros2_control_feetech.xacro (real hardware) for joint names,
    command interfaces, and joint position limits, but uses
    mock_components/GenericSystem so the controller pipeline can run
    without Gazebo or Feetech servos.
  -->
  <ros2_control name="RobotSystem" type="system">
    <hardware>
      <plugin>mock_components/GenericSystem</plugin>
      <param name="mock_sensor_commands">false</param>
    </hardware>

    <joint name="1">
      <command_interface name="position">
        <param name="min">-1.91986</param>
        <param name="max">1.91986</param>
      </command_interface>
      <state_interface name="position" />
    </joint>
    <joint name="2">
      <command_interface name="position">
        <param name="min">-1.74533</param>
        <param name="max">1.74533</param>
      </command_interface>
      <state_interface name="position" />
    </joint>
    <joint name="3">
      <command_interface name="position">
        <param name="min">-1.74533</param>
        <param name="max">1.5708</param>
      </command_interface>
      <state_interface name="position" />
    </joint>
    <joint name="4">
      <command_interface name="position">
        <param name="min">-1.65806</param>
        <param name="max">1.65806</param>
      </command_interface>
      <state_interface name="position" />
    </joint>
    <joint name="5">
      <command_interface name="position">
        <param name="min">-2.79253</param>
        <param name="max">2.79253</param>
      </command_interface>
      <state_interface name="position" />
    </joint>
    <joint name="6">
      <command_interface name="position">
        <param name="min">-0.174533</param>
        <param name="max">1.74533</param>
      </command_interface>
      <state_interface name="position" />
    </joint>
  </ros2_control>
</robot>
```

- [ ] **Step 2: 差分確認**

`git diff` で `so101_ros2_control.xacro` の min/max 値と完全一致しているか目視確認. 値が 1 か所でも違ったら修正.

- [ ] **Step 3: コミット**

```bash
git add ros2_ws/src/lerobot_description/urdf/so101_ros2_control_mock.xacro
git commit -m "feat(lerobot_description): add mock-hardware ros2_control xacro"
```

---

## Task 2: mock 用 URDF トップレベル xacro を追加

**Files:**
- Create: `ros2_ws/src/lerobot_description/urdf/so101_mock.urdf.xacro`

- [ ] **Step 1: ファイル作成**

`ros2_ws/src/lerobot_description/urdf/so101_mock.urdf.xacro` を以下の内容で作成する:

```xml
<?xml version="1.0"?>
<robot name="so101" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base links/joints -->
  <xacro:include filename="$(find lerobot_description)/urdf/so101_base.xacro" />

  <!-- Use mock ros2_control (no Gazebo, no Feetech hardware) -->
  <xacro:include filename="$(find lerobot_description)/urdf/so101_ros2_control_mock.xacro" />

</robot>
```

- [ ] **Step 2: xacro 展開テスト (手動, dev shell 上で)**

Nix dev shell 内で `ros2` が使える環境で:

```bash
cd ros2_ws
source install/setup.bash
xacro src/lerobot_description/urdf/so101_mock.urdf.xacro > /tmp/so101_mock.urdf
```

`/tmp/so101_mock.urdf` の先頭付近に `<ros2_control ...>` が含まれ, `<plugin>mock_components/GenericSystem</plugin>` が含まれること, `gz_ros2_control` への参照が含まれないことを確認.

- [ ] **Step 3: コミット**

```bash
git add ros2_ws/src/lerobot_description/urdf/so101_mock.urdf.xacro
git commit -m "feat(lerobot_description): add mock-hardware SO-101 URDF"
```

---

## Task 3: lerobot_description の package.xml に mock_components 依存を追加

**Files:**
- Modify: `ros2_ws/src/lerobot_description/package.xml`

- [ ] **Step 1: 依存追加**

`ros2_ws/src/lerobot_description/package.xml` の `</package>` 直前にある `<exec_depend>` 群の末尾に下記 1 行を追記する (`gz_ros2_control` の `<exec_depend>` の直後あたり):

```xml
  <exec_depend>mock_components</exec_depend>
```

挿入後の該当ブロックは次のようになる:

```xml
  <exec_depend>gz_ros2_control</exec_depend>
  <exec_depend>mock_components</exec_depend>
  <exec_depend>ros2launch</exec_depend>
```

- [ ] **Step 2: コミット**

```bash
git add ros2_ws/src/lerobot_description/package.xml
git commit -m "build(lerobot_description): depend on mock_components"
```

---

## Task 4: RViz 設定ファイルを作成

**Files:**
- Create: `ros2_ws/src/lerobot_description/rviz/vr_teleop_rviz.rviz`

- [ ] **Step 1: ベース RViz 設定のコピー元を確認**

既存 `ros2_ws/src/lerobot_description/rviz/display.rviz` を雛形にする. Fixed Frame が `base_link` で `RobotModel` が 1 個入っており, `so101.urdf.xacro` と同じルートリンクを使っている.

- [ ] **Step 2: 新規 RViz 設定ファイル作成**

`ros2_ws/src/lerobot_description/rviz/vr_teleop_rviz.rviz` を以下の内容で作成する (`display.rviz` と同等の Panels + Visualization Manager 構成):

```yaml
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /RobotModel1
      Splitter Ratio: 0.5
    Tree Height: 542
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Goal Pose1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: rviz_common/Time
    Experimental: false
    Name: Time
    SyncMode: 0
    SyncSource: ""
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/RobotModel
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /follower/robot_description
      Enabled: true
      Name: RobotModel
      Visual Enabled: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Default Light: true
    Fixed Frame: base_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 1.0
      Name: Current View
      Pitch: 0.7853981633974483
      Target Frame: <Fixed Frame>
      Yaw: 0.7853981633974483
Window Geometry:
  Height: 800
  Width: 1200
```

ポイント: `Fixed Frame: base_link`, `RobotModel` の Description Topic は `/follower/robot_description` (本 launch の `robot_state_publisher` は `/follower` 名前空間で起動するため).

- [ ] **Step 3: コミット**

```bash
git add ros2_ws/src/lerobot_description/rviz/vr_teleop_rviz.rviz
git commit -m "feat(lerobot_description): add RViz config for mock VR teleop"
```

---

## Task 5: vr_teleop_rviz.launch.py を作成

**Files:**
- Create: `ros2_ws/src/teleop_ik/launch/vr_teleop_rviz.launch.py`

- [ ] **Step 1: launch ファイル作成**

`ros2_ws/src/teleop_ik/launch/vr_teleop_rviz.launch.py` を以下の内容で作成する:

```python
"""Launch the follower controller chain in mock-hardware mode plus teleop_ik + RViz.

No real Feetech servos and no Gazebo required. The SO-101 URDF is
loaded with mock_components/GenericSystem so controller_manager,
arm_controller, gripper_controller, and joint_state_broadcaster can
all be spawned against the same yaml used by the real hardware launch.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lerobot_description_dir = get_package_share_directory("lerobot_description")
    lerobot_controller_dir = get_package_share_directory("lerobot_controller")
    teleop_ik_dir = get_package_share_directory("teleop_ik")

    default_urdf = os.path.join(
        lerobot_description_dir, "urdf", "so101_mock.urdf.xacro"
    )
    default_controllers = os.path.join(
        lerobot_controller_dir, "config", "so101_follower_controllers.yaml"
    )
    default_rviz = os.path.join(
        lerobot_description_dir, "rviz", "vr_teleop_rviz.rviz"
    )
    default_params = os.path.join(
        teleop_ik_dir, "config", "teleop_ik_params.yaml"
    )

    urdf_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value=default_urdf,
        description="Path to SO-101 mock URDF/xacro file",
    )
    controllers_arg = DeclareLaunchArgument(
        "controllers_file",
        default_value=default_controllers,
        description="Path to follower controllers YAML",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz,
        description="Path to RViz configuration file",
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to teleop_ik parameter YAML",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("urdf_path")]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="follower",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="follower",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": False},
            LaunchConfiguration("controllers_file"),
        ],
        output="screen",
    )

    def _spawner(controller_name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                controller_name,
                "--controller-manager",
                "/follower/controller_manager",
            ],
        )

    spawners = TimerAction(
        period=2.0,
        actions=[
            _spawner("joint_state_broadcaster"),
            _spawner("arm_controller"),
            _spawner("gripper_controller"),
        ],
    )

    ik_launch = os.path.join(teleop_ik_dir, "launch", "teleop_ik.launch.py")
    teleop_ik = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ik_launch),
        launch_arguments={
            "params_file": LaunchConfiguration("params_file"),
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
    )

    return LaunchDescription(
        [
            urdf_arg,
            controllers_arg,
            rviz_arg,
            params_arg,
            robot_state_publisher_node,
            controller_manager_node,
            spawners,
            teleop_ik,
            rviz_node,
        ]
    )
```

- [ ] **Step 2: 構文チェック (Python)**

```bash
cd ros2_ws
source install/setup.bash
python3 -c "import ast, pathlib; ast.parse(pathlib.Path('src/teleop_ik/launch/vr_teleop_rviz.launch.py').read_text())"
```

エラーなく終了すること. 期待出力: (なし).

- [ ] **Step 3: コミット**

```bash
git add ros2_ws/src/teleop_ik/launch/vr_teleop_rviz.launch.py
git commit -m "feat(teleop_ik): add VR teleop RViz (mock hardware) launch"
```

---

## Task 6: launch テストを追加

**Files:**
- Create: `ros2_ws/src/teleop_ik/test/test_vr_teleop_rviz_launch.py`

- [ ] **Step 1: テストファイル作成**

`ros2_ws/src/teleop_ik/test/test_vr_teleop_rviz_launch.py` を以下の内容で作成する (既存 `test_vr_teleop_launch.py` と同じスタイル):

```python
import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node


LAUNCH_PATH = (
    Path(__file__).parents[1] / "launch" / "vr_teleop_rviz.launch.py"
)


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("vr_teleop_rviz_launch", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _flatten(entities):
    for entity in entities:
        if isinstance(entity, TimerAction):
            yield from _flatten(entity.actions)
        else:
            yield entity


def test_vr_teleop_rviz_launch_exposes_arguments():
    launch_description = _load_launch_module().generate_launch_description()
    argument_names = {
        entity.name
        for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }

    assert {
        "urdf_path",
        "controllers_file",
        "rviz_config",
        "params_file",
    } <= argument_names


def test_vr_teleop_rviz_launch_spawns_required_nodes():
    launch_description = _load_launch_module().generate_launch_description()
    nodes = [
        entity
        for entity in _flatten(launch_description.entities)
        if isinstance(entity, Node)
    ]
    node_kinds = {(n.package, n.executable) for n in nodes}

    assert ("robot_state_publisher", "robot_state_publisher") in node_kinds
    assert ("controller_manager", "ros2_control_node") in node_kinds
    assert ("rviz2", "rviz2") in node_kinds
    assert ("teleop_ik", "teleop_ik_node") in node_kinds


def test_vr_teleop_rviz_launch_includes_teleop_ik_launch():
    launch_description = _load_launch_module().generate_launch_description()
    includes = [
        entity
        for entity in launch_description.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]

    assert any(
        "teleop_ik.launch.py" in entity.launch_description_source.location
        for entity in includes
    )


def test_vr_teleop_rviz_launch_namespaces_follower():
    launch_description = _load_launch_module().generate_launch_description()
    nodes = [
        entity
        for entity in _flatten(launch_description.entities)
        if isinstance(entity, Node)
    ]

    assert all(node.namespace == "follower" for node in nodes if node.package in {
        "robot_state_publisher",
        "controller_manager",
    })
```

ポイント:

- `_flatten` で `TimerAction` の中の Node を取り出す.
- `controller_manager` の `spawner` は `Node` として表現されるが, `ros2_control_node` とは `executable` が異なる (`spawner` vs `ros2_control_node`). テストでは `ros2_control_node` のみを必須としている.
- `teleop_ik_node` は `IncludeLaunchDescription` 経由で現れるので, 厳密には `nodes` 集合には入らない. テスト 3 で `IncludeLaunchDescription` の存在を確認する.
- `robot_state_publisher` / `controller_manager` (`ros2_control_node`) は `namespace == "follower"` であることを確認する.

- [ ] **Step 2: テスト手動実行**

Nix dev shell + `colcon build` 済みの環境で:

```bash
cd ros2_ws
source install/setup.bash
python3 -m pytest -q src/teleop_ik/test/test_vr_teleop_rviz_launch.py
```

期待出力: `4 passed` (4 件すべて pass).

- [ ] **Step 3: 既存 launch テストが引き続き通ることを確認**

```bash
cd ros2_ws
source install/setup.bash
python3 -m pytest -q src/teleop_ik/test/test_vr_teleop_launch.py
```

期待出力: `2 passed` (既存 2 件が変わらず通る).

- [ ] **Step 4: コミット**

```bash
git add ros2_ws/src/teleop_ik/test/test_vr_teleop_rviz_launch.py
git commit -m "test(teleop_ik): cover VR teleop RViz launch"
```

---

## Task 7: README に起動コマンドを追記

**Files:**
- Modify: `ros2_ws/README.md`
- Modify: `README.md` (必要に応じて)

- [ ] **Step 1: ros2_ws/README.md の「代表的なコマンド」セクションに追記**

`ros2_ws/README.md` の「代表的なコマンド」セクション (「`# MoveIt 2`」の前あたり) に次のブロックを追記する:

```markdown
# SoArmVR + RViz (実機/Gazebo なしで VR テレオペ動作確認)
ros2 launch teleop_ik vr_teleop_rviz.launch.py
```

挿入後, 「代表的なコマンド」セクションは次の流れになる:

```bash
# シミュレーション表示 (rviz)
ros2 launch lerobot_description so101_display.launch.py
# Gazebo シミュレーション
ros2 launch lerobot_description so101_gazebo.launch.py

# フォロワー実機 + コントローラ (is_sim:=True でシミュレーション)
ros2 launch lerobot_controller so101_follower_controller.launch.py \
  is_sim:=False usb_port:=/dev/ttyACM0
# リーダー・フォロワー (teleoperation)
ros2 launch lerobot_controller so101_leader_follower.launch.py

# SoArmVR・フォロワー実機 (VR teleoperation)
ros2 launch teleop_ik vr_teleop.launch.py usb_port:=/dev/ttyACM0

# SoArmVR + RViz (実機/Gazebo なしで VR テレオペ動作確認)
ros2 launch teleop_ik vr_teleop_rviz.launch.py

# MoveIt 2
ros2 launch lerobot_moveit so101_moveit.launch.py is_sim:=False
```

- [ ] **Step 2: ルート README.md の構成説明を必要に応じて更新**

`README.md` の「構成」セクションの表現で, 「SoArmVR + 実機 or 実機なしで RViz」と分けて書かれている場合, 新 launch の存在を 1 行で触れる. 書かれていない場合 (`ros2_ws/README.md` への参照のみ) は更新不要.

該当行が既に存在し, 「SoArmVR・フォロワー実機 (VR teleoperation) は `ros2_ws/README.md` 参照」と書かれている場合は, 「SoArmVR + RViz (実機なし) も同じく `ros2_ws/README.md` 参照」と 1 行追記する.

- [ ] **Step 3: 差分確認**

```bash
git diff ros2_ws/README.md README.md
```

期待: それぞれ新しい launch コマンド 1 ブロック (ros2_ws 側) と 1 行以内の追記 (ルート側) のみ.

- [ ] **Step 4: コミット**

```bash
git add ros2_ws/README.md README.md
git commit -m "docs: document VR teleop RViz (mock hardware) launch"
```

---

## Task 8: 最終ビルド・テスト確認

**Files:** (なし, 確認のみ)

- [ ] **Step 1: colcon build**

```bash
nix develop .#ros           # ルートで
cd ros2_ws
colcon build --symlink-install
```

期待出力: パッケージ `lerobot_description`, `teleop_ik` ともにエラーなくビルド. 警告は許容.

- [ ] **Step 2: launch テスト実行**

```bash
cd ros2_ws
source install/setup.bash
python3 -m pytest -q \
  src/teleop_ik/test/test_vr_teleop_launch.py \
  src/teleop_ik/test/test_vr_teleop_rviz_launch.py
```

期待出力: `6 passed` (既存 2 + 新規 4).

- [ ] **Step 3: xacro 展開確認**

```bash
cd ros2_ws
source install/setup.bash
xacro src/lerobot_description/urdf/so101_mock.urdf.xacro > /tmp/so101_mock.urdf
grep -c mock_components/GenericSystem /tmp/so101_mock.urdf
grep -c gz_ros2_control /tmp/so101_mock.urdf
```

期待出力: 1 行目 `1`, 2 行目 `0` (mock のみ入り, Gazebo プラグインは入らない).

- [ ] **Step 4: 結果まとめ**

全ステップ pass なら, ルートで:

```bash
git log --oneline -10
```

直前のコミットが「docs: document VR teleop RViz (mock hardware) launch」であることを確認. もし途中で失敗したら, 該当タスクに戻って修正.

- [ ] **Step 5: PR 作成 (任意)**

`AGENTS.md` の方針に従い, 完了後に PR を作成する. PR タイトル案:

```
feat: VR teleop in RViz via mock ros2_control hardware
```

本文には次のチェックリストを含める:

- [ ] `colcon build` 通過
- [ ] `test_vr_teleop_launch.py` 2 件 pass
- [ ] `test_vr_teleop_rviz_launch.py` 4 件 pass
- [ ] xacro 展開で `mock_components/GenericSystem` 1 個, `gz_ros2_control` 0 個
- [ ] README の起動コマンド追記

---

## Notes

- `mock_components` は ROS 2 Jazzy の `ros-jazzy-mock-components` (Debian) / `ros-humble-mock-components` 等, ディストリに対応するパッケージが入る. ディストリ依存なので Nix shell (`nix-ros-overlay`) 側の追加が必要であれば別途タスク化する (本計画では未対応, 必要なら `flake.nix` 更新を後続タスクにする).
- `so101_follower_controllers.yaml` の `update_rate: 10` (Hz) は mock hardware でも実機と同じ値を使う. `mock_components/GenericSystem` は既定で実機と同じ周期で command → state を反映するため, そのまま流用して問題ない.
- 既存 `test_vr_teleop_launch.py` は CMake に登録されていない. `pytest` で手動実行する運用が既存スタイル. 今回の新規テストも同じ運用 (Task 6 Step 2).
