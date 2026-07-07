# VR Teleop: RViz (mock hardware) モード 設計書

- 日付: 2026-07-07
- 対象: `ros2_ws/src/lerobot_description/`, `ros2_ws/src/teleop_ik/`, `ros2_ws/README.md`
- ブランチ: (未作成, 実装着手時に切る)

## 1. 背景とゴール

現状, SO-101 の VR テレオペ (`vr_teleop.launch.py`) は
`lerobot_controller/so101_follower_controller.launch.py` を
`is_sim:=False` で起動する前提であり, follower 実機
(`feetech_ros2_driver/FeetechHardwareInterface`) が無いと
URDF が展開できず, `controller_manager` も立ち上がらない.

そのため, 「SoArmVR + teleop_ik_node + RViz」だけで
VR テレオペの動作を可視化・確認することができない.
実機がない開発機 (macOS / RoboStack 環境, および実機を
繋いでいない Linux ホスト) で開発・検証する経路が無い.

これに対し, 次を要望する:

- 実機ハードウェア (Feetech) や Gazebo 無しで
  SO-101 の RViz テレオペ環境を起動できること.
- `teleop_ik_node` が publish する JointTrajectory が,
  実機 launch と同じ controller 経路
  (`arm_controller`, `gripper_controller`,
  `joint_state_broadcaster`) を通って
  `/follower/joint_states` まで届くこと.
- 実機 launch の構造 (`vr_teleop.launch.py`,
  `so101_follower_controller.launch.py`,
  `so101_follower_controllers.yaml`) に極力影響を与えないこと.
- IK / 安全ロジック / 既存トピック QoS は変更しない.

ゴール:

- `ros2_control` の mock hardware plugin
  (`mock_components/GenericSystem`) を使った
  SO-101 用 URDF (`so101_mock.urdf.xacro`) を追加.
- `teleop_ik/launch/vr_teleop_rviz.launch.py` を新規追加し,
  上記 mock URDF + 既存 controller 設定 + `teleop_ik_node` +
  `rviz2` を一つの launch で起動する.
- 既存 launch テスト (`test_vr_teleop_launch.py`) には影響しない.
- README に起動コマンドを追記.

## 2. 確定方針

| 項目 | 決定 |
| --- | --- |
| 起動コマンド | `ros2 launch teleop_ik vr_teleop_rviz.launch.py` |
| ハードウェア層 | `mock_components/GenericSystem` 1 個 (System インターフェース) |
| 対象関節 | 1, 2, 3, 4, 5, 6 (アーム 5 + グリッパ 1) |
| state interface | position のみ (velocity は将来必要になったら追加) |
| 既存 URDF の扱い | `so101.urdf.xacro` (Gazebo) と `so101_hw.urdf.xacro` (Feetech) は無改修. 第 3 の URDF として `so101_mock.urdf.xacro` を追加. |
| controller 設定 | 既存の `lerobot_controller/config/so101_follower_controllers.yaml` をそのまま流用. |
| 起動する node | `/follower/robot_state_publisher`, `/follower/controller_manager` (`ros2_control_node`), 3 個の spawner (`joint_state_broadcaster`, `arm_controller`, `gripper_controller`), `teleop_ik_node`, `rviz2` |
| `teleop_ik_node` 側 | 改修なし. 既存 `teleop_ik.launch.py` を IncludeLaunch する. |
| 既存 launch への影響 | `vr_teleop.launch.py` / `so101_follower_controller.launch.py` には手を入れない. |
| launch テスト | `test_vr_teleop_launch.py` は既存挙動を維持. 新 launch 用に `test_vr_teleop_rviz_launch.py` を追加. |

## 3. アーキテクチャ

### 3.1 全体フロー

```text
┌──────────┐  /teleop/*      ┌──────────────┐
│  SoArmVR │ ───────────────►│ teleop_ik_node│
│ (Unity)  │                 │  (C++)        │
└──────────┘                 └───┬──────────┘
                                 │ /follower/arm_controller/joint_trajectory
                                 │ /follower/gripper_controller/joint_trajectory
                                 ▼
                       ┌────────────────────────┐
                       │  /follower/controller_manager  │
                       │   ├ arm_controller     │ (mock hardware の command_interface へ書き込み)
                       │   ├ gripper_controller │
                       │   └ joint_state_broadcaster
                       └────────────┬───────────┘
                                    │ /follower/joint_states
                                    ▼
                       ┌────────────────────────┐
                       │ /follower/robot_state_publisher │
                       └────────────┬───────────┘
                                    │ /tf (TF ツリー)
                                    ▼
                              ┌──────────┐
                              │   RViz   │
                              └──────────┘
```

mock hardware は `arm_controller` / `gripper_controller` から
書き込まれた position コマンドを次周期の state として返すため,
`joint_state_broadcaster` の `/follower/joint_states` が
`robot_state_publisher` 経由で TF に反映され, RViz のモデルが動く.

### 3.2 追加・変更ファイル

#### 3.2.1 `ros2_ws/src/lerobot_description/urdf/so101_ros2_control_mock.xacro` (新規)

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

  <!--
    ros2_control mock hardware for RViz-only teleop testing.
    実機 URDF (so101_ros2_control_feetech.xacro) と同じ関節名・上下限・
    command_interface を取り, hardware plugin だけを mock に差し替える.
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

上下限値は `so101_ros2_control.xacro` および
`so101_ros2_control_feetech.xacro` と完全に一致させる.
joint 名 (数字) も既存と一致させ,
`lerobot_controller/config/so101_follower_controllers.yaml` の
`joints` 指定と矛盾しないようにする.

#### 3.2.2 `ros2_ws/src/lerobot_description/urdf/so101_mock.urdf.xacro` (新規)

```xml
<?xml version="1.0"?>
<robot name="so101" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:include filename="$(find lerobot_description)/urdf/so101_base.xacro" />
  <xacro:include filename="$(find lerobot_description)/urdf/so101_ros2_control_mock.xacro" />

</robot>
```

`so101_base.xacro` のみを include するため,
`so101_gazebo.xacro` (Gazebo プラグイン) は読み込まれない.
mock hardware と Gazebo プラグインの二重登録を避ける.

#### 3.2.3 `ros2_ws/src/lerobot_description/package.xml` (変更)

`mock_components` の実行依存を追加:

```xml
<exec_depend>mock_components</exec_depend>
```

#### 3.2.4 `ros2_ws/src/teleop_ik/launch/vr_teleop_rviz.launch.py` (新規)

- mock URDF を `xacro` で展開.
- 既存 `teleop_ik.launch.py` を IncludeLaunchDescription.
- `controller_manager` 用に `ros2_control_node` を
  `/follower` 名前空間で起動.
- `joint_state_broadcaster`, `arm_controller`,
  `gripper_controller` の 3 個を
  `controller_manager` 経由で spawn.
- `robot_state_publisher` を `/follower` 名前空間で起動.
- `rviz2` を起動.
- 公開 launch 引数:
  - `urdf_path` (default: `$(find lerobot_description)/urdf/so101_mock.urdf.xacro`)
  - `controllers_file` (default: `lerobot_controller` の
    `config/so101_follower_controllers.yaml`)
  - `rviz_config` (default: `teleop_ik` 側に新規追加する
    `config/vr_teleop_rviz.rviz`)
  - `params_file` (default: `teleop_ik/config/teleop_ik_params.yaml`)

シーケンス:

1. `robot_state_publisher` を先に起動.
2. `controller_manager` を起動.
3. 各 spawner を `TimerAction(period=2.0)` で遅延起動し,
   `controller_manager` の立ち上がりを待つ.
4. `teleop_ik_node` を起動.
5. `rviz2` を起動.

#### 3.2.5 `ros2_ws/src/teleop_ik/config/vr_teleop_rviz.rviz` (新規)

- Fixed Frame: `base_link` (URDF のルート)
- `RobotModel` 1 個 (Robot Description: `robot_description`)
- 視点は `lerobot_description/rviz/display.rviz` と同じ
  (-1, -1, 1 付近) を既定とする.
- TF / Marker 系は表示しない (VR 入力の確認用としては不要).

#### 3.2.6 `ros2_ws/src/teleop_ik/test/test_vr_teleop_rviz_launch.py` (新規)

launch ファイル単体の構文テスト:

- `DeclareLaunchArgument` で `urdf_path`, `controllers_file`,
  `rviz_config`, `params_file` が公開されていること.
- `Node` に `ros2_control_node`,
  `robot_state_publisher`, `rviz2`, `teleop_ik_node`
  の各パッケージ/実行ファイルが含まれること.
- `IncludeLaunchDescription` 経由で
  `teleop_ik/launch/teleop_ik.launch.py` が読み込まれること.

#### 3.2.7 `ros2_ws/README.md` (変更)

「代表的なコマンド」セクションに追記:

```bash
# SoArmVR + RViz (実機/Gazebo なしで VR テレオペ動作確認)
ros2 launch teleop_ik vr_teleop_rviz.launch.py
```

## 4. テスト戦略

| 種別 | 内容 |
| --- | --- |
| launch 構文 | `test_vr_teleop_rviz_launch.py` で launch ファイル単体検証 |
| mock URDF 構築 | `xacro` 経由で URDF XML がエラーなく展開できること (`colcon build` 時に CMake のテストフェーズで実行) |
| 統合 | macOS / RoboStack でも `ros2 launch` がエラーなく立ち上がる (起動コマンドのドキュメントとして README に記載) |

実機/実環境での確認は範囲外とする. ただし, 実機 launch
(`vr_teleop.launch.py`) の動作には一切影響を与えない.

## 5. 影響範囲

| 変更対象 | 影響 |
| --- | --- |
| `vr_teleop.launch.py` | 触らない. |
| `so101_follower_controller.launch.py` | 触らない. |
| `so101_follower_controllers.yaml` | 触らない. |
| `teleop_ik_node` (C++) | 触らない. |
| `teleop_ik.launch.py` | 触らない. 新 launch から include される. |
| `SoArmVR` (Unity) | 触らない. |
| `lerobot_description` 既存 URDF | 触らない. 新規 `so101_mock.urdf.xacro` および ros2_control xacro を追加するだけ. |
| `lerobot_description/package.xml` | `mock_components` 依存を追加. |
| `test_vr_teleop_launch.py` | 触らない. |

## 6. 段階リリース計画

実装は次の順に進める:

1. mock 用 ros2_control xacro と URDF を作成.
2. `lerobot_description/package.xml` に依存追加.
3. RViz 設定ファイル (`.rviz`) を作成.
4. `vr_teleop_rviz.launch.py` を実装.
5. launch テストを追加し, `colcon test` を通過させる.
6. README に起動コマンドを追記.
7. `colcon build` + `colcon test` の全パスを確認.

## 7. リスクと対応

| リスク | 対応 |
| --- | --- |
| mock hardware が `controller_manager` と組み合わせたときに spawn できない | `controller_manager` の `update_rate` と controller 設定が既存 YAML と一致しているため, 既存 Gazebo/実機 launch と同じ手順で spawn できるはず. 失敗時は controller_manager のログを見て `TimerAction` の待ち時間を伸ばす. |
| 上下限が Gazebo/Feetech いずれかと食い違う | 上限値は 3 つの ros2_control xacro 間で完全一致させる (本書の §3.2.1). |
| `mock_components` が見つからない | `lerobot_description/package.xml` に `exec_depend: mock_components` を追加する. 配布側で未パッケージの場合は Nix shell の追加検討. |
| 起動順序で `controller_manager` より先に `teleop_ik_node` が動く | `teleop_ik_node` は単に `JointTrajectory` を publish するだけで, 購読する相手が無くても動作する. ただし trajectory が無消費で捨てられ, RViz が動かない. 仕様どおり (`TimerAction` で順序を保証). |
| 既存 launch (`vr_teleop.launch.py`) への影響 | 新 launch は別ファイルで追加し, 既存 launch には一切手を入れない. テストも既存と新規で分離. |

## 8. 関連ドキュメント

- [`2026-06-13-vr-teleop-safety-design.md`](2026-06-13-vr-teleop-safety-design.md) … IK 安全ロジック
- [`2026-07-01-vr-reset-command-design.md`](2026-07-01-vr-reset-command-design.md) … リセット経路
- [`2026-06-23-teleop-ik-cpp-rewrite-design.md`](2026-06-23-teleop-ik-cpp-rewrite-design.md) … IK ノード C++ 化
- [`ros2_ws/src/lerobot_description/urdf/so101_ros2_control.xacro`](../../ros2_ws/src/lerobot_description/urdf/so101_ros2_control.xacro) … Gazebo 用 ros2_control
- [`ros2_ws/src/lerobot_description/urdf/so101_ros2_control_feetech.xacro`](../../ros2_ws/src/lerobot_description/urdf/so101_ros2_control_feetech.xacro) … 実機用 ros2_control
- [`ros2_ws/src/lerobot_controller/config/so101_follower_controllers.yaml`](../../ros2_ws/src/lerobot_controller/config/so101_follower_controllers.yaml) … controller 設定
- [`ros2_ws/src/teleop_ik/launch/vr_teleop.launch.py`](../../ros2_ws/src/teleop_ik/launch/vr_teleop.launch.py) … 既存実機 VR テレオペ launch
