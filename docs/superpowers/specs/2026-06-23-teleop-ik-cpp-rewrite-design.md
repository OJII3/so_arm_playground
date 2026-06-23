# teleop_ik: Python から C++ への書き換え 設計書

- 日付: 2026-06-23
- 対象: `ros2_ws/src/teleop_ik/`
- ブランチ: `feat/teleop-ik-cpp`

## 1. 背景とゴール

`ros2_ws/src/teleop_ik/` は SoArmVR (Unity) と SO-101 follower を橋渡しする
ROS 2 パッケージで, VR テレオペレーションの中心ロジックを担う. 現状は
Pure Python 実装 (`rclpy` + `pinocchio`) だが, 以下の理由から C++ へ
移植する.

- VR テレオペは 50〜90 Hz の入力ストリームと 1ms オーダーの
  スティック dt 計算が要る. C++ にすることでジッタと CPU 負荷を
  同時に下げられる.
- 同ワークスペースの C++ パッケージ (`feetech_ros2_driver`,
  `lerobot_controller`) と実装言語が揃い, ビルドシステム・lint
  ・テスト戦略を一貫させられる.
- Python 3 / `rclpy` のリリース系 (`jazzy` の Python ABI) への
  依存を 1 つ減らせる.
- ヘッダ・メッセージ生成 (rosidl) は C++/Python 共通なので, ROS
  インターフェース契約は維持できる.

ゴール:

- `teleop_ik_node` と `gamepad_teleop_node` の 2 ノードを C++
  (`rclcpp`) で再実装し, Python 版を完全削除する.
- ノード名・トピック名・パラメータ・launch ファイル・YAML 設定
  は現行と完全互換を保ち, 既存の launch (`teleop_ik.launch.py`
  `gamepad_teleop.launch.py` `vr_teleop.launch.py`) と YAML
  (`teleop_ik_params.yaml`, `gamepad_params.yaml`) は無変更で
  動作する.
- テストを gtest (`ament_cmake_gtest`) に全面移植する.

## 2. 確定方針

| 項目 | 決定 |
| --- | --- |
| 実装言語 | C++17 (`CMAKE_CXX_STANDARD 17` は既存パッケージと統一, 17 を採用) |
| ROS 2 クライアント | `rclcpp` |
| 線形代数 | `Eigen3` (`Vector3d`, `VectorXd`) |
| IK ライブラリ | `pinocchio` C++ (nixpkgs `pkgs.pinocchio`, 4.0.0) |
| xacro 処理 | `xacro` CLI を `popen` で起動 (Python 版のフォールバックと同一) |
| メッセージ | 既存 `teleop_ik/msg/TargetPoseWithInput.msg` をそのまま使用 |
| Python 版 | 削除 (リポジトリに残さない) |
| ビルドシステム | `ament_cmake` (Python セットアップと console script 廃止) |
| テスト | gtest (`ament_cmake_gtest`) に全面書き換え |
| launch ファイル | そのまま (Python launch は無変更) |
| YAML 設定 | そのまま |

> 17 を選んだ理由: `feetech_ros2_driver` / `lerobot_controller` が C++20
> だが, `pinocchio` 4.0.0 自身の要求は C++14 以上で, 既存パッケージと
> 揃える C++20 でも問題ない. 今回は 17 を採用し, 必要に応じて 20 へ
> 引き上げる. 実装着手時に隣接パッケージに合わせる.

## 3. アーキテクチャ

### 3.1 ターゲット構成

```
teleop_ik/
├── CMakeLists.txt                          # ament_cmake (C++)
├── package.xml                             # C++ 依存に更新
├── msg/TargetPoseWithInput.msg             # そのまま
├── include/teleop_ik/
│   ├── coordinate_utils.hpp                # 新規
│   └── visibility_control.hpp              # ament 慣例
├── src/
│   ├── coordinate_utils.cpp                # lib ターゲット
│   ├── ik_node.cpp                         # 実行ファイル
│   └── gamepad_node.cpp                    # 実行ファイル
├── launch/                                 # そのまま
├── config/                                 # そのまま
└── test/
    ├── test_coordinate_utils.cpp           # 新規 (gtest)
    ├── test_ik_node.cpp                    # 新規 (gtest)
    ├── test_qos.cpp                        # 新規 (gtest + rclcpp)
    ├── test_target_msg.cpp                 # 新規 (gtest)
    ├── test_packaging.cpp                  # 新規 (gtest)
    └── test_vr_teleop_launch.py            # 残す (launch ファイルは Python のため)
```

### 3.2 CMake ターゲット

| ターゲット | 種類 | ソース | 役割 |
| --- | --- | --- | --- |
| `teleop_ik_core` | STATIC/SHARED lib | `src/coordinate_utils.cpp` | 純粋関数の座標変換. テスト対象 |
| `teleop_ik_node` | executable | `src/ik_node.cpp` | VR / gamepad からの target pose を IK 経由で JointTrajectory にして publish |
| `gamepad_teleop_node` | executable | `src/gamepad_node.cpp` | Joy 入力を TargetPoseWithInput + gripper に変換して publish |

インストール先:

- `teleop_ik_node` → `lib/${PROJECT_NAME}/teleop_ik_node`
- `gamepad_teleop_node` → `lib/${PROJECT_NAME}/gamepad_teleop_node`
- ヘッダ → `include/${PROJECT_NAME}/`

### 3.3 コンポーネント責務

| コンポーネント | 責務 |
| --- | --- |
| `coordinate_utils.hpp/cpp` | Unity ↔ ROS の座標変換 (位置・quaternion) |
| `ik_node` | パラメータ宣言, URDF/xacro 読み込み, Pinocchio モデル構築, セッション開始/停止, joint_states 反映, スティック積分, position IK (CLIK), JointTrajectory publish |
| `gamepad_node` | パラメータ宣言, Joy 入力受信, タイマ駆動で位置・gripper を目標 publish, アクティブ信号のトグル publish |

### 3.4 データフロー (ik_node)

```
[外部: Unity / gamepad]
  │
  ▼
/teleop/active         (Bool, RELIABLE)
/teleop/target         (TargetPoseWithInput, BEST_EFFORT)
/teleop/gripper        (Float64, RELIABLE)
/follower/joint_states (JointState, RELIABLE)
  │
  ▼
[TeleopIKNode]
  ├─ _on_active(Bool)
  │     └─ セッション開始/停止. _active フラグ・arm_init_pos・q_solution・integrated_stick をリセット
  ├─ _on_target_with_input(TargetPoseWithInput)
  │     ├─ Unity→ROS 座標変換 (unity_conversion パラメータで切替)
  │     ├─ 初回: unity_anchor_pos を記録して return
  │     ├─ 以降: delta = ros_pos - unity_anchor_pos
  │     ├─ スティック deadzone + dt 積分 (j4 ← stick_y, j5 ← stick_x)
  │     ├─ q_seed = q_solution をコピーし, 手首 j4/j5 を上書き
  │     └─ _solve_ik(target_pos, q_seed) → publish arm trajectory
  ├─ _on_gripper(Float64)
  │     └─ [0, 1] → [GRIPPER_LOWER, GRIPPER_UPPER] に写像して publish
  └─ _on_joint_states(JointState)
        └─ q_current を更新
  │
  ▼
/follower/arm_controller/joint_trajectory    (JointTrajectory)
/follower/gripper_controller/joint_trajectory (JointTrajectory)
```

### 3.5 データフロー (gamepad_node)

```
/joy (sensor_msgs/Joy)
  │
  ▼
[GamepadTeleopNode]
  ├─ _on_joy(Joy)        … 最新の Joy を保持
  └─ _timer_callback()   … 50 Hz 周期
        ├─ Cross ボタンの rising edge で _active をトグルし /teleop/active を publish
        ├─ _active 中, 軸値を deadzone 後に速度積分し target_x/y/z を更新
        ├─ R1/L1 で gripper_value を更新し /teleop/gripper を publish
        └─ TargetPoseWithInput (stick=0) を /teleop/target に publish
```

## 4. 詳細設計

### 4.1 coordinate_utils (lib)

```cpp
namespace teleop_ik {

// Unity (left-handed Y-up, X-right Z-forward) → ROS (right-handed Z-up, X-forward Y-left)
// ros_x =  unity_z
// ros_y = -unity_x
// ros_z =  unity_y
Eigen::Vector3d unity_position_to_ros(double x, double y, double z, double scale = 1.0);

// Vector part follows the same axis mapping. w is flipped for handedness.
Eigen::Vector4d unity_quaternion_to_ros(double x, double y, double z, double w);

}  // namespace teleop_ik
```

Python 実装と完全一致する. スケールは `scale` 引数で適用.

### 4.2 ik_node (C++)

#### 4.2.1 パラメータ

Python 版と同一. デフォルト値も同一.

| 名前 | 型 | デフォルト | 用途 |
| --- | --- | --- | --- |
| `urdf_path` | string | "" (必須) | xacro/URDF パス |
| `end_effector_frame` | string | "gripper" | Pinocchio フレーム名 |
| `position_scale` | double | 1.0 | Unity→ROS 位置のスケール |
| `ik_damping` | double | 1e-6 | CLIK のダンピング係数 |
| `ik_max_iterations` | int | 100 | IK 反復上限 |
| `ik_tolerance` | double | 1e-4 | 収束判定 |
| `trajectory_time_from_start` | double | 0.1 | JointTrajectoryPoint の time_from_start |
| `unity_conversion` | bool | true | Unity→ROS 座標変換を行うか |
| `stick_velocity_scale` | double | 1.5 | スティック角速度スケール (rad/s) |
| `stick_deadzone` | double | 0.1 | スティック deadzone |
| `stick_max_delta_per_msg` | double | 0.2 | 1 メッセージあたりの手首角制限 |
| `stick_fallback_dt` | double | 0.0111 | stamp 不正時のフォールバック dt |

#### 4.2.2 状態変数

- `model_`, `data_` (Pinocchio), `ee_frame_id_`
- `arm_joint_ids_` (5 個: 1〜5), `position_joint_ids_` (1〜3), `wrist_joint_ids_` (4〜5)
- `q_current_`, `q_solution_` (Eigen::VectorXd)
- `active_`, `arm_init_pos_`, `unity_anchor_pos_` (Eigen::Vector3d, optional), `wrist_init_pos_` (Eigen::Vector2d, optional)
- `integrated_stick_` (Eigen::Vector2d), `last_msg_stamp_` (std::optional<double>)
- `trajectory_time_` (double, パラメータキャッシュ)

#### 4.2.3 xacro 処理

```cpp
std::string process_xacro(const std::string& xacro_path) {
  // 1) xacro CLI を起動
  //    "xacro <xacro_path>" を popen して stdout を読み込む
  // 2) 失敗したら RCLCPP_FATAL で throw
}
```

Python 版の `_process_xacro` のフォールバック実装と等価. Python の
`xacro` モジュール呼び出しは廃止 (ライブラリ依存を増やさないため).

> 前提: Nix 開発シェルに `xacro` コマンドが PATH に存在する.
> `ros2_ws/nix/shell.nix` に既載 (Python パッケージとして).

#### 4.2.4 IK ソルバ (CLIK, damped least squares)

Python 版と数式・戻り値・ログ出力を完全一致させる.

```cpp
std::optional<Eigen::VectorXd> solve_ik(
    const Eigen::Vector3d& target_position, const Eigen::VectorXd& q_seed);
```

アルゴリズム:

1. `q = clamp_joints(q_seed)`
2. `position_velocity_indexes = {joints[position_joint_ids_[i]].idx_v}`
3. `dt = 0.2` (固定)
4. for _ in range(max_iter):
   - `pinocchio::forwardKinematics(model, data, q)`
   - `pinocchio::updateFramePlacements(model, data)`
   - `err = target_position - data.oMf[ee_frame_id_].translation()`
   - if `||err|| < tol`: return `clamp_joints(q)`
   - `J = pinocchio::computeFrameJacobian(model, data, q, ee_frame_id_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED).topRows(3).col(position_velocity_indexes)`
   - `JJt = J * J.transpose() + damping * I3`
   - `position_dq = J.transpose() * JJt.ldlt().solve(err)`
   - `dq = zeros(nv); dq[position_velocity_indexes] = position_dq`
   - `q = clamp_joints(pinocchio::integrate(model, q, dq * dt))`
5. 失敗: `RCLCPP_WARN` して `std::nullopt` を返す

#### 4.2.5 スティック deadzone

`apply_stick_deadzone(x, y, deadzone)` は Python 版と数式同一:

```
mag = hypot(x, y)
if mag <= deadzone or mag < 1e-9 or deadzone >= 1.0: return (0, 0)
scale = (mag - deadzone) / (mag * (1 - deadzone))
return (x * scale, y * scale)
```

#### 4.2.6 stamp → time

```cpp
std::optional<double> stamp_to_time(const builtin_interfaces::msg::Time& stamp);
// sec==0 and nanosec==0 → nullopt
// otherwise: sec + nanosec * 1e-9
```

#### 4.2.7 clamp_joints

```cpp
Eigen::VectorXd clamp_joints(const Eigen::VectorXd& q) const;
// q を model.lowerPositionLimit と model.upperPositionLimit で要素ごと clamp
```

Python 版と等価 (`np.clip` の C++ 版).

#### 4.2.8 publish_arm_trajectory / publish_gripper_trajectory

JointTrajectory の組み立て方は Python 版と同一.

```cpp
trajectory_msgs::msg::JointTrajectory make_arm_trajectory(const Eigen::VectorXd& q) const;
trajectory_msgs::msg::JointTrajectory make_gripper_trajectory(double angle) const;
```

### 4.3 gamepad_node (C++)

Python 版と同一の挙動.

- パラメータ: `publish_rate`, `linear_speed`, `vertical_speed`,
  `deadzone`, `axis_x`, `axis_y`, `axis_z`, `button_gripper_open`,
  `button_gripper_close`, `button_toggle_active`
- 50 Hz タイマで target 位置・gripper を publish
- Cross (デフォルト 0) の rising edge で `_active` をトグル
- stick = (0, 0) で `TargetPoseWithInput` を publish
- QoS なし (デフォルト RELIABLE) で OK (Python 版と同じ)

### 4.4 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)
project(teleop_ik)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  if(WARNINGS_AS_ERRORS)
    add_compile_options(-Werror)
  endif()
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/TargetPoseWithInput.msg
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
)

add_library(teleop_ik_core SHARED
  src/coordinate_utils.cpp
)
target_include_directories(teleop_ik_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(teleop_ik_core Eigen3)
target_link_libraries(teleop_ik_core pinocchio::pinocchio)

add_executable(teleop_ik_node src/ik_node.cpp)
target_link_libraries(teleop_ik_node teleop_ik_core)
ament_target_dependencies(teleop_ik_node
  rclcpp geometry_msgs std_msgs sensor_msgs trajectory_msgs builtin_interfaces)
target_link_libraries(teleop_ik_node pinocchio::pinocchio)

add_executable(gamepad_teleop_node src/gamepad_node.cpp)
ament_target_dependencies(gamepad_teleop_node
  rclcpp std_msgs sensor_msgs)

install(TARGETS
  teleop_ik_core teleop_ik_node gamepad_teleop_node
  EXPORT export_teleop_ikTargets
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_coordinate_utils test/test_coordinate_utils.cpp)
  target_link_libraries(test_coordinate_utils teleop_ik_core)
  ament_add_gtest(test_target_msg test/test_target_msg.cpp)
  ament_add_gtest(test_ik_node test/test_ik_node.cpp)
  target_link_libraries(test_ik_node teleop_ik_core)
  ament_add_gtest(test_qos test/test_qos.cpp)
  target_link_libraries(test_qos teleop_ik_node)
  ament_add_gtest(test_packaging test/test_packaging.cpp)
endif()

ament_package()
```

> `pinocchio::pinocchio` というターゲット名は upstream によって異なる.
> nixpkgs 4.0.0 の `pkgs.pinocchio` は `pkg-config` 経由で公開するため,
> 実装時に `pkg_check_modules(pinocchio REQUIRED IMPORTED_TARGET pinocchio)` か,
> `find_package(pinocchio QUIET)` を確認する. 不確実なら
> `pkg-config` ベース (`pkg_check_modules`) にフォールバックする.
> 詳細は plan で詰める.

### 4.5 package.xml

```xml
<package format="3">
  <name>teleop_ik</name>
  <version>0.1.0</version>
  <description>VR teleop IK node for SO-101 arm using Pinocchio (C++)</description>
  <maintainer email="ojii3dev@gmail.com">OJII3</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>trajectory_msgs</depend>
  <depend>builtin_interfaces</depend>
  <depend>rclcpp</depend>
  <depend>eigen</depend>
  <depend>pinocchio</depend>

  <exec_depend>joy</exec_depend>
  <exec_depend>lerobot_controller</exec_depend>
  <exec_depend>lerobot_description</exec_depend>

  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

Python 依存 (`rclpy`, `python3-pytest`) は削除. `pinocchio` は
`pkgs.pinocchio` の C++ ライブラリ.

## 5. テスト戦略

### 5.1 gtest 一覧

| テストファイル | 対象 | テスト内容 |
| --- | --- | --- |
| `test_coordinate_utils.cpp` | `unity_position_to_ros`, `unity_quaternion_to_ros` | Python 版 `test_coordinate_utils.py` と同一の期待値 (軸マッピング, scale 適用, w 反転) |
| `test_target_msg.cpp` | `teleop_ik::msg::TargetPoseWithInput` | デフォルト値 0, フィールド代入, ヘッダ確認 |
| `test_ik_node.cpp` | `clamp_joints`, `solve_ik`, `_on_target_with_input` のスティック積分 | URDF 制限, 収束/非収束, スティック dt 積分, セッションリセット, deadzone, max delta |
| `test_qos.cpp` | ノード QoS 設定 | `/teleop/target` 購読が BEST_EFFORT, `/teleop/active` と `/teleop/gripper` が RELIABLE |
| `test_packaging.cpp` | インストール成果物 | `ik_node`, `gamepad_teleop_node` 実行ファイルが `lib/teleop_ik/` に存在し, pinocchio ライブラリがリンクされている |
| `test_vr_teleop_launch.py` | launch ファイル構造 | 残す (Python launch ファイルなので pytest のまま) |

### 5.2 テスト容易性のための設計

`test_ik_node.cpp` の `_on_target_with_input` 系テストは
Python 版で `TeleopIKNode.__new__` していた箇所に対応する. C++ では
`TeleopIKNode` の public/private 境界を以下のようにする.

- 公開 API (`on_active`, `on_target_with_input`, `on_gripper`,
  `on_joint_states`): テストからもアクセス可能 (`public:`).
- 内部ロジック (`solve_ik`, `clamp_joints`,
  `apply_stick_deadzone`, `stamp_to_time`): `public:` にして
  gtest から直接呼べるようにする. これらは「ノードのメンバとして
  振る舞う」だけで, 隠蔽する強い理由がない (Python 版も
  アンダースコア convention だけで, テストからは参照していた).

別解として, `ik_node` のロジックを `ik_solver` クラス (lib ターゲット
`ik_solver` を `teleop_ik_core` に追加) に切り出す手もあるが,
スコープを抑えるため, まずは前者 (メンバ関数を public 化) で
行く. リファクタが要るなら plan フェーズで再検討.

### 5.3 パラメータ注入

`test_ik_node.cpp` の `_on_target_with_input` 系テストでは,
`TeleopIKNode` コンストラクタを呼ばずにモデル・状態を組み立てる
必要がある. 以下の静的メソッドまたは friend テストクラスを用意.

```cpp
class TeleopIKNode : public rclcpp::Node {
  // ...
 public:
  // テスト用セットアップ. URDF XML を直接渡せる.
  static std::unique_ptr<TeleopIKNode> make_for_test(
      const rclcpp::NodeOptions& options,
      const std::string& urdf_xml);
};
```

または, コンストラクタに `rclcpp::NodeOptions` 経由で
`urdf_path = ""` を許容するオーバーロードを追加. 詳細は
plan フェーズで詰める.

## 6. Nix 環境更新

`ros2_ws/nix/shell.nix` に `pinocchio` を追加:

```nix
with pkgs;
with pkgs.rosPackages.${rosDistro};
with extraPkgs;
[
  # 既存
  ament-cmake
  ament-lint-auto
  ament-lint-common
  ...
  # 追加
  pinocchio
]
```

`python3Packages.pinocchio` と `python3Packages.coal` は他パッケージ
で使われていないか確認し, 使われていなければ削除. (確認の結果を
plan フェーズで反映.)

`ros2_ws/nix/*.nix` は `ros2nix` で再生成する:

```bash
cd ros2_ws
nix run github:wentasah/ros2nix -- \
  --distro jazzy --output-dir nix --output-as-nix-pkg-name --nixfmt \
  $(find src -name package.xml)
```

ただし Nix 式の自動生成は diff が大きくなりがちなので, まずは
`shell.nix` のみ手動更新 + `*.nix` は再生成の差分を確認してから
マージする方針.

## 7. マイルストーン

| # | 内容 | 検証 |
| --- | --- | --- |
| 1 | `coordinate_utils.hpp/cpp` + gtest (`test_coordinate_utils.cpp`) | gtest パス |
| 2 | `CMakeLists.txt` を C++ ビルドに書き換え, lib ターゲットがビルド可能 | `colcon build` (少なくとも `teleop_ik_core` まで) |
| 3 | `ik_node.cpp` 実装 + 単体テスト (`test_ik_node.cpp`) | gtest パス |
| 4 | `gamepad_node.cpp` 実装 | `colcon build` 成功 |
| 5 | `test_qos.cpp`, `test_target_msg.cpp`, `test_packaging.cpp` 追加 | 全 gtest パス |
| 6 | Python 版ソースと関連テストを削除, `package.xml` 整理 | `colcon build` 警告なし, `test_vr_teleop_launch.py` 残存 |
| 7 | `ros2_ws/nix/shell.nix` 更新, `ros2_ws/nix/*.nix` 再生成 | `nix flake check` または `nix develop .#ros` の評価成功 |
| 8 | launch 経由で起動でき, 既存トピック/パラメータがすべて公開されている | (実機がないため) 起動してパラメータ一覧と topic 一覧を目視確認 |

## 8. リスクと対策

| リスク | 対策 |
| --- | --- |
| `pinocchio` 4.0.0 の C++ API で `buildModelFromXML` のシグネチャが Python と異なる (Python は dict, C++ は string) | `urdf_xml` (string) を渡して同じ結果になる. C++ は signature が単純なので問題なし. |
| `pinocchio::neutral(model)` の戻り値型の差異 (`Eigen::VectorXd` vs `Model::ConfigVectorType`) | `Model::ConfigVectorType` または `Eigen::VectorXd` で受け取れる. `auto` で受け, メンバに格納. |
| `computeFrameJacobian` の C++ シグネチャ (テンプレート引数) | 引数の渡し方は実装着手時に pinocchio の example を確認. 動作確認は plan フェーズ. |
| xacro CLI 起動時の PATH 不足 | `popen` 起動前に `getenv("PATH")` をログ. 失敗時は RCLCPP_FATAL. |
| gtest から `rclcpp::init/shutdown` を呼ぶ際の制約 | 各テストケースで `SetUp/TearDown` を使い, 1 度だけ init する fixture を用意. |
| Eigen の `Eigen::VectorXd` と `pinocchio::neutral` のサイズ差異 | pinocchio の model の nq と同じサイズの VectorXd として受け取る. サイズが合わなければ assert. |
| 既存の launch / YAML との非互換 | ノード名・トピック名・パラメータ名・型を Python 版と 1:1 で維持. 起動テストはマイルストーン 8 で実施. |

## 9. 範囲外

- IK アルゴリズムの改良 (CLIK → 何かに変更など). 機能等価の書き換え
  のみ.
- launch ファイルの C++ 化 (ROS 2 は launch を C++ で書けるが,
  既存資産を温存).
- 新規パラメータの追加. パラメータは現行のものだけ.
- `TargetPoseWithInput.msg` の変更. 現行のまま.
- 実機での VR テレオペ確認 (Linux 実機 / VM が必要なため).
  マイルストーン 8 の目視確認までで完了とする.

## 10. 関連ドキュメント

- [`ros2_ws/README.md`](../../../ros2_ws/README.md) … ROS 2 ワークスペース全体の説明
- [`flake.nix`](../../../flake.nix) … Nix 開発シェル定義
- [`docs/superpowers/specs/2026-06-21-soarmvr-anchor-projection-stick-wrist-design.md`](2026-06-21-soarmvr-anchor-projection-stick-wrist-design.md) … 直近の関連 spec (anchor + stick 手首)
