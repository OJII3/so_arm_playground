# teleop_ik: VR リセットコマンド (IK バイパス) 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** VR コントローラの A ボタン押下で, SO-101 follower のアーム 5 関節 + グリッパを IK バイパスで home 姿勢に戻すリセット機能を追加する.

**Architecture:**
- 新規 ROS メッセージ `teleop_ik/msg/ResetCommand.msg` を定義. NaN フォールバックで任意オーバーライド可.
- `teleop_ik_node` が `/teleop/reset` を subscribe し, `home_j*_rad` パラメータ + メッセージの値で JointTrajectory を 2 つ (arm / gripper) publish. IK 計算は走らない.
- SoArmVR (Unity) 側: inputactions に Reset を追加し, RosTeleoperationSink に publisher を持つ. rising edge で 1 ショット publish.

**Tech Stack:** ROS 2 Jazzy, C++17, rclcpp, Pinocchio 4.0.0, gtest (ament_cmake_gtest), colcon, Nix (`nix develop .#ros`), C# / Unity (nix develop .#soarmvr, rosettadds-genmsg).

**Spec:** `docs/superpowers/specs/2026-07-01-vr-reset-command-design.md`

**参考:**
- 既存実装: `ros2_ws/src/teleop_ik/src/ik_node.cpp`, `ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp`
- 既存テスト: `ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp`, `ros2_ws/src/teleop_ik/test/test_target_msg.cpp`
- 既存 message 定義: `ros2_ws/src/teleop_ik/msg/TargetPoseWithInput.msg`
- 既存 VR sink: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/RosTeleoperationSink.cs`
- C# msg 生成 spec: `docs/superpowers/specs/2026-06-13-soarmvr-rosettadds-migration-design.md`

---

## File Structure

### Created files (ROS 2)
- `ros2_ws/src/teleop_ik/msg/ResetCommand.msg` — 新規メッセージ定義.
- `ros2_ws/src/teleop_ik/test/test_reset_msg.cpp` — メッセージのラウンドトリップ単体テスト.

### Modified files (ROS 2)
- `ros2_ws/src/teleop_ik/CMakeLists.txt` — `rosidl_generate_interfaces` に `ResetCommand.msg` を追加.
- `ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp` — `on_reset_msg` / `on_reset` 宣言, `sub_reset_` メンバ追加.
- `ros2_ws/src/teleop_ik/src/ik_node.cpp` — `init_ros_node` に param 宣言 + `sub_reset_` 作成, `on_reset_msg` / `on_reset` 実装.
- `ros2_ws/src/teleop_ik/config/teleop_ik_params.yaml` — `home_j1_rad`..`home_j6_rad` + `reset_duration_sec` を追加.
- `ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp` — `ResetFixture` と 7 件のテスト追加.

### Created files (Unity / SoArmVR)
- `SoArmVR/Assets/Msgs/teleop_ik/msg/ResetCommand.msg` — ROS 側 .msg のミラー.
- `SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/TeleopIk/ResetCommandMessage.cs` ほか — `rosettadds-genmsg` が生成. リポジトリにコミット.
- `SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/TeleopIk/ResetCommandMessageSerializer.cs` — 同上.
- `SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/TeleopIk/ResetCommandMessage.meta` ほか — 同上.

### Modified files (Unity / SoArmVR)
- `SoArmVR/Assets/_SoArmVR/Input/SoArmTeleoperation.inputactions` — `Reset` アクション + `<XRController>{RightHand}/primaryButton` バインド.
- `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/ITeleoperationSink.cs` — `PublishReset()` 追加.
- `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/DebugTeleoperationSink.cs` — 空実装追加.
- `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/RosTeleoperationSink.cs` — `_resetPub` + `PublishReset` 実装.
- `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationSession.cs` — `_resetAction` + rising edge 検出.

### Unchanged files
- `ros2_ws/src/teleop_ik/launch/*.py` (follower / gamepad / vr_teleop launch は無改修)
- `ros2_ws/src/lerobot_controller/**` (follower 側)
- `ros2_ws/src/feetech_ros2_driver/**` (driver 側)
- `SoArmVR/Assets/_SoArmVR/Scenes/**` (シーン側の参照は SerializeField 経由で自動結線)
- `SoArmVR/Assets/_SoArmVR/Scripts/_SoArmVR.asmdef` (新 namespace 追加があれば Task で更新)

---

## 事前準備

実装着手前に以下を確認すること.

- [ ] Linux ホストで `nix develop .#ros` と `nix develop .#soarmvr` が起動できること
- [ ] `ros2_ws/` で `colcon build --symlink-install --packages-select teleop_ik` が現状で成功すること
- [ ] `colcon test --packages-select teleop_ik` が全パスすること (ベースライン)
- [ ] `SoArmVR/Assets/Msgs/teleop_ik/` ディレクトリが存在すること (無ければ作成)

---

## Task 1: ブランチ作成とベースライン確認

**Files:**
- Modify: git 状態

- [ ] **Step 1: フィーチャーブランチを作成して checkout**

```bash
git checkout -b feat/vr-reset-command
```

- [ ] **Step 2: ベースラインが通ることを確認**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon build --symlink-install --packages-select teleop_ik && colcon test --packages-select teleop_ik"
```

Expected: ビルドと全テストが成功する. 問題があればベースラインから直す.

- [ ] **Step 3: Task 2 へ進む**

ベースラインが通れば次へ.

---

## Task 2: ResetCommand.msg 新規定義 + CMakeLists 更新

**Files:**
- Create: `ros2_ws/src/teleop_ik/msg/ResetCommand.msg`
- Modify: `ros2_ws/src/teleop_ik/CMakeLists.txt:28-31`

新規メッセージを定義し, `rosidl_generate_interfaces` に登録する. この時点では生成コードが壊れるが Task 4 で直す.

- [ ] **Step 1: .msg ファイルを作成**

`ros2_ws/src/teleop_ik/msg/ResetCommand.msg` を以下の内容で作成:

```
# teleop_ik/msg/ResetCommand.msg
# Signal to publish a JointTrajectory to move the arm + gripper to a
# pre-configured home pose, bypassing IK.

# Standard ROS header
std_msgs/Header header

# Home joint angles in radians.
# Element order matches joint_names ["1", "2", "3", "4", "5", "6"].
# NaN value at index i = use node parameter `home_j{i+1}_rad` for that joint.
# This allows partial overrides (e.g., override arm but keep gripper default).
float32[6] home_joints

# Trajectory time_from_start in seconds.
# <= 0.0 = use node parameter `reset_duration_sec`.
float32 duration_sec
```

- [ ] **Step 2: CMakeLists.txt の `rosidl_generate_interfaces` を更新**

`ros2_ws/src/teleop_ik/CMakeLists.txt:28-31` を以下に置換:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/TargetPoseWithInput.msg
  msg/ResetCommand.msg
  DEPENDENCIES geometry_msgs std_msgs
)
```

- [ ] **Step 3: ビルド成功を確認**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon build --symlink-install --packages-select teleop_ik 2>&1 | tail -30"
```

Expected: ビルド成功. `teleop_ik__rosidl_typesupport_cpp` 等の生成物に `ResetCommand` が含まれる (CMake 出力に `teleop_ik/msg/ResetCommand.hpp` 等の生成行が見える).

- [ ] **Step 4: 生成ヘッダの存在を確認**

```bash
ls ros2_ws/build/teleop_ik/rosidl_generator_cpp/teleop_ik/msg/
```

Expected: `reset_command.hpp` が存在する.

- [ ] **Step 5: コミット**

```bash
git add ros2_ws/src/teleop_ik/msg/ResetCommand.msg ros2_ws/src/teleop_ik/CMakeLists.txt
git commit -m "feat(teleop_ik): add ResetCommand message definition"
```

---

## Task 3: メッセージ単体テストを追加 (TDD: 赤)

**Files:**
- Create: `ros2_ws/src/teleop_ik/test/test_reset_msg.cpp`
- Modify: `ros2_ws/src/teleop_ik/CMakeLists.txt:90-92` (gtest ターゲット登録)

- [ ] **Step 1: テストファイルを作成**

`ros2_ws/src/teleop_ik/test/test_reset_msg.cpp` を以下の内容で作成:

```cpp
// teleop_ik/test/test_reset_msg.cpp
#include <cmath>
#include <gtest/gtest.h>

#include "teleop_ik/msg/reset_command.hpp"

TEST(ResetCommandMsg, DefaultValues)
{
  teleop_ik::msg::ResetCommand msg;
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_EQ(msg.home_joints[i], 0.0f) << "i=" << i;
  }
  EXPECT_EQ(msg.duration_sec, 0.0f);
}

TEST(ResetCommandMsg, SetFields)
{
  teleop_ik::msg::ResetCommand msg;
  msg.home_joints[0] = 0.1f;
  msg.home_joints[3] = -0.5f;
  msg.home_joints[5] = 1.57f;
  msg.duration_sec = 2.5f;
  EXPECT_FLOAT_EQ(msg.home_joints[0], 0.1f);
  EXPECT_FLOAT_EQ(msg.home_joints[3], -0.5f);
  EXPECT_FLOAT_EQ(msg.home_joints[5], 1.57f);
  EXPECT_FLOAT_EQ(msg.duration_sec, 2.5f);
}

TEST(ResetCommandMsg, NaNSentinel)
{
  teleop_ik::msg::ResetCommand msg;
  for (size_t i = 0; i < 6; ++i) {
    msg.home_joints[i] = std::numeric_limits<float>::quiet_NaN();
  }
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_TRUE(std::isnan(msg.home_joints[i])) << "i=" << i;
  }
}
```

> ヘッダ名は rosidl_generator_cpp の命名規則に従い
> `reset_command.hpp` (snake_case) になる.
> もし CMake ビルド時に大文字始まり (`ResetCommand.hpp`) になったら
> include を `#include "teleop_ik/msg/ResetCommand.hpp"` に直すこと.

- [ ] **Step 2: CMakeLists.txt に gtest ターゲットを追加**

`ros2_ws/src/teleop_ik/CMakeLists.txt` の `ament_add_gtest(test_target_msg_cpp ...)` の下あたりに追記:

```cmake
  ament_add_gtest(test_reset_msg_cpp test/test_reset_msg.cpp)
  target_link_libraries(test_reset_msg_cpp teleop_ik__rosidl_typesupport_cpp)
```

- [ ] **Step 3: ビルドして成功を確認**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon build --symlink-install --packages-select teleop_ik --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -20"
```

Expected: ビルド成功.

- [ ] **Step 4: テスト実行 — 全パスすることを確認**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon test --packages-select teleop_ik --event-handlers console_direct+ 2>&1 | tail -30"
```

Expected: `test_reset_msg_cpp` の 3 ケース全 PASS. 既存テストも全 PASS.

- [ ] **Step 5: コミット**

```bash
git add ros2_ws/src/teleop_ik/test/test_reset_msg.cpp ros2_ws/src/teleop_ik/CMakeLists.txt
git commit -m "test(teleop_ik): add unit tests for ResetCommand message"
```

---

## Task 4: ik_node.hpp に on_reset_msg / on_reset / sub_reset_ を宣言

**Files:**
- Modify: `ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp:14-23, 79-111`

ヘッダの宣言のみ追加. 実装は Task 6.

- [ ] **Step 1: include に `teleop_ik/msg/reset_command.hpp` を追加**

`ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp:20` (`#include <teleop_ik/msg/target_pose_with_input.hpp>`) の下に追記:

```cpp
#include <teleop_ik/msg/reset_command.hpp>
```

- [ ] **Step 2: on_reset_msg 宣言を追加**

`ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp:83` (`void on_joint_states_msg(...)`) の上あたりに追記:

```cpp
  void on_reset_msg(const teleop_ik::msg::ResetCommand::SharedPtr msg);
```

- [ ] **Step 3: on_reset 宣言を private に追加**

`ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp` の `private:` セクション (`void init_ros_node();` の下) に追加:

```cpp
  void on_reset(const teleop_ik::msg::ResetCommand & msg);
```

- [ ] **Step 4: sub_reset_ メンバを宣言**

`ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp:108` (`rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;`) の下に追加:

```cpp
  rclcpp::Subscription<teleop_ik::msg::ResetCommand>::SharedPtr sub_reset_;
```

- [ ] **Step 5: ビルドして成功を確認**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon build --symlink-install --packages-select teleop_ik 2>&1 | tail -20"
```

Expected: ヘッダ追加だけならリンク失敗はしない (`.cpp` 側に `on_reset_msg` / `on_reset` の定義が無いため警告や未定義参照の可能性あり. 詳細は Step 6 の期待値参照).

- [ ] **Step 6: テスト実行して現状を確認 (まだテストは無いので既存のみ実行)**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon test --packages-select teleop_ik --event-handlers console_direct+ 2>&1 | tail -30"
```

Expected: Task 4 時点では `on_reset_msg` / `on_reset` の実装が無いので, ライブラリだけビルドできれば OK. もしリンクエラーで `colcon test` が走らない場合は Task 5 で先にパラメータ宣言 + `sub_reset_` 作成 (`.cpp` 側のスケルトン) を入れて, テストは Task 6 まで保留. ただし「宣言だけ追加」自体はヘッダオンリーの変更で `.cpp` に手を入れないため, ライブラリ側のリンクは通る.

- [ ] **Step 7: コミット**

```bash
git add ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp
git commit -m "refactor(teleop_ik): declare on_reset_msg / on_reset / sub_reset_ in header"
```

---

## Task 5: on_reset テストを追加 (TDD: 赤)

**Files:**
- Modify: `ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp` (末尾に `ResetFixture` と 7 件のテスト追加)

- [ ] **Step 1: 必要な include をファイルの先頭に追加**

`ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp` の include ブロックに以下が無ければ追加:

```cpp
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <teleop_ik/msg/reset_command.hpp>
```

- [ ] **Step 2: `ResetFixture` を `CallbacksFixture` の直後に追加**

`ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp` の `CallbacksFixture` 構造体定義の直後 (187 行目 `}  // namespace` の直前) に追加:

```cpp
struct ResetFixture : public TeleopIKHelpersTest
{
  void SetUp() override
  {
    const char * path = std::getenv("TELEOP_IK_TEST_URDF_PATH");
    ASSERT_NE(path, nullptr) << "Set TELEOP_IK_TEST_URDF_PATH to expanded URDF file path";
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({rclcpp::Parameter("urdf_path", std::string(path))});
    for (size_t i = 0; i < 6; ++i) {
      const std::string name = "home_j" + std::to_string(i + 1) + "_rad";
      opts.parameter_overrides.push_back(
          rclcpp::Parameter(name, 0.1 * static_cast<double>(i + 1)));
    }
    opts.parameter_overrides.push_back(
        rclcpp::Parameter("reset_duration_sec", 1.5));
    node_ = std::make_shared<teleop_ik::TeleopIKNode>(opts);
    ASSERT_NE(node_, nullptr);
  }
  std::shared_ptr<teleop_ik::TeleopIKNode> node_;
};
```

> `teleop_ik::TeleopIKNode` には `(rclcpp::NodeOptions)` を受ける
> コンストラクタが既存 (`ik_node.hpp:36` 参照).

- [ ] **Step 3: 7 件の失敗するテストを末尾に追加**

`ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp` のファイル末尾 (最後の `}` の直前) に追加:

```cpp
// ---- ResetCommand 経路 ----

TEST_F(ResetFixture, OnResetUsesParamDefaultsForAllNaN)
{
  teleop_ik::msg::ResetCommand msg;
  for (size_t i = 0; i < 6; ++i) {
    msg.home_joints[i] = std::numeric_limits<float>::quiet_NaN();
  }
  msg.duration_sec = 0.0f;

  node_->on_reset_msg(std::make_shared<teleop_ik::msg::ResetCommand>(msg));

  // セッションがクリアされる.
  EXPECT_FALSE(node_->active_);
  // パラメータ値で JointTrajectory が publish されることは
  // OnResetPublishesArmAndGripper で検証する.
}

TEST_F(ResetFixture, OnResetPartialOverride)
{
  teleop_ik::msg::ResetCommand msg;
  msg.home_joints[0] = 0.7f;
  for (size_t i = 1; i < 6; ++i) {
    msg.home_joints[i] = std::numeric_limits<float>::quiet_NaN();
  }
  msg.duration_sec = 0.0f;

  node_->on_reset_msg(std::make_shared<teleop_ik::msg::ResetCommand>(msg));

  EXPECT_FALSE(node_->active_);
}

TEST_F(ResetFixture, OnResetUsesProvidedDuration)
{
  teleop_ik::msg::ResetCommand msg;
  msg.duration_sec = 0.5f;
  // home_joints は使わない (duration だけテスト).

  node_->on_reset_msg(std::make_shared<teleop_ik::msg::ResetCommand>(msg));

  // 仕様: duration_sec > 0 → そのまま使用.
  // JointTrajectory の中身は OnResetPublishesArmAndGripper で確認.
  EXPECT_FALSE(node_->active_);
}

TEST_F(ResetFixture, OnResetDurationFallsBackOnZeroOrNaN)
{
  teleop_ik::msg::ResetCommand msg;
  msg.duration_sec = 0.0f;

  node_->on_reset_msg(std::make_shared<teleop_ik::msg::ResetCommand>(msg));

  // 仕様: duration_sec <= 0 → パラメータ値 (1.5 sec) を使用.
  // ここで duration を直接検証できないので OnResetPublishesArmAndGripper で確認.
  EXPECT_FALSE(node_->active_);
}

TEST_F(ResetFixture, OnResetClearsActiveSession)
{
  node_->active_ = true;
  node_->unity_anchor_set_ = true;
  node_->unity_anchor_pos_ = Eigen::Vector3d(1.0, 2.0, 3.0);
  node_->integrated_stick_ = Eigen::Vector2d(0.4, 0.5);

  teleop_ik::msg::ResetCommand msg;
  node_->on_reset_msg(std::make_shared<teleop_ik::msg::ResetCommand>(msg));

  EXPECT_FALSE(node_->active_);
  EXPECT_FALSE(node_->unity_anchor_set_);
  EXPECT_EQ(node_->integrated_stick_.x(), 0.0);
  EXPECT_EQ(node_->integrated_stick_.y(), 0.0);
}

TEST_F(ResetFixture, OnResetPublishesArmAndGripper)
{
  // 別ノードを作って publish を捕捉.
  auto probe = std::make_shared<rclcpp::Node>("reset_probe");
  std::vector<trajectory_msgs::msg::JointTrajectory> arm_msgs;
  std::vector<trajectory_msgs::msg::JointTrajectory> gripper_msgs;
  auto arm_sub = probe->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/follower/arm_controller/joint_trajectory", 10,
      [&](trajectory_msgs::msg::JointTrajectory::SharedPtr m) { arm_msgs.push_back(*m); });
  auto gripper_sub = probe->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/follower/gripper_controller/joint_trajectory", 10,
      [&](trajectory_msgs::msg::JointTrajectory::SharedPtr m) { gripper_msgs.push_back(*m); });

  teleop_ik::msg::ResetCommand msg;
  for (size_t i = 0; i < 6; ++i) {
    msg.home_joints[i] = std::numeric_limits<float>::quiet_NaN();
  }
  msg.duration_sec = 0.0f;
  node_->on_reset_msg(std::make_shared<teleop_ik::msg::ResetCommand>(msg));

  // publish を処理.
  rclcpp::spin_some(node_);
  rclcpp::spin_some(probe);

  ASSERT_EQ(arm_msgs.size(), 1u);
  ASSERT_EQ(gripper_msgs.size(), 1u);
  // arm: joint_names = {"1".."5"}, positions は param 値.
  ASSERT_EQ(arm_msgs[0].joint_names.size(), 5u);
  ASSERT_EQ(arm_msgs[0].points.size(), 1u);
  EXPECT_EQ(arm_msgs[0].points[0].positions.size(), 5u);
  EXPECT_NEAR(arm_msgs[0].points[0].positions[0], 0.1, 1e-6);
  EXPECT_NEAR(arm_msgs[0].points[0].positions[1], 0.2, 1e-6);
  EXPECT_NEAR(arm_msgs[0].points[0].positions[2], 0.3, 1e-6);
  EXPECT_NEAR(arm_msgs[0].points[0].positions[3], 0.4, 1e-6);
  EXPECT_NEAR(arm_msgs[0].points[0].positions[4], 0.5, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(arm_msgs[0].points[0].time_from_start).seconds(), 1.5, 1e-6);
  // gripper: joint_names = {"6"}, positions[0] = 0.6.
  ASSERT_EQ(gripper_msgs[0].joint_names.size(), 1u);
  ASSERT_EQ(gripper_msgs[0].points.size(), 1u);
  EXPECT_EQ(gripper_msgs[0].points[0].positions.size(), 1u);
  EXPECT_NEAR(gripper_msgs[0].points[0].positions[0], 0.6, 1e-6);
}

TEST_F(ResetFixture, OnResetDoesNotInvokeSolver)
{
  node_->q_solution_ = node_->q_current_;

  teleop_ik::msg::ResetCommand msg;
  for (size_t i = 0; i < 6; ++i) {
    msg.home_joints[i] = std::numeric_limits<float>::quiet_NaN();
  }
  node_->on_reset_msg(std::make_shared<teleop_ik::msg::ResetCommand>(msg));

  EXPECT_NEAR((node_->q_solution_ - node_->q_current_).norm(), 0.0, 1e-9);
}
```

> `OnResetPublishesArmAndGripper` だけ
> `rclcpp::spin_some` を使うので
> `RclcppEnvironment` (ファイル先頭の global env) の
> `rclcpp::init` が走っている前提. 既存テストと同じ global env
> を使うので追加不要.

- [ ] **Step 4: ビルドして成功を確認**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon build --symlink-install --packages-select teleop_ik --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -30"
```

Expected: テストはビルドできる. ただし `on_reset_msg` 未実装なのでリンクで `undefined reference` が出る可能性あり. その場合は Task 6 でスタブを入れてリンクを通す.

- [ ] **Step 5: テスト実行 (まだ `on_reset_msg` 未実装なので失敗 or リンク失敗を確認)**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon test --packages-select teleop_ik --event-handlers console_direct+ 2>&1 | tail -40"
```

Expected: `on_reset_msg` 未定義リンクエラー, またはテストが fail. どちらでも「現状では green ではない」ことが確認できれば OK.

- [ ] **Step 6: コミット**

```bash
git add ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp
git commit -m "test(teleop_ik): add TDD tests for on_reset_msg (red)"
```

---

## Task 6: on_reset 実装 (TDD: 緑)

**Files:**
- Modify: `ros2_ws/src/teleop_ik/src/ik_node.cpp` (init_ros_node に param 追加, sub_reset_ 作成, on_reset_msg / on_reset 実装)

- [ ] **Step 1: `init_ros_node` のパラメータ宣言ブロックに 7 個のパラメータを追加**

`ros2_ws/src/teleop_ik/src/ik_node.cpp:131` (`this->declare_parameter<double>("stick_fallback_dt", 0.0111);`) の下に追加:

```cpp
  this->declare_parameter<double>("home_j1_rad", 0.0);
  this->declare_parameter<double>("home_j2_rad", 0.0);
  this->declare_parameter<double>("home_j3_rad", 0.0);
  this->declare_parameter<double>("home_j4_rad", 0.0);
  this->declare_parameter<double>("home_j5_rad", 0.0);
  this->declare_parameter<double>("home_j6_rad", 0.0);
  this->declare_parameter<double>("reset_duration_sec", 2.0);
```

- [ ] **Step 2: `init_ros_node` の subscriber 作成ブロックに `sub_reset_` を追加**

`ros2_ws/src/teleop_ik/src/ik_node.cpp:184-186` (`sub_joint_states_` の `create_subscription`) の下に追加:

```cpp
  sub_reset_ = this->create_subscription<teleop_ik::msg::ResetCommand>(
      "/teleop/reset", 10,
      std::bind(&TeleopIKNode::on_reset_msg, this, std::placeholders::_1));
```

- [ ] **Step 3: `on_reset_msg` と `on_reset` を実装**

`ros2_ws/src/teleop_ik/src/ik_node.cpp` の `on_joint_states_msg` 定義の後 (ファイル末尾の `}  // namespace teleop_ik` の直前) に追加:

```cpp
void TeleopIKNode::on_reset_msg(const teleop_ik::msg::ResetCommand::SharedPtr msg)
{
  on_reset(*msg);
}

void TeleopIKNode::on_reset(const teleop_ik::msg::ResetCommand & msg)
{
  // 1. ホーム関節角度を解決 (NaN は param フォールバック)
  std::array<double, 6> home{};
  const std::array<const char *, 6> param_names = {
    "home_j1_rad", "home_j2_rad", "home_j3_rad",
    "home_j4_rad", "home_j5_rad", "home_j6_rad",
  };
  for (size_t i = 0; i < 6; ++i) {
    if (!std::isnan(msg.home_joints[i])) {
      home[i] = msg.home_joints[i];
    } else {
      home[i] = this->get_parameter(param_names[i]).as_double();
    }
  }

  // 2. duration を解決 (<=0 / NaN は param フォールバック)
  double duration = msg.duration_sec;
  if (!(duration > 0.0)) {  // NaN も false なのでここに落ちる
    duration = this->get_parameter("reset_duration_sec").as_double();
  }

  // 3. セッションを解除 (次の target msg は破棄される)
  active_ = false;
  unity_anchor_set_ = false;
  integrated_stick_.setZero();

  // 4. arm (joint 1-5) と gripper (joint 6) の軌道を publish
  Eigen::VectorXd q_arm = Eigen::VectorXd::Zero(model_.nq);
  for (size_t i = 0; i < 5; ++i) {
    if (arm_joint_ids_[i] == static_cast<pinocchio::JointIndex>(-1)) continue;
    const auto idx_q = model_.joints[arm_joint_ids_[i]].idx_q();
    q_arm[idx_q] = home[i];
  }
  pub_arm_->publish(make_arm_trajectory(q_arm, duration));
  pub_gripper_->publish(make_gripper_trajectory(home[5], duration));
}
```

> 必要なら `init_ros_node` 内や `on_reset` の前に
> `#include <cmath>` が無いことを確認. 既存 `ik_node.cpp:6` に
> `<cmath>` があるので OK.

- [ ] **Step 4: ビルドして成功を確認**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon build --symlink-install --packages-select teleop_ik --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -20"
```

Expected: ビルド成功.

- [ ] **Step 5: テスト実行 — 新規 7 件が全 PASS することを確認**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon test --packages-select teleop_ik --event-handlers console_direct+ 2>&1 | tail -50"
```

Expected:
- `OnResetUsesParamDefaultsForAllNaN` ✓ PASS
- `OnResetPartialOverride` ✓ PASS
- `OnResetUsesProvidedDuration` ✓ PASS
- `OnResetDurationFallsBackOnZeroOrNaN` ✓ PASS
- `OnResetClearsActiveSession` ✓ PASS
- `OnResetPublishesArmAndGripper` ✓ PASS
- `OnResetDoesNotInvokeSolver` ✓ PASS
- 既存テストも全 PASS.

- [ ] **Step 6: コミット**

```bash
git add ros2_ws/src/teleop_ik/src/ik_node.cpp
git commit -m "feat(teleop_ik): implement on_reset path bypassing IK solver"
```

---

## Task 7: teleop_ik_params.yaml に home_j*_rad と reset_duration_sec を追加

**Files:**
- Modify: `ros2_ws/src/teleop_ik/config/teleop_ik_params.yaml`

- [ ] **Step 1: YAML の末尾にパラメータを追加**

`ros2_ws/src/teleop_ik/config/teleop_ik_params.yaml` の末尾に追記:

```yaml
    # Home joint angles for /teleop/reset (VR A button).
    # All zeros (the default) means the arm folds to a "neutral" pose
    # at startup; override per-launch to match the desired rest pose.
    home_j1_rad: 0.0
    home_j2_rad: 0.0
    home_j3_rad: 0.0
    home_j4_rad: 0.0
    home_j5_rad: 0.0
    home_j6_rad: 0.0
    # Time-from-start for the single-waypoint reset trajectory.
    reset_duration_sec: 2.0
```

- [ ] **Step 2: YAML パースが通ることを launch 経由で確認**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon build --symlink-install --packages-select teleop_ik 2>&1 | tail -10"
```

Expected: ビルド成功 (YAML 自体は起動時に読まれるが, ここではファイルが壊れていないか yq 等で読めるか確認.

```bash
nix develop .#ros --command bash -c "yq -P ros2_ws/src/teleop_ik/config/teleop_ik_params.yaml | tail -10"
```

Expected: 末尾 10 行ほどに `home_j1_rad`..`reset_duration_sec` が表示される.

- [ ] **Step 3: コミット**

```bash
git add ros2_ws/src/teleop_ik/config/teleop_ik_params.yaml
git commit -m "feat(teleop_ik): expose home_j*_rad and reset_duration_sec in params yaml"
```

---

## Task 8: 起動経路の smoke 確認 (lifecycle / param 読み込み)

**Files:**
- (確認のみ, ファイル変更なし)

launch から実際に teleop_ik_node を起動し, パラメータが反映されること, `/teleop/reset` を購読できることを確認する.

- [ ] **Step 1: teleop_ik_node を単独で起動**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && ros2 launch teleop_ik teleop_ik.launch.py"
```

Expected: ノードが起動し, `urdf_path` の解決でエラーが出ないこと.
起動したら別ターミナルで:

```bash
nix develop .#ros --command bash -c "ros2 node list && ros2 param list /teleop_ik_node | grep -E 'home_j|reset_duration'"
```

Expected: `/teleop_ik_node` が表示され, `home_j1_rad`..`home_j6_rad` と `reset_duration_sec` が param list に出る.

- [ ] **Step 2: /teleop/reset サブスクライバの存在を確認**

```bash
nix develop .#ros --command bash -c "ros2 topic info /teleop/reset"
```

Expected: サブスクライバ型 `teleop_ik/msg/ResetCommand` が 1 件以上ある (起動直後なら 1 件).

- [ ] **Step 3: ノードを Ctrl+C で停止**

異常が無ければ Task 9 へ.

- [ ] **Step 4: コミットは不要 (確認のみ)**

---

## Task 9: SoArmVR 側 .msg ミラーを作成

**Files:**
- Create: `SoArmVR/Assets/Msgs/teleop_ik/msg/ResetCommand.msg`
- (ディレクトリもなければ作成: `SoArmVR/Assets/Msgs/teleop_ik/msg/`)

- [ ] **Step 1: ディレクトリが無ければ作成**

```bash
mkdir -p SoArmVR/Assets/Msgs/teleop_ik/msg
```

- [ ] **Step 2: .msg をミラー**

`SoArmVR/Assets/Msgs/teleop_ik/msg/ResetCommand.msg` を Task 2 と同内容で作成. ただし Unity 側の .meta ファイルも要るので Step 3 で対応.

ファイル内容は Task 2 と同じ.

- [ ] **Step 3: コミット**

```bash
git add SoArmVR/Assets/Msgs/teleop_ik/msg/ResetCommand.msg
git commit -m "feat(soarmvr): mirror ResetCommand.msg for rosettadds-genmsg"
```

> .meta ファイルは Unity Editor 起動時に自動生成される.
> CI フローでは Unity が走らないため手動で .meta を書く選択肢もあるが,
> 既存 .msg ファイルに .meta が無い/有るが混在しているため,
> ここでは生成は Unity 側で行う前提で進める.
> 万一 CI で Unity ビルドを走らせる場合は `nix develop .#soarmvr` で
> uloop 経由で再生成する.

---

## Task 10: C# バインディングを `rosettadds-genmsg` で再生成

**Files:**
- Create: `SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/TeleopIk/ResetCommandMessage.cs` 他
- Modify: (生成物のみ)

- [ ] **Step 1: rosettadds-genmsg の場所を確認**

```bash
nix develop .#soarmvr --command bash -c "which dotnet && dotnet --version"
```

Expected: `dotnet` のパスとバージョン (8.x) が表示される.

- [ ] **Step 2: `rosettadds-genmsg` のクローン (未取得なら)**

既存 spec `2026-06-13-soarmvr-rosettadds-migration-design.md` §3.2 に従い `ROSettaDDS` を clone. 既に取得済みならスキップ.

```bash
# 取得先 (例: ホームディレクトリや任意の workdir)
git clone https://github.com/ojii3/ROSettaDDS.git /tmp/ROSettaDDS
```

- [ ] **Step 3: genmsg を実行**

```bash
nix develop .#soarmvr --command bash -c "\
  dotnet run --project /tmp/ROSettaDDS/tools/rosettadds-genmsg -- \
    --input SoArmVR/Assets/Msgs \
    --output SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs"
```

Expected: `SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/TeleopIk/ResetCommandMessage.cs` 他が生成される.

- [ ] **Step 4: 生成物の中身を確認**

```bash
ls SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/TeleopIk/ | grep -i reset
```

Expected: `ResetCommandMessage.cs`, `ResetCommandMessageSerializer.cs` 等が生成されている.

- [ ] **Step 5: コミット**

```bash
git add SoArmVR/Assets/_SoArmVR/Scripts/GeneratedMsgs/
git commit -m "feat(soarmvr): regenerate C# bindings for ResetCommand"
```

---

## Task 11: SoArmTeleoperation.inputactions に Reset アクションを追加

**Files:**
- Modify: `SoArmVR/Assets/_SoArmVR/Input/SoArmTeleoperation.inputactions`

- [ ] **Step 1: 既存 inputactions を読む**

```bash
cat SoArmVR/Assets/_SoArmVR/Input/SoArmTeleoperation.inputactions
```

- [ ] **Step 2: actions 配列に Reset を追加**

既存 `Stick` アクションの直下に追加:

```json
                {
                    "name": "Reset",
                    "type": "Button",
                    "id": "<新規 UUID (uuidgen などで生成)>",
                    "expectedControlType": "Button",
                    "processors": "",
                    "interactions": "",
                    "initialStateCheck": true
                }
```

> UUID は `uuidgen | tr 'a-z' 'A-Z'` で生成 (ROS の流儀に揃える).

- [ ] **Step 3: bindings 配列に primaryButton バインドを追加**

既存 `Stick` バインドの直下に追加:

```json
                {
                    "name": "",
                    "id": "<新規 UUID>",
                    "path": "<XRController>{RightHand}/primaryButton",
                    "interactions": "",
                    "processors": "",
                    "groups": "",
                    "action": "Reset",
                    "isComposite": false,
                    "isPartOfComposite": false
                }
```

- [ ] **Step 4: JSON 妥当性チェック**

```bash
python3 -c "import json; json.load(open('SoArmVR/Assets/_SoArmVR/Input/SoArmTeleoperation.inputactions'))" && echo OK
```

Expected: `OK` が出力される.

- [ ] **Step 5: コミット**

```bash
git add SoArmVR/Assets/_SoArmVR/Input/SoArmTeleoperation.inputactions
git commit -m "feat(soarmvr): add Reset input action bound to right controller A button"
```

---

## Task 12: ITeleoperationSink に PublishReset() を追加

**Files:**
- Modify: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/ITeleoperationSink.cs`

- [ ] **Step 1: インターフェースを読む**

```bash
cat SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/ITeleoperationSink.cs
```

- [ ] **Step 2: PublishReset() を追加**

インターフェース本体に `OnSessionEnd()` の下あたりに追加:

```csharp
    void PublishReset();
```

- [ ] **Step 3: コミット**

```bash
git add SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/ITeleoperationSink.cs
git commit -m "refactor(soarmvr): add PublishReset to ITeleoperationSink"
```

---

## Task 13: DebugTeleoperationSink に空実装を追加

**Files:**
- Modify: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/DebugTeleoperationSink.cs`

- [ ] **Step 1: ファイル末尾に空実装を追加**

`DebugTeleoperationSink.cs` のクラス末尾 (`}` の直前) に追加:

```csharp
    public void PublishReset()
    {
        // Debug sink does not forward to ROS.
    }
```

- [ ] **Step 2: コミット**

```bash
git add SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/DebugTeleoperationSink.cs
git commit -m "refactor(soarmvr): add empty PublishReset to DebugTeleoperationSink"
```

---

## Task 14: RosTeleoperationSink に reset publisher を追加

**Files:**
- Modify: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/RosTeleoperationSink.cs`

- [ ] **Step 1: using ディレクティブに ResetCommandMessage を追加**

ファイル先頭の `using` ブロックに以下があるか確認し, 無ければ追加:

```csharp
using ROSettaDDS.Msgs.TeleopIk;
```

(Task 10 の genmsg で namespace が変わる場合は `using ROSettaDDS.Msgs.TeleopIk;` の代わりに実際の namespace に合わせる. 既存 spec の慣例に従う.)

- [ ] **Step 2: メンバ `_resetPub` を追加**

`Publisher<Float64Message> _gripperPub;` の下あたりに追加:

```csharp
        Publisher<ResetCommandMessage> _resetPub;
```

- [ ] **Step 3: `InitParticipant` 内で `_resetPub` を作成**

`_gripperPub = _participant.CreatePublisher<Float64Message>(...)` の下あたりに追加:

```csharp
            _resetPub = _participant.CreatePublisher<ResetCommandMessage>(
                "/teleop/reset",
                ResetCommandMessageSerializer.Instance,
                ReliabilityQos.Reliable,
                DurabilityQos.Volatile,
                ResetCommandMessage.DdsTypeName);
```

- [ ] **Step 4: `OnDestroy` で `_resetPub` を破棄**

`_gripperPub?.Dispose();` の下あたりに追加:

```csharp
            _resetPub?.Dispose();
```

`_resetPub = null;` も `OnDestroy` の null 代入列に追加.

- [ ] **Step 5: `PublishReset` 実装をクラス末尾に追加**

`PublishActive` メソッドの直下に追加:

```csharp
        public void PublishReset()
        {
            _ = PublishResetAsync();
        }

        async System.Threading.Tasks.Task PublishResetAsync()
        {
            if (_resetPub == null) return;
            var now = System.DateTimeOffset.UtcNow;
            var stamp = new RosTime((int)now.ToUnixTimeSeconds(), (uint)(now.Millisecond * 1_000_000));
            var msg = new ResetCommandMessage(
                new Header(stamp, "teleop_reset"),
                new float[] { float.NaN, float.NaN, float.NaN, float.NaN, float.NaN, float.NaN },
                0.0f
            );
            try
            {
                await _resetPub.PublishAsync(msg);
            }
            catch (System.ObjectDisposedException) { }
        }
```

> `ResetCommandMessage` のコンストラクタシグネチャは
> genmsg 出力に合わせる. 万一 `float[]` ではなく
> `System.Collections.Generic.List<float>` 等の場合は
> genmsg 出力に合わせて書き直す.

- [ ] **Step 6: コミット**

```bash
git add SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/RosTeleoperationSink.cs
git commit -m "feat(soarmvr): publish ResetCommand from RosTeleoperationSink on A button"
```

---

## Task 15: TeleoperationSession に rising edge 検出を追加

**Files:**
- Modify: `SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationSession.cs`

- [ ] **Step 1: `_resetAction` フィールドを追加**

`[SerializeField, Tooltip("...")] InputActionProperty _stickAction;` の下あたりに追加:

```csharp
        [SerializeField, Tooltip("リセット発火（右コントローラ A ボタン押下の瞬間に 1 回）")]
        InputActionProperty _resetAction;
```

- [ ] **Step 2: `OnEnable` / `OnDisable` に Enable/Disable を追加**

`_stickAction.action?.Enable();` の下 (`OnEnable`) に追加:

```csharp
            _resetAction.action?.Enable();
```

`OnDisable` の同じ位置にも:

```csharp
            _resetAction.action?.Disable();
```

- [ ] **Step 3: `_prev_reset_pressed_` メンバを追加**

`int _sampleId;` の下あたりに追加:

```csharp
        bool _prevResetPressed;
```

- [ ] **Step 4: `Update()` 内に rising edge 検出を追加**

`if (_active) PushSample();` の下あたりに追加:

```csharp
            bool reset_pressed = _resetAction.action != null && _resetAction.action.IsPressed();
            if (reset_pressed && !_prevResetPressed)
                _sink?.PublishReset();
            _prevResetPressed = reset_pressed;
```

- [ ] **Step 5: コミット**

```bash
git add SoArmVR/Assets/_SoArmVR/Scripts/Teleoperation/TeleoperationSession.cs
git commit -m "feat(soarmvr): trigger PublishReset on A button rising edge in TeleoperationSession"
```

---

## Task 16: Unity ビルド確認 (手動)

**Files:**
- (確認のみ, ファイル変更なし)

- [ ] **Step 1: uloop 経由で Unity ビルドが成功することを確認**

```bash
nix develop .#soarmvr --command bash -c "uloop build"
```

Expected: Unity ビルドが成功し, 生成 C# ファイルが asmdef から参照可能になっている.

> もし uloop 経由が難しい場合は
> `nix develop .#soarmvr` の中で直接 `dotnet build` や
> Unity Editor を開いて手動ビルドする.

- [ ] **Step 2: TeleoperationSession の Inspector 上で `_resetAction` が inputactions に結線されているか確認**

これは人手による Unity Editor での確認が必要. リポジトリ上の SerializeField には値が無いので, プレハブ/シーン編集が必要. 詳細は省略.

- [ ] **Step 3: コミットは不要 (確認のみ)**

---

## Task 17: 既存テスト影響確認 + 全テスト再実行

- [ ] **Step 1: 全テスト再実行**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon build --symlink-install --packages-select teleop_ik --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select teleop_ik --event-handlers console_direct+ 2>&1 | tail -60"
```

Expected: 既存テスト + 新規 10 件 (`test_reset_msg.cpp` 3 件 + `ResetFixture` 7 件) が全 PASS.

- [ ] **Step 2: launch テストも実行**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon test --packages-select teleop_ik --event-handlers console_direct+ --ctest-args -R launch 2>&1 | tail -20"
```

Expected: `test_vr_teleop_launch.py` が PASS (変更していないため影響なし).

- [ ] **Step 3: 結果を確認**

何か fail があれば直してコミット. 問題なければ Task 18 へ.

---

## Task 18: README 更新 (任意)

**Files:**
- Modify: `ros2_ws/README.md` (存在すれば)

- [ ] **Step 1: README にリセット機能の説明を追記**

無ければ「`/teleop/reset` topic on VR A button」と一行だけ追加. 詳細はこの spec / plan へのリンクで十分.

- [ ] **Step 2: コミット (変更があった場合のみ)**

```bash
git add ros2_ws/README.md
git commit -m "docs(ros2_ws): document /teleop/reset behavior"
```

---

## Task 19: PR 作成

- [ ] **Step 1: 変更内容を確認**

```bash
git log --oneline main..HEAD
git diff main..HEAD --stat
```

- [ ] **Step 2: リモートに push**

```bash
git push -u origin feat/vr-reset-command
```

- [ ] **Step 3: `gh` で PR 作成**

```bash
gh pr create \
  --title "feat(teleop_ik): VR reset command (IK bypass to home pose)" \
  --body "## 概要
VR コントローラ A ボタン押下で SO-101 を home 姿勢に戻すリセット機能を追加. IK バイパスで関節空間 setpoint を直接 publish.

## 変更点
- 新規メッセージ teleop_ik/msg/ResetCommand
- 新規トピック /teleop/reset (RELIABLE)
- teleop_ik_node: on_reset_msg / on_reset 実装
- teleop_ik_params.yaml: home_j1_rad..home_j6_rad + reset_duration_sec
- SoArmVR: inputactions に Reset, RosTeleoperationSink に publisher, TeleoperationSession で rising edge 検出
- C# バインディング再生成

## テスト
- test_reset_msg.cpp: 3 件
- test_ik_node_helpers.cpp ResetFixture: 7 件

## 関連
- spec: docs/superpowers/specs/2026-07-01-vr-reset-command-design.md
- plan: docs/superpowers/plans/2026-07-01-vr-reset-command.md"
```

Expected: PR URL が表示される. ユーザに URL を共有.

---

## 自己レビューチェックリスト (Plan 著者用)

実装着手前に Plan 著者 (= 私) が確認:

1. **Spec coverage:**
   - §3.2 ResetCommand 定義 → Task 2, 3, 9
   - §4.2 CMakeLists → Task 2
   - §4.3 ヘッダ宣言 → Task 4
   - §4.4 IK ノード実装 → Task 5, 6
   - §4.5 YAML → Task 7
   - §4.6 Unity .msg ミラー → Task 9
   - §4.7 inputactions → Task 11
   - §4.8 ITeleoperationSink → Task 12
   - §4.9 DebugSink → Task 13
   - §4.10 RosTeleoperationSink → Task 14
   - §4.11 TeleoperationSession → Task 15
   - §5.1 単体テスト → Task 3, 5
   - §5.4 手動検証 → Task 8, 16
   - 全項目に対応タスクあり.

2. **Placeholder scan:**
   - "TBD" / "TODO" / "fill in details" 等の表現なし. UUID のみプレースホルダだが, 実装時に `uuidgen` で生成する旨を明記.

3. **Type consistency:**
   - `on_reset_msg(const SharedPtr)` → `on_reset(const &)` → Task 4 ヘッダ, Task 6 実装で一致.
   - `ResetCommandMessage` の C# コンストラクタシグネチャは genmsg 出力次第. Task 14 Step 5 で `// genmsg 出力に合わせる` と注記.
   - `home_rad_` メンバは spec 通り追加しない (Task 6 で `get_parameter` 都度読み). ヘッダ宣言 (Task 4) にも追加しない.
   - `_prevResetPressed` (C#) と `_prev_reset_pressed_` (旧 spec) は Task 15 で `_prevResetPressed` に統一.
   - `make_arm_trajectory` / `make_gripper_trajectory` のシグネチャは既存関数をそのまま使う. Task 6 の `q_arm` は `model_.nq` 次元 (既存 `q_current_` と同じ流儀).
