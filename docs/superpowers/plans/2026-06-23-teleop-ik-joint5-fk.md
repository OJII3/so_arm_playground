# teleop_ik: 手首制御の分割 (IK 1-4, FK 5) 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `teleop_ik_node` の関節分担を変更する。`position_joint_ids_` を `[1,2,3]` から `[1,2,3,4]` に拡大し, `wrist_joint_ids_` を `[4,5]` から `[5]` に縮小する。`q_seed` での stick 由来上書きは joint 4 のみ継続し, joint 5 はソルバ外で `q_solution_` に注入する FK として残す。ROS インターフェース (トピック名・パラメータ・joint 名) は変えない。

**Architecture:**
- IK ソルバは 3D 位置ターゲットを joint 1-4 (4DOF, 1 自由度冗長) で解く CLIK. 既存のヤコビアン構築・積分の動的サイズ処理がそのまま使える.
- joint 5 はソルバが触らず, `on_target_with_input` 内で `q_solution_[idx_q_5] = wrist_init_pos_.x() + integrated_stick_.x()` として書き戻す.
- 既存の trajectory publish (`make_arm_trajectory`) は全 5 関節を `q_solution_` から読むため, joint 5 の FK 値がそのまま publish される.

**Tech Stack:** ROS 2 Jazzy, C++17, rclcpp, Pinocchio 4.0.0, gtest (ament_cmake_gtest), colcon, Nix (`nix develop .#ros`).

**Spec:** `docs/superpowers/specs/2026-06-23-teleop-ik-joint5-fk-design.md`

**参考:**
- 既存実装: `ros2_ws/src/teleop_ik/src/ik_node.cpp`
- 既存テスト: `ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp`
- C++ 化 spec/plan:
  - `docs/superpowers/specs/2026-06-23-teleop-ik-cpp-rewrite-design.md`
  - `docs/superpowers/plans/2026-06-23-teleop-ik-cpp-rewrite.md`

---

## File Structure

### Modified files
- `ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp` — `position_joint_ids_` を `std::array<..., 3>` → `std::array<..., 4>`, `wrist_joint_ids_` を `std::array<..., 2>` → `std::array<..., 1>`.
- `ros2_ws/src/teleop_ik/src/ik_node.cpp` — `init_ros_node` / `make_for_test` の joint id 設定, `on_active` の wrist 初期化パス, `on_target_with_input` の `q_seed` ロジックと `q_solution_` への joint 5 注入.
- `ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp` — テスト追加 (SolveIkAdjustsJoint4, SolveIkHasFourPositionJoints) と改訂 (SolveIkKeepsWristJointTargetsFixed → SolveIkKeepsJoint5Fixed, WristResetsOnSessionStart).

### Unchanged files
- `ros2_ws/src/teleop_ik/CMakeLists.txt` (テストのソース追加なしで対応可)
- `ros2_ws/src/teleop_ik/launch/*.py`
- `ros2_ws/src/teleop_ik/config/*.yaml`
- `ros2_ws/src/teleop_ik/msg/TargetPoseWithInput.msg`
- `ros2_ws/src/teleop_ik/src/gamepad_node.cpp` (stick_x/y の解釈は ik_node 側のみ)

---

## 事前準備

実装着手前に以下を確認すること.

- [ ] Linux ホストで `nix develop .#ros` が起動できること
- [ ] `ros2_ws/` で `colcon build --symlink-install --packages-select teleop_ik` が現状 (C++ 版) で成功すること
- [ ] `colcon test --packages-select teleop_ik` が全パスすること (ベースライン)

---

## Task 1: ブランチ作成

**Files:**
- Modify: git 状態

- [ ] **Step 1: フィーチャーブランチを作成して checkout**

```bash
git checkout -b feat/teleop-ik-joint5-fk
```

- [ ] **Step 2: ベースラインが通ることを確認**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon build --symlink-install --packages-select teleop_ik && colcon test --packages-select teleop_ik"
```

Expected: ビルドと全テストが成功する. 問題があればベースラインから直す.

- [ ] **Step 3: コミット (ブランチ作成のみ, コード変更なしの場合は空コミットは不要なので省略可)**

ベースラインが通れば Task 2 へ進む.

---

## Task 2: ヘッダの配列サイズ変更

**Files:**
- Modify: `ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp:89-91`

`position_joint_ids_` を 3 → 4, `wrist_joint_ids_` を 2 → 1 に拡大/縮小する.
これにより `ik_node.cpp` 側の `position_joint_ids_[i] = arm_joint_ids_[i]` (i=0..2) と
`wrist_joint_ids_[1] = arm_joint_ids_[4]` が範囲外となり, ビルドが壊れる.
Task 3 以降で修正する.

- [ ] **Step 1: ヘッダの該当行を編集**

`ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp:89-91` を以下に置換:

```cpp
  std::array<pinocchio::JointIndex, 5> arm_joint_ids_{};
  std::array<pinocchio::JointIndex, 4> position_joint_ids_{};
  std::array<pinocchio::JointIndex, 1> wrist_joint_ids_{};
```

- [ ] **Step 2: ビルドして失敗を確認**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon build --symlink-install --packages-select teleop_ik 2>&1 | tail -40"
```

Expected: `ik_node.cpp` の `position_joint_ids_[i] = arm_joint_ids_[i]` (i=2) は通るが
`wrist_joint_ids_[1] = arm_joint_ids_[4]` などで `array out of range` などの
コンパイルエラーが出る. エラー出力をそのまま次のタスクで確認する.

- [ ] **Step 3: コミット**

```bash
git add ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp
git commit -m "refactor(teleop_ik): resize position/wrist joint id arrays for IK 1-4 FK 5"
```

---

## Task 3: テスト追加と改訂 (TDD: 失敗するテストを書く)

**Files:**
- Modify: `ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp:129-147, 237-261`

新挙動を検証するテストを追加し, 旧テストを改訂する. このタスク時点では
まだ `init_ros_node` / `make_for_test` / `on_active` / `on_target_with_input` の
本体変更が未着手なので, 一部のテストは **失敗する** ことが期待値.

- [ ] **Step 1: 失敗するテスト `SolveIkHasFourPositionJoints` を追加**

`ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp` の末尾 (ファイル末尾の
`}` 直前, 314 行目付近) に追加:

```cpp
TEST_F(TeleopIKHelpersTest, SolveIkHasFourPositionJoints)
{
  // 新方式: position_joint_ids_ には joint 1, 2, 3, 4 が入る.
  EXPECT_EQ(node_->position_joint_ids_.size(), 4u);
}

TEST_F(TeleopIKHelpersTest, SolveIkAdjustsJoint4)
{
  // joint 4 は position_joint_ids_ に含まれているため, IK の seed 値から
  // ソルバが動かすことが許容される. ここでは position_joint_ids_ に
  // joint 4 が含まれていることだけ検証する.
  const auto jid_4 = node_->model_.getJointId("4");
  EXPECT_NE(
      std::find(
          node_->position_joint_ids_.begin(),
          node_->position_joint_ids_.end(),
          jid_4),
      node_->position_joint_ids_.end());

  // さらに, ソルバが joint 4 を変更しうることを確認するため, ターゲットを
  // 少しだけ動かして solve_ik を呼ぶ. joint 4 の seed 値はソルバの
  // 冗長 DOF 解決によって変化しうる (変化しなくても許容).
  pinocchio::forwardKinematics(node_->model_, node_->data_, node_->q_current_);
  pinocchio::updateFramePlacements(node_->model_, node_->data_);
  const Eigen::Vector3d current = node_->data_.oMf[node_->ee_frame_id_].translation();
  const Eigen::Vector3d target = current + Eigen::Vector3d(0.0, -0.01, 0.0);
  const auto idx_q_4 = node_->model_.joints[node_->model_.getJointId("4")].idx_q();
  Eigen::VectorXd seed = node_->q_current_;
  seed[idx_q_4] = 0.0;  // 現在値から意図的にずらす
  const auto result = node_->solve_ik(target, seed, 1e-6, 100, 1e-4);
  ASSERT_TRUE(result.has_value());
  // joint 4 が joints 制限内に収まっている.
  EXPECT_GE((*result)[idx_q_4], node_->model_.lowerPositionLimit[idx_q_4] - 1e-9);
  EXPECT_LE((*result)[idx_q_4], node_->model_.upperPositionLimit[idx_q_4] + 1e-9);
}
```

> `<algorithm>` の `std::find` を使うので, ファイル先頭のインクルードに
> `<algorithm>` が無ければ追加. 既存ヘッダ (line 1-19) を確認の上,
> 無ければ `#include <algorithm>` を `<array>` の下あたりに足す.

- [ ] **Step 2: 旧テスト `SolveIkKeepsWristJointTargetsFixed` を `SolveIkKeepsJoint5Fixed` に改訂**

`ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp:129-147` を以下に置換:

```cpp
TEST_F(TeleopIKHelpersTest, SolveIkKeepsJoint5Fixed)
{
  // position joint 1〜3 の seed を変えたが, joint 5 はソルバの対象外
  // (FK 制御) なので seed 値が保持される.
  // 一方 joint 4 は新方式で position_joint_ids_ に含まれているため,
  // ソルバの冗長 DOF 解決で動きうる. ここでは joint 5 のみ検証する.
  Eigen::VectorXd seed = node_->q_current_;
  const auto idx_q_4 = node_->model_.joints[node_->model_.getJointId("4")].idx_q();
  const auto idx_q_5 = node_->model_.joints[node_->model_.getJointId("5")].idx_q();
  seed[idx_q_4] = 0.2;
  seed[idx_q_5] = -0.3;
  pinocchio::forwardKinematics(node_->model_, node_->data_, seed);
  pinocchio::updateFramePlacements(node_->model_, node_->data_);
  const Eigen::Vector3d target = node_->data_.oMf[node_->ee_frame_id_].translation() +
    Eigen::Vector3d(0.0, -0.005, 0.0);
  const auto result = node_->solve_ik(target, seed, 1e-6, 100, 1e-4);
  ASSERT_TRUE(result.has_value());
  // joint 5 はソルバ外なので seed 値が保持される.
  EXPECT_NEAR((*result)[idx_q_5], -0.3, 1e-9);
  // 参考: joint 4 はソルバ内だが, このテストでは特に値を固定しない.
}
```

- [ ] **Step 3: ビルドしてテストが失敗することを確認**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon build --symlink-install --packages-select teleop_ik --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -20"
```

Expected: ビルドは Task 2 完了時点と同じ理由でまだ失敗する.
`ik_node.cpp` の修正は Task 4 以降.

- [ ] **Step 4: テストファイルだけコミット**

```bash
git add ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp
git commit -m "test(teleop_ik): add TDD tests for IK 1-4 FK 5 split (red)"
```

---

## Task 4: `init_ros_node` と `make_for_test` を新サイズに合わせる

**Files:**
- Modify: `ros2_ws/src/teleop_ik/src/ik_node.cpp:151-163, 221-233`

`position_joint_ids_` を 4 要素 (joint 1-4), `wrist_joint_ids_` を 1 要素 (joint 5) にする.

- [ ] **Step 1: `init_ros_node` 内の joint id 設定を編集**

`ros2_ws/src/teleop_ik/src/ik_node.cpp:151-163` を以下に置換:

```cpp
  for (size_t i = 0; i < 5; ++i) {
    const std::string name = std::to_string(i + 1);
    if (!model_.existJointName(name)) {
      RCLCPP_FATAL(get_logger(), "Joint '%s' not found in URDF", name.c_str());
      throw std::runtime_error("Joint '" + name + "' not found in URDF");
    }
    arm_joint_ids_[i] = model_.getJointId(name);
  }
  // IK ソルバが触れる関節: joint 1, 2, 3, 4 (4DOF, 3D 位置ターゲットで 1 自由度冗長).
  for (size_t i = 0; i < 4; ++i) {
    position_joint_ids_[i] = arm_joint_ids_[i];
  }
  // FK 制御の関節: joint 5 のみ (stick_x の速度積分).
  wrist_joint_ids_[0] = arm_joint_ids_[4];
```

- [ ] **Step 2: `make_for_test` の joint id 設定を編集**

`ros2_ws/src/teleop_ik/src/ik_node.cpp:221-233` を以下に置換:

```cpp
  for (size_t i = 0; i < 5; ++i) {
    const std::string name = std::to_string(i + 1);
    if (node->model_.existJointName(name)) {
      node->arm_joint_ids_[i] = node->model_.getJointId(name);
    } else {
      node->arm_joint_ids_[i] = static_cast<pinocchio::JointIndex>(-1);
    }
  }
  // IK ソルバ対象: joint 1, 2, 3, 4. FK: joint 5.
  for (size_t i = 0; i < 4; ++i) {
    node->position_joint_ids_[i] = node->arm_joint_ids_[i];
  }
  node->wrist_joint_ids_[0] = node->arm_joint_ids_[4];
```

- [ ] **Step 3: ビルド成功を確認**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon build --symlink-install --packages-select teleop_ik --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -20"
```

Expected: ビルド成功 (`array out of range` などのエラーが消える).

- [ ] **Step 4: テスト実行 — `SolveIkHasFourPositionJoints` が成功することを確認**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon test --packages-select teleop_ik --event-handlers console_direct+ 2>&1 | tail -40"
```

Expected:
- `SolveIkHasFourPositionJoints` ✓ PASS
- `SolveIkKeepsJoint5Fixed` ✓ PASS
- `SolveIkAdjustsJoint4` ✓ PASS
- 既存テスト (`SolveIkReturnsNulloptForUnreachableTarget` ほか) ✓ PASS
- `WristResetsOnSessionStart` はまだ旧パス (`wrist_joint_ids_[0]` を joint 4 として扱う) なので **失敗** することが期待値. Task 5 で直す.

- [ ] **Step 5: コミット**

```bash
git add ros2_ws/src/teleop_ik/src/ik_node.cpp
git commit -m "refactor(teleop_ik): expand position_joint_ids_ to joints 1-4, shrink wrist to joint 5"
```

---

## Task 5: `on_active` の wrist 初期化パスを新経路に

**Files:**
- Modify: `ros2_ws/src/teleop_ik/src/ik_node.cpp:316-330`

`on_active(true)` で `wrist_init_pos_` を取り出す際,
旧: `wrist_joint_ids_[0]` → joint 4, `wrist_joint_ids_[1]` → joint 5.
新: `arm_joint_ids_[3]` → joint 4, `wrist_joint_ids_[0]` → joint 5.

- [ ] **Step 1: `on_active` 内の wrist 初期化を編集**

`ros2_ws/src/teleop_ik/src/ik_node.cpp:316-330` を以下に置換:

```cpp
      // wrist 初期角 = q_current_ の joint 4/5 値. 関節 4 → stick_y
      // (wrist_init_pos.y()), 関節 5 → stick_x (wrist_init_pos.x())
      // という軸マッピングは on_target_with_input と同じ.
      // joint 4 は arm_joint_ids_[3], joint 5 は wrist_joint_ids_[0] (= arm_joint_ids_[4]).
      wrist_init_pos_.setZero();
      const auto jid_4 = arm_joint_ids_[3];
      if (jid_4 != static_cast<pinocchio::JointIndex>(-1)) {
        const auto idx_q_4 = model_.joints[jid_4].idx_q();
        if (idx_q_4 >= 0 && static_cast<Eigen::Index>(idx_q_4) < q_current_.size()) {
          wrist_init_pos_.y() = q_current_[idx_q_4];  // stick_y → joint 4
        }
      }
      if (wrist_joint_ids_[0] != static_cast<pinocchio::JointIndex>(-1)) {
        const auto idx_q_5 = model_.joints[wrist_joint_ids_[0]].idx_q();
        if (idx_q_5 >= 0 && static_cast<Eigen::Index>(idx_q_5) < q_current_.size()) {
          wrist_init_pos_.x() = q_current_[idx_q_5];  // stick_x → joint 5
        }
      }
```

- [ ] **Step 2: テスト実行 — `WristResetsOnSessionStart` が成功することを確認**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon build --symlink-install --packages-select teleop_ik --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -10 && colcon test --packages-select teleop_ik --event-handlers console_direct+ 2>&1 | tail -30"
```

Expected: `WristResetsOnSessionStart` ✓ PASS (新経路で joint 4 を
`arm_joint_ids_[3]` から, joint 5 を `wrist_joint_ids_[0]` から取得).

- [ ] **Step 3: コミット**

```bash
git add ros2_ws/src/teleop_ik/src/ik_node.cpp
git commit -m "refactor(teleop_ik): route wrist init through arm_joint_ids_[3] and wrist_joint_ids_[0]"
```

---

## Task 6: `on_target_with_input` で joint 5 を FK 注入

**Files:**
- Modify: `ros2_ws/src/teleop_ik/src/ik_node.cpp:411-428`

`q_seed[idx_q_5]` への上書きを削除し, IK 収束後に
`q_solution_[idx_q_5] = wrist_init_pos_.x() + integrated_stick_.x()` を
書き戻す. これで `make_arm_trajectory` が joint 5 の FK 値を publish する.

- [ ] **Step 1: q_seed ロジックと IK 結果処理を編集**

`ros2_ws/src/teleop_ik/src/ik_node.cpp:411-428` を以下に置換:

```cpp
  Eigen::VectorXd q_seed = q_solution_;
  // joint 4 は IK ソルバの冗長 DOF. stick_y 積分値を seed の bias として渡す.
  if (arm_joint_ids_[3] != static_cast<pinocchio::JointIndex>(-1)) {
    const auto idx_q_4 = model_.joints[arm_joint_ids_[3]].idx_q();
    q_seed[idx_q_4] = wrist_init_pos_.y() + integrated_stick_.y();
  }
  // joint 5 は FK (position_joint_ids_ に含めない) なので q_seed で
  // 上書きしない. q_solution_ からの前回値 (= 直近の stick 積分値) を保持.
  q_seed = clamp_joints(q_seed);

  if (auto result = solve_ik(target_pos, q_seed, ik_damping, ik_max_iterations, ik_tolerance);
      result.has_value()) {
    q_solution_ = *result;
    // joint 5 はソルバ外. stick_x 由来の FK 値を q_solution_ に注入して
    // make_arm_trajectory がそのまま publish できるようにする.
    if (wrist_joint_ids_[0] != static_cast<pinocchio::JointIndex>(-1)) {
      const auto idx_q_5 = model_.joints[wrist_joint_ids_[0]].idx_q();
      q_solution_[idx_q_5] = wrist_init_pos_.x() + integrated_stick_.x();
    }
    return true;
  }
  // IK 失敗: 古い q_solution_ を維持し, publish しない.
  return false;
```

- [ ] **Step 2: ビルドして全テスト実行**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon build --symlink-install --packages-select teleop_ik --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -10 && colcon test --packages-select teleop_ik --event-handlers console_direct+ 2>&1 | tail -40"
```

Expected: 全テスト PASS. 特に:
- `OnTargetWithInputIntegratesStickPerMessage` — stick 積分は継続
- `SolveIkKeepsJoint5Fixed` — joint 5 が保持される
- `SolveIkAdjustsJoint4` — joint 4 が position_joint_ids_ に含まれ, ソルバが動く
- `WristResetsOnSessionStart` — 新経路で wrist_init_pos_ がセットされる
- `TargetUsesPreviousSuccessfulSolutionAsNextSeed` — q_solution_ の seed 連鎖が機能

- [ ] **Step 3: コミット**

```bash
git add ros2_ws/src/teleop_ik/src/ik_node.cpp
git commit -m "feat(teleop_ik): inject FK value for joint 5 after IK converges"
```

---

## Task 7: `WristResetsOnSessionStart` テストの期待値を確認

**Files:**
- Modify: `ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp:237-261` (確認のみ)

新方式でも `wrist_init_pos_.y()` (joint 4) と `wrist_init_pos_.x()` (joint 5)
の両方が `q_current_` から正しくセットされることを検証するテスト.
Task 5 で実装は完了しているので, テストが期待通り動作しているかを
確認するだけで OK. テストが落ちていればアサーションを修正する.

- [ ] **Step 1: テストコードを読む**

`ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp:237-261` を開き,
以下のアサーションが含まれていることを確認:

```cpp
EXPECT_NEAR(node_->wrist_init_pos_.y(), 0.3, 1e-9);  // joint 4 (stick_y)
EXPECT_NEAR(node_->wrist_init_pos_.x(), -0.4, 1e-9); // joint 5 (stick_x)
```

新方式でも joint 4 (stick_y) と joint 5 (stick_x) の両方がリセットされる
ことが期待値. もし期待値のコメントが古いまま (`wrist_joint_ids_[0] = 4,
wrist_joint_ids_[1] = 5` のような記述) なら, 以下のコメントに書き換える:

```cpp
  // wrist_init_pos_ は on_target_with_input のスティック軸マッピングと整合.
  // joint 4 (stick_y) → wrist_init_pos_.y()  … arm_joint_ids_[3] 経由
  // joint 5 (stick_x) → wrist_init_pos_.x()  … wrist_joint_ids_[0] (= arm_joint_ids_[4]) 経由
```

- [ ] **Step 2: テストが既に PASS していることを確認**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon test --packages-select teleop_ik --event-handlers console_direct+ --ctest-args -R WristResetsOnSessionStart 2>&1 | tail -20"
```

Expected: `WristResetsOnSessionStart` ✓ PASS.

- [ ] **Step 3: コメントを修正した場合はコミット**

```bash
git add ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp
git commit -m "docs(teleop_ik): update wrist init test comments for new joint paths"
```

修正が無ければこのコミットは不要.

---

## Task 8: 全テスト・lint の実行

**Files:**
- (なし, 検証のみ)

- [ ] **Step 1: ビルドキャッシュをクリアしてクリーンビルド**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && rm -rf build/teleop_ik install/teleop_ik && colcon build --symlink-install --packages-select teleop_ik --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -20"
```

Expected: 警告なしでビルド成功.

- [ ] **Step 2: 全テスト実行**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon test --packages-select teleop_ik --event-handlers console_direct+ 2>&1 | tail -40"
```

Expected: 全テスト PASS.

- [ ] **Step 3: ament lint (任意だが推奨)**

```bash
nix develop .#ros --command bash -c "cd ros2_ws && colcon lint --packages-select teleop_ik 2>&1 | tail -20"
```

Expected: 警告なし (既存と同等). 問題がなければ次に進む.

- [ ] **Step 4: git status で未コミット変更がないことを確認**

```bash
git status
```

Expected: `nothing to commit, working tree clean`.

---

## Task 9: PR 作成

**Files:**
- (なし, 検証のみ)

- [ ] **Step 1: リモートに push**

```bash
git push -u origin feat/teleop-ik-joint5-fk
```

- [ ] **Step 2: PR を作成**

```bash
gh pr create \
  --base main \
  --title "teleop_ik: split wrist control — IK for joints 1-4, FK for joint 5" \
  --body "## 概要

\`teleop_ik_node\` の関節分担を変更. \`position_joint_ids_\` を [1,2,3] から [1,2,3,4] に拡大, \`wrist_joint_ids_\` を [4,5] から [5] に縮小.

## 変更内容

- joint 1, 2, 3, 4 を IK ソルバの対象 (4DOF, 3D 位置ターゲットで 1 自由度冗長)
- joint 5 のみ FK として stick_x 速度積分で直接制御
- stick_y 由来の joint 4 制御は q_seed の bias として継続
- ROS インターフェース (トピック名・パラメータ・joint 名) は変えない

## テスト

- \`colcon test --packages-select teleop_ik\` 全パス
- 新規テスト: \`SolveIkAdjustsJoint4\`, \`SolveIkHasFourPositionJoints\`
- 改訂テスト: \`SolveIkKeepsWristJointTargetsFixed\` → \`SolveIkKeepsJoint5Fixed\`

## 関連

- spec: docs/superpowers/specs/2026-06-23-teleop-ik-joint5-fk-design.md
- plan: docs/superpowers/plans/2026-06-23-teleop-ik-joint5-fk.md
"
```

- [ ] **Step 3: PR URL をユーザーに報告**

完了.

---

## 自己レビュー (実装着手前に確認)

実装計画が spec をカバーしているか:

- [x] §3.1 ソルバ構成 (`position_joint_ids_` 4, `wrist_joint_ids_` 1) — Task 2, 4
- [x] §3.2 q_seed 組み立て (joint 4 のみ上書き) — Task 6
- [x] §3.3 IK 結果への joint 5 注入 — Task 6
- [x] §3.4 セッション開始時 wrist 初期化 — Task 5
- [x] §4.1 ヘッダサイズ変更 — Task 2
- [x] §4.2 `init_ros_node` / `make_for_test` — Task 4
- [x] §4.3 テスト改訂・追加 — Task 3, 7
- [x] §5 テスト戦略 — Task 3, 6, 7, 8
- [x] §6 マイルストーン — Task 1-8 で順次達成
- [x] §7 リスク — コメント・テスト名で対応
