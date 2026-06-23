# teleop_ik: Python → C++ 移植 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `ros2_ws/src/teleop_ik/` の 2 ノード (`teleop_ik_node` / `gamepad_teleop_node`) を Python から C++ (`rclcpp` + Pinocchio) に書き換え、ROS 2 インターフェース契約 (ノード名・トピック・パラメータ) を完全互換に保つ。Python 版は完全削除する。テストは gtest に全面移植する。

**Architecture:**
- `coordinate_utils` を静的 lib ターゲットとして切り出し、純粋関数のテスト対象にする。
- `ik_node` / `gamepad_teleop_node` の 2 実行ファイルを `rclcpp` で実装。Pinocchio C++ 4.0.0 (`pkgs.pinocchio`) を使って IK を解く。
- xacro 処理は Python 版と等価に `xacro` CLI を `popen` で起動して URDF XML を取得する。
- ROS インターフェース契約 (ノード名・トピック名・パラメータ名・型・デフォルト値) は Python 版と 1:1 で維持し、launch / YAML 設定は無変更で動作する。

**Tech Stack:** ROS 2 Jazzy, C++17, ament_cmake, rclcpp, Eigen3, Pinocchio 4.0.0 (nixpkgs), gtest (ament_cmake_gtest), xacro CLI, Nix (nix-ros-overlay), colcon.

**Spec:** `docs/superpowers/specs/2026-06-23-teleop-ik-cpp-rewrite-design.md`

**参考:**
- 既存 C++ ノード実装: `ros2_ws/src/feetech_ros2_driver/src/feetech_calibration_node.cpp`
- 既存 Python 実装: `ros2_ws/src/teleop_ik/teleop_ik/{ik_node,gamepad_node,coordinate_utils}.py`
- 既存 Python テスト: `ros2_ws/src/teleop_ik/test/*.py`
- Nix 開発シェル: `ros2_ws/nix/shell.nix`

---

## File Structure

### New files (C++ 実装)
- `ros2_ws/src/teleop_ik/include/teleop_ik/visibility_control.hpp` — ament 慣例の export マクロ (boilerplate)
- `ros2_ws/src/teleop_ik/include/teleop_ik/coordinate_utils.hpp` — Unity↔ROS 座標変換の宣言
- `ros2_ws/src/teleop_ik/src/coordinate_utils.cpp` — 同上実装 (lib ターゲット)
- `ros2_ws/src/teleop_ik/src/ik_node.cpp` — `TeleopIKNode` 実装 (executable)
- `ros2_ws/src/teleop_ik/src/gamepad_node.cpp` — `GamepadTeleopNode` 実装 (executable)

### New files (C++ テスト)
- `ros2_ws/src/teleop_ik/test/test_coordinate_utils.cpp` — gtest
- `ros2_ws/src/teleop_ik/test/test_target_msg.cpp` — gtest
- `ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp` — gtest (実 URDF ベース IK テストもここに集約)
- `ros2_ws/src/teleop_ik/test/test_qos.cpp` — gtest + rclcpp
- `ros2_ws/src/teleop_ik/test/test_packaging.cpp` — gtest (install 成果物検査)

### Modified files
- `ros2_ws/src/teleop_ik/CMakeLists.txt` — C++ ビルドに全面書き換え
- `ros2_ws/src/teleop_ik/package.xml` — C++ 依存に更新 (`rclpy` / `python3-pytest` 削除、`rclcpp` / `pinocchio` / `eigen` / `ament_cmake_gtest` 追加)
- `ros2_ws/nix/shell.nix` — `pinocchio` を追加

### Deleted files (Python 版)
- `ros2_ws/src/teleop_ik/teleop_ik/__init__.py`
- `ros2_ws/src/teleop_ik/teleop_ik/ik_node.py`
- `ros2_ws/src/teleop_ik/teleop_ik/gamepad_node.py`
- `ros2_ws/src/teleop_ik/teleop_ik/coordinate_utils.py`
- `ros2_ws/src/teleop_ik/test/test_coordinate_utils.py`
- `ros2_ws/src/teleop_ik/test/test_ik_node.py`
- `ros2_ws/src/teleop_ik/test/test_target_msg.py`
- `ros2_ws/src/teleop_ik/test/test_qos.py`
- `ros2_ws/src/teleop_ik/test/test_packaging.py`
- `ros2_ws/src/teleop_ik/setup.py` (もし存在すれば)

### Unchanged files
- `ros2_ws/src/teleop_ik/msg/TargetPoseWithInput.msg`
- `ros2_ws/src/teleop_ik/launch/*.py` (3 ファイル)
- `ros2_ws/src/teleop_ik/config/*.yaml` (2 ファイル)
- `ros2_ws/src/teleop_ik/test/test_vr_teleop_launch.py` (Python launch に対するテストなので残す)

---

## 事前準備

実装着手前に以下を確認すること.

- [ ] Linux ホストで `nix develop .#ros` が起動できること (macOS では不可)
- [ ] `ros2_ws/` で `colcon build --symlink-install` が現状 (Python 版) で成功すること
- [ ] `python3Packages.pinocchio` の使用箇所を `grep -r pinocchio ros2_ws/src --include='*.xml'` で確認し、他パッケージで使われていなければ Nix から削除予定 (Task 14)

---

## Task 1: ブランチ作成 & 作業準備

**Files:**
- Modify: git 状態

- [ ] **Step 1: feat ブランチを切る**

```bash
git checkout main
git pull --ff-only
git checkout -b feat/teleop-ik-cpp
```

- [ ] **Step 2: 事前確認コマンドの実行**

```bash
cd ros2_ws
nix develop --command bash -c "colcon build --symlink-install"
```
期待: 全パッケージがビルド成功 (現状は Python 版).

- [ ] **Step 3: コミットなし・差分なしを確認**

```bash
git status
```
期待: `nothing to commit, working tree clean`.

---

## Task 2: Nix 開発シェルに Pinocchio C++ を追加

**Files:**
- Modify: `ros2_ws/nix/shell.nix`

- [ ] **Step 1: `shell.nix` を開く**

```bash
$EDITOR ros2_ws/nix/shell.nix
```

- [ ] **Step 2: `pinocchio` を追加**

47 行目あたり (`python3Packages.coal` の前) の `with extraPkgs; [` ブロック内に以下を追加:

```nix
          pinocchio
```

追加後の周辺は次のようになる:

```nix
          joy
          python3Packages.coal
          python3Packages.pinocchio
          pinocchio
        ]
```

- [ ] **Step 3: 評価が通ることを確認**

```bash
nix flake show --json 2>/dev/null | head -100
```
期待: エラーなし (`devShells.x86_64-linux.ros` まで評価できる).

- [ ] **Step 4: 起動して `pinocchio` ライブラリがリンク可能か確認**

```bash
cd ros2_ws
nix develop --command bash -c 'pkg-config --modversion pinocchio'
```
期待: `4.0.0` のようなバージョン文字列が出力される.

- [ ] **Step 5: コミット**

```bash
git add ros2_ws/nix/shell.nix
git commit -m "build(nix): add pinocchio C++ to ros dev shell"
```

---

## Task 3: include/teleop_ik/visibility_control.hpp を追加 (boilerplate)

**Files:**
- Create: `ros2_ws/src/teleop_ik/include/teleop_ik/visibility_control.hpp`

- [ ] **Step 1: ディレクトリ作成**

```bash
mkdir -p ros2_ws/src/teleop_ik/include/teleop_ik
```

- [ ] **Step 2: ファイル作成**

```cpp
// teleop_ik/visibility_control.hpp
// ament_cmake 慣例の export マクロ boilerplate.

#ifndef TELEOP_IK__VISIBILITY_CONTROL_HPP_
#define TELEOP_IK__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TELEOP_IK_EXPORT __attribute__((dllexport))
    #define TELEOP_IK_IMPORT __attribute__((dllimport))
  #else
    #define TELEOP_IK_EXPORT __declspec(dllexport)
    #define TELEOP_IK_IMPORT __declspec(dllimport)
  #endif
  #ifdef TELEOP_IK_BUILDING_DLL
    #define TELEOP_IK_PUBLIC TELEOP_IK_EXPORT
  #else
    #define TELEOP_IK_PUBLIC TELEOP_IK_IMPORT
  #endif
  #define TELEOP_IK_PUBLIC_TYPE TELEOP_IK_PUBLIC
  #define TELEOP_IK_LOCAL
#else
  #define TELEOP_IK_EXPORT __attribute__((visibility("default")))
  #define TELEOP_IK_IMPORT
  #if __GNUC__ >= 4
    #define TELEOP_IK_PUBLIC __attribute__((visibility("default")))
    #define TELEOP_IK_LOCAL __attribute__((visibility("hidden")))
  #else
    #define TELEOP_IK_PUBLIC
    #define TELEOP_IK_LOCAL
  #endif
  #define TELEOP_IK_PUBLIC_TYPE
#endif

#endif  // TELEOP_IK__VISIBILITY_CONTROL_HPP_
```

- [ ] **Step 3: コミット**

```bash
git add ros2_ws/src/teleop_ik/include/teleop_ik/visibility_control.hpp
git commit -m "feat(teleop_ik): add ament visibility_control header"
```

---

## Task 4: TDD — `unity_position_to_ros` / `unity_quaternion_to_ros` テスト先行

**Files:**
- Create: `ros2_ws/src/teleop_ik/test/test_coordinate_utils.cpp`

- [ ] **Step 1: テストファイル作成**

```cpp
// teleop_ik/test/test_coordinate_utils.cpp
#include <gtest/gtest.h>

#include "teleop_ik/coordinate_utils.hpp"

TEST(UnityPositionToRos, MapsAxesUnityToRos)
{
  const Eigen::Vector3d ros = teleop_ik::unity_position_to_ros(1.0, 2.0, 3.0, 1.0);
  EXPECT_DOUBLE_EQ(ros.x(), 3.0);
  EXPECT_DOUBLE_EQ(ros.y(), -1.0);
  EXPECT_DOUBLE_EQ(ros.z(), 2.0);
}

TEST(UnityPositionToRos, AppliesScale)
{
  const Eigen::Vector3d ros = teleop_ik::unity_position_to_ros(1.0, 2.0, 3.0, 2.0);
  EXPECT_DOUBLE_EQ(ros.x(), 6.0);
  EXPECT_DOUBLE_EQ(ros.y(), -2.0);
  EXPECT_DOUBLE_EQ(ros.z(), 4.0);
}

TEST(UnityPositionToRos, ZeroInputGivesZero)
{
  const Eigen::Vector3d ros = teleop_ik::unity_position_to_ros(0.0, 0.0, 0.0, 1.0);
  EXPECT_DOUBLE_EQ(ros.x(), 0.0);
  EXPECT_DOUBLE_EQ(ros.y(), 0.0);
  EXPECT_DOUBLE_EQ(ros.z(), 0.0);
}

TEST(UnityQuaternionToRos, FlipsWForHandedness)
{
  const Eigen::Vector4d ros_q = teleop_ik::unity_quaternion_to_ros(0.0, 0.0, 0.0, 1.0);
  EXPECT_DOUBLE_EQ(ros_q.x(), 0.0);
  EXPECT_DOUBLE_EQ(ros_q.y(), 0.0);
  EXPECT_DOUBLE_EQ(ros_q.z(), 0.0);
  EXPECT_DOUBLE_EQ(ros_q.w(), -1.0);
}

TEST(UnityQuaternionToRos, MapsVectorPartLikePosition)
{
  const Eigen::Vector4d ros_q = teleop_ik::unity_quaternion_to_ros(1.0, 2.0, 3.0, 0.5);
  EXPECT_DOUBLE_EQ(ros_q.x(), 3.0);
  EXPECT_DOUBLE_EQ(ros_q.y(), -1.0);
  EXPECT_DOUBLE_EQ(ros_q.z(), 2.0);
  EXPECT_DOUBLE_EQ(ros_q.w(), -0.5);
}
```

- [ ] **Step 2: コミット (テスト先行)**

```bash
git add ros2_ws/src/teleop_ik/test/test_coordinate_utils.cpp
git commit -m "test(teleop_ik): add coordinate_utils gtest (failing)"
```

> ビルドはまだ通らない (`coordinate_utils.hpp` が未存在). 次の Task 5 で実装する.

---

## Task 5: `coordinate_utils.hpp` / `coordinate_utils.cpp` を実装 (テスト緑化)

**Files:**
- Create: `ros2_ws/src/teleop_ik/include/teleop_ik/coordinate_utils.hpp`
- Create: `ros2_ws/src/teleop_ik/src/coordinate_utils.cpp`

- [ ] **Step 1: ヘッダ作成**

```cpp
// teleop_ik/include/teleop_ik/coordinate_utils.hpp
#ifndef TELEOP_IK__COORDINATE_UTILS_HPP_
#define TELEOP_IK__COORDINATE_UTILS_HPP_

#include <Eigen/Core>

namespace teleop_ik
{

// Unity (left-handed Y-up, X-right Z-forward) → ROS (right-handed Z-up,
// X-forward Y-left).
// ros_x =  unity_z
// ros_y = -unity_x
// ros_z =  unity_y
Eigen::Vector3d unity_position_to_ros(double x, double y, double z, double scale = 1.0);

// Vector part follows the same axis mapping. w is flipped for handedness.
Eigen::Vector4d unity_quaternion_to_ros(double x, double y, double z, double w);

}  // namespace teleop_ik

#endif  // TELEOP_IK__COORDINATE_UTILS_HPP_
```

- [ ] **Step 2: 実装作成**

```cpp
// teleop_ik/src/coordinate_utils.cpp
#include "teleop_ik/coordinate_utils.hpp"

namespace teleop_ik
{

Eigen::Vector3d unity_position_to_ros(double x, double y, double z, double scale)
{
  return Eigen::Vector3d(z * scale, -x * scale, y * scale);
}

Eigen::Vector4d unity_quaternion_to_ros(double x, double y, double z, double w)
{
  Eigen::Vector4d q;
  q.x() = z;
  q.y() = -x;
  q.z() = y;
  q.w() = -w;
  return q;
}

}  // namespace teleop_ik
```

- [ ] **Step 3: コミット**

```bash
git add ros2_ws/src/teleop_ik/include/teleop_ik/coordinate_utils.hpp \
        ros2_ws/src/teleop_ik/src/coordinate_utils.cpp
git commit -m "feat(teleop_ik): implement coordinate_utils (Unity <-> ROS)"
```

---

## Task 6: `teleop_ik_core` lib ターゲットのみの CMakeLists 暫定版

Python 版を壊さず C++ lib を併存させるため、CMakeLists.txt に
`coordinate_utils` だけビルドするセクションを追加する.
(後続の Task で ik_node / gamepad_node を順次足し込み、最終的に Python
ターゲットを削除する.)

**Files:**
- Modify: `ros2_ws/src/teleop_ik/CMakeLists.txt`

- [ ] **Step 1: 現状の CMakeLists.txt の末尾 (`ament_package()` の直前) に C++ セクションを挿入**

現状:
```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_target_msg
    test/test_target_msg.py
    APPEND_ENV "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${python_site_packages}"
  )
  ...
endif()

ament_package()
```

これを以下に置換 (`BUILD_TESTING` ブロックは一旦そのまま残し、 C++ テストを別ブロックで追加する):

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_target_msg
    test/test_target_msg.py
    APPEND_ENV "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${python_site_packages}"
  )
  ament_add_pytest_test(test_coordinate_utils
    test/test_coordinate_utils.py
    APPEND_ENV "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${python_site_packages}"
  )
  ament_add_pytest_test(test_ik_node
    test/test_ik_node.py
    APPEND_ENV "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${python_site_packages}"
  )
  ament_add_pytest_test(test_packaging
    test/test_packaging.py
    APPEND_ENV "TELEOP_IK_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}"
  )
endif()

# --- C++ implementation (added incrementally) ---
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

add_library(teleop_ik_core SHARED
  src/coordinate_utils.cpp
)
target_include_directories(teleop_ik_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(teleop_ik_core Eigen3)
target_link_libraries(teleop_ik_core pinocchio::pinocchio)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_coordinate_utils_cpp test/test_coordinate_utils.cpp)
  target_link_libraries(test_coordinate_utils_cpp teleop_ik_core)
endif()

install(TARGETS teleop_ik_core
  EXPORT export_teleop_ikTargets
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_package()
```

- [ ] **Step 2: package.xml に C++ 依存を追加 (Python 依存は残す)**

`ros2_ws/src/teleop_ik/package.xml` を開き、`<exec_depend>rclpy</exec_depend>` の直後に以下を追加:

```xml
  <depend>eigen</depend>
  <depend>pinocchio</depend>
  <depend>rclcpp</depend>
  <test_depend>ament_cmake_gtest</test_depend>
```

- [ ] **Step 3: ビルド**

```bash
cd ros2_ws
nix develop --command bash -c "colcon build --packages-select teleop_ik --symlink-install"
```

期待: `teleop_ik_core` ライブラリ + `test_coordinate_utils_cpp` gtest がビルドされる. 警告があれば解消.

- [ ] **Step 4: gtest を実行**

```bash
nix develop --command bash -c "source install/setup.bash && colcon test --packages-select teleop_ik --event-handlers console_direct+"
```

期待: `test_coordinate_utils_cpp` の 5 テストがすべて PASS. 既存の Python pytest も PASS していること (共存できている).

- [ ] **Step 5: コミット**

```bash
git add ros2_ws/src/teleop_ik/CMakeLists.txt \
        ros2_ws/src/teleop_ik/package.xml
git commit -m "build(teleop_ik): add teleop_ik_core lib and gtest target"
```

---

## Task 7: TDD — `test_target_msg.cpp` を追加 (メッセージ取り込み)

**Files:**
- Create: `ros2_ws/src/teleop_ik/test/test_target_msg.cpp`

- [ ] **Step 1: ファイル作成**

```cpp
// teleop_ik/test/test_target_msg.cpp
#include <gtest/gtest.h>

#include "teleop_ik/msg/target_pose_with_input.hpp"

TEST(TargetPoseWithInputMsg, DefaultValues)
{
  teleop_ik::msg::TargetPoseWithInput msg;
  EXPECT_TRUE(msg.stick_x == 0.0f);
  EXPECT_TRUE(msg.stick_y == 0.0f);
  EXPECT_EQ(msg.pose.position.x, 0.0);
  EXPECT_EQ(msg.pose.position.y, 0.0);
  EXPECT_EQ(msg.pose.position.z, 0.0);
}

TEST(TargetPoseWithInputMsg, SetFields)
{
  teleop_ik::msg::TargetPoseWithInput msg;
  msg.stick_x = 0.5f;
  msg.stick_y = -0.25f;
  msg.pose.position.x = 1.0;
  msg.pose.position.y = 2.0;
  msg.pose.position.z = 3.0;
  EXPECT_FLOAT_EQ(msg.stick_x, 0.5f);
  EXPECT_FLOAT_EQ(msg.stick_y, -0.25f);
  EXPECT_DOUBLE_EQ(msg.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(msg.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(msg.pose.position.z, 3.0);
}
```

- [ ] **Step 2: CMakeLists.txt の C++ テストブロックに target 追加**

`ament_add_gtest(test_coordinate_utils_cpp ...)` の直後に追加:

```cmake
  ament_add_gtest(test_target_msg_cpp test/test_target_msg.cpp)
```

- [ ] **Step 3: ビルド & テスト**

```bash
cd ros2_ws
nix develop --command bash -c "colcon build --packages-select teleop_ik --symlink-install && \
  source install/setup.bash && \
  colcon test --packages-select teleop_ik --event-handlers console_direct+"
```

期待: `test_target_msg_cpp` 含む全 gtest + 既存 pytest が PASS.

- [ ] **Step 4: コミット**

```bash
git add ros2_ws/src/teleop_ik/test/test_target_msg.cpp \
        ros2_ws/src/teleop_ik/CMakeLists.txt
git commit -m "test(teleop_ik): add test_target_msg gtest"
```

---

## Task 8: TDD — `ik_node` 純粋ヘルパー (`clamp_joints` / `apply_stick_deadzone` / `stamp_to_time`) のテスト先行

URDF を読み込んだノードインスタンスをセットアップする都合上、Task 8〜11 は `TeleopIKNode` のヘッダ宣言と `make_for_test` factory を先に足し、テストで URDF XML を直接渡してインスタンスを作る.

**Files:**
- Create: `ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp`
- Create: `ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp` (暫定宣言)

- [ ] **Step 1: 暫定ヘッダ作成**

`TeleopIKNode` の純粋ヘルパーを「後で実装する」前提でテストから呼べるよう、最小限の宣言だけ書く.

```cpp
// teleop_ik/include/teleop_ik/ik_node.hpp
#ifndef TELEOP_IK__IK_NODE_HPP_
#define TELEOP_IK__IK_NODE_HPP_

#include <optional>
#include <string>

#include <Eigen/Core>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

namespace teleop_ik
{

class TeleopIKNode : public rclcpp::Node
{
 public:
  // テスト用ファクトリ. URDF XML を直接渡して部分初期化する.
  // 実装は src/ik_node.cpp に置く.
  static std::unique_ptr<TeleopIKNode> make_for_test(
      const std::string & urdf_xml);

  TeleopIKNode();

  // 純粋ヘルパー (テストから直接呼ぶ).
  Eigen::VectorXd clamp_joints(const Eigen::VectorXd & q) const;
  std::pair<double, double> apply_stick_deadzone(
      double x, double y, double deadzone) const;
  std::optional<double> stamp_to_time(const builtin_interfaces::msg::Time & stamp) const;

  // メンバ: テストから状態を組み立てるため public としている.
  // 実装は src/ik_node.cpp に置く.
  Eigen::VectorXd q_current_;
  // 他のメンバは実装時に足す.
};

}  // namespace teleop_ik

#endif  // TELEOP_IK__IK_NODE_HPP_
```

> このヘッダは暫定版. 実装着手時にメンバ・メソッドを順次足す.

- [ ] **Step 2: テストファイル作成**

```cpp
// teleop_ik/test/test_ik_node_helpers.cpp
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "teleop_ik/ik_node.hpp"

namespace
{

// Lerobot SO-101 の URDF は ROS 2 環境から読まなくても xacro CLI で
// 展開した結果が手元にある. テストでは、ファイル I/O を避けるため
// 固定の URDF 文字列を使う代わりに、pinocchio で読める最小モデル
// (1 リンク) を組み立てる. これでも clamp / deadzone / stamp の
// 純粋ロジックは検証できる.
std::string minimal_urdf_xml()
{
  return R"(<robot name="min">
  <link name="base_link"/>
  <link name="link1"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0"/>
  </joint>
</robot>)";
}

class TeleopIKHelpersTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    node_ = teleop_ik::TeleopIKNode::make_for_test(minimal_urdf_xml());
    ASSERT_NE(node_, nullptr);
  }
  std::unique_ptr<teleop_ik::TeleopIKNode> node_;
};

}  // namespace

TEST_F(TeleopIKHelpersTest, ClampJointsClipsToModelLimits)
{
  Eigen::VectorXd below = node_->q_current_;
  below.fill(-2.0);
  Eigen::VectorXd out_below = node_->clamp_joints(below);
  EXPECT_LE(out_below.minCoeff(), -1.0);
  EXPECT_GE(out_below.maxCoeff(), -1.0);
}

TEST_F(TeleopIKHelpersTest, ApplyStickDeadzoneZerosWithinZone)
{
  EXPECT_EQ(node_->apply_stick_deadzone(0.05, 0.05, 0.1), (std::pair<double, double>{0.0, 0.0}));
  EXPECT_EQ(node_->apply_stick_deadzone(0.5, 0.0, 0.1), (std::pair<double, double>(0.5, 0.0)));
}

TEST_F(TeleopIKHelpersTest, StampToTimeHandlesZero)
{
  builtin_interfaces::msg::Time t;
  EXPECT_FALSE(node_->stamp_to_time(t).has_value());
  t.sec = 1;
  t.nanosec = 500000000;
  auto v = node_->stamp_to_time(t);
  ASSERT_TRUE(v.has_value());
  EXPECT_DOUBLE_EQ(*v, 1.5);
}
```

- [ ] **Step 3: CMakeLists.txt にテスト target 追加**

`ament_add_gtest(test_target_msg_cpp ...)` の後に追加:

```cmake
  ament_add_gtest(test_ik_node_helpers_cpp test/test_ik_node_helpers.cpp)
  target_link_libraries(test_ik_node_helpers_cpp teleop_ik_core)
```

- [ ] **Step 4: ビルド (失敗することを確認)**

```bash
cd ros2_ws
nix develop --command bash -c "colcon build --packages-select teleop_ik --symlink-install 2>&1 | tail -30"
```

期待: コンパイルエラー (link 失敗) — `make_for_test`, `clamp_joints` 等の未実装シンボル.

- [ ] **Step 5: コミット (テスト先行・赤)**

```bash
git add ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp \
        ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp \
        ros2_ws/src/teleop_ik/CMakeLists.txt
git commit -m "test(teleop_ik): add ik_node helpers gtest (red)"
```

---

## Task 9: `TeleopIKNode` 暫定実装 (ヘルパー 3 種を緑化)

**Files:**
- Create: `ros2_ws/src/teleop_ik/src/ik_node.cpp`

- [ ] **Step 1: 暫定実装を書く**

`make_for_test` と `clamp_joints` / `apply_stick_deadzone` / `stamp_to_time` を実装する最小コード. 他のメンバは後続 Task で足す.

```cpp
// teleop_ik/src/ik_node.cpp
#include "teleop_ik/ik_node.hpp"

#include <cmath>
#include <stdexcept>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/rclcpp.hpp>

namespace teleop_ik
{

TeleopIKNode::TeleopIKNode() : rclcpp::Node("teleop_ik_node")
{
}

std::unique_ptr<TeleopIKNode> TeleopIKNode::make_for_test(const std::string & urdf_xml)
{
  auto node = std::unique_ptr<TeleopIKNode>(new TeleopIKNode());
  // rclcpp::Node のコンストラクタは宣言パラメータを読むが、本テストでは
  // パラメータを宣言しないため rclcpp::Node の最低限の初期化で十分.
  pinocchio::Model model;
  pinocchio::urdf::buildModelFromXML(urdf_xml, model);
  // q_current_ は q のゼロ初期化ではなく pinocchio::neutral 相当.
  node->q_current_ = pinocchio::neutral(model);
  // 実装時は model_/data_ もメンバとして保持する.
  return node;
}

Eigen::VectorXd TeleopIKNode::clamp_joints(const Eigen::VectorXd & q) const
{
  // 暫定: 制限はゼロとしている. 実装時は model の lower/upperPositionLimit を使う.
  return q;
}

std::pair<double, double> TeleopIKNode::apply_stick_deadzone(
    double x, double y, double deadzone) const
{
  const double mag = std::hypot(x, y);
  if (mag <= deadzone || mag < 1e-9 || deadzone >= 1.0) {
    return {0.0, 0.0};
  }
  const double scale = (mag - deadzone) / (mag * (1.0 - deadzone));
  return {x * scale, y * scale};
}

std::optional<double> TeleopIKNode::stamp_to_time(
    const builtin_interfaces::msg::Time & stamp) const
{
  const auto sec = static_cast<int64_t>(stamp.sec);
  const auto nsec = static_cast<int64_t>(stamp.nanosec);
  if (sec == 0 && nsec == 0) {
    return std::nullopt;
  }
  return static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
}

}  // namespace teleop_ik
```

- [ ] **Step 2: ビルド & テスト**

```bash
cd ros2_ws
nix develop --command bash -c "colcon build --packages-select teleop_ik --symlink-install && \
  source install/setup.bash && \
  colcon test --packages-select teleop_ik --event-handlers console_direct+"
```

期待: `test_ik_node_helpers_cpp` の全テストが PASS.
ただし `ClampJointsClipsToModelLimits` は暫定実装で素通しなので、
`EXPECT_LE(out_below.minCoeff(), -1.0)` と `EXPECT_GE(out_below.maxCoeff(), -1.0)` の両方を満たす (q_current_ が neutral なので値域はゼロだが、clamp 関数の戻り値の全要素が [-1, 1] に入るという弱い保証).

- [ ] **Step 3: コミット (緑化)**

```bash
git add ros2_ws/src/teleop_ik/src/ik_node.cpp
git commit -m "feat(teleop_ik): add TeleopIKNode helper scaffolding (green)"
```

---

## Task 10: `clamp_joints` を URDF 制限に正しく対応させ、ヘッダに Pinocchio モデル保持

- [ ] **Step 1: ヘッダに Pinocchio モデル保持用のメンバを追加**

`include/teleop_ik/ik_node.hpp` のクラス内に以下を追加:

```cpp
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <array>

  // ...
 public:
  // ...
  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex ee_frame_id_ = 0;
  // テスト用: arm 関節 (5 個) と position 関節 (1〜3) のジョイント ID.
  // make_for_test で実 URDF を読んだときに populate する.
  std::array<pinocchio::JointIndex, 5> arm_joint_ids_{};
  std::array<pinocchio::JointIndex, 3> position_joint_ids_{};
  std::array<pinocchio::JointIndex, 2> wrist_joint_ids_{};
```

- [ ] **Step 2: `make_for_test` で model と joint ids を保持**

`make_for_test` の引数に `ee_frame_name` を追加し、実装で関節 ID も populate する:

```cpp
static std::unique_ptr<TeleopIKNode> make_for_test(
    const std::string & urdf_xml, const std::string & ee_frame_name = "ee");
```

```cpp
std::unique_ptr<TeleopIKNode> TeleopIKNode::make_for_test(
    const std::string & urdf_xml, const std::string & ee_frame_name)
{
  auto node = std::unique_ptr<TeleopIKNode>(new TeleopIKNode());
  pinocchio::urdf::buildModelFromXML(urdf_xml, node->model_);
  node->data_ = pinocchio::Data(node->model_);
  node->q_current_ = pinocchio::neutral(node->model_);

  // 関節 ID を名前 "1".."5" から引く. 見つからなければそのスロットを未定義扱い.
  for (size_t i = 0; i < 5; ++i) {
    const std::string name = std::to_string(i + 1);
    if (node->model_.existJointName(name)) {
      node->arm_joint_ids_[i] = node->model_.getJointId(name);
    } else {
      node->arm_joint_ids_[i] = static_cast<pinocchio::JointIndex>(-1);
    }
  }
  for (size_t i = 0; i < 3; ++i) {
    node->position_joint_ids_[i] = node->arm_joint_ids_[i];
  }
  node->wrist_joint_ids_[0] = node->arm_joint_ids_[3];
  node->wrist_joint_ids_[1] = node->arm_joint_ids_[4];

  if (!node->model_.existFrame(ee_frame_name)) {
    throw std::runtime_error("Frame '" + ee_frame_name + "' not found in URDF");
  }
  node->ee_frame_id_ = node->model_.getFrameId(ee_frame_name);
  return node;
}
```

- [ ] **Step 3: `clamp_joints` を実装**

```cpp
Eigen::VectorXd TeleopIKNode::clamp_joints(const Eigen::VectorXd & q) const
{
  return q.cwiseMax(model_.lowerPositionLimit).cwiseMin(model_.upperPositionLimit);
}
```

- [ ] **Step 4: minimal URDF を 3 関節に拡張**

`test_ik_node_helpers.cpp` の `minimal_urdf_xml()` を以下に置換 (SO-101 の "1"/"2"/"3" 命名に揃える):

```cpp
std::string minimal_urdf_xml()
{
  return R"(<robot name="min">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="link3"/>
  <link name="ee"/>
  <joint name="1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.5" upper="1.5"/>
    <origin xyz="0 0 0.05"/>
  </joint>
  <joint name="2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5" upper="1.5"/>
    <origin xyz="0 0 0.1"/>
  </joint>
  <joint name="3" type="revolute">
    <parent link="link2"/>
    <child link="link3"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.5" upper="1.5"/>
    <origin xyz="0 1 0"/>
  </joint>
  <joint name="ee_joint" type="fixed">
    <parent link="link3"/>
    <child link="ee"/>
    <origin xyz="0 0 0.1"/>
  </joint>
</robot>)";
}
```

テスト fixture を更新:

```cpp
void SetUp() override
{
  node_ = teleop_ik::TeleopIKNode::make_for_test(minimal_urdf_xml(), "ee");
  ASSERT_NE(node_, nullptr);
}
```

- [ ] **Step 5: テストを更新して制限を厳密にチェック**

`test_ik_node_helpers.cpp` の `ClampJointsClipsToModelLimits` を以下に置換:

```cpp
TEST_F(TeleopIKHelpersTest, ClampJointsClipsToModelLimits)
{
  const Eigen::VectorXd below = node_->model_.lowerPositionLimit.array() - 1.0;
  const Eigen::VectorXd above = node_->model_.upperPositionLimit.array() + 1.0;
  const Eigen::VectorXd out_below = node_->clamp_joints(below);
  const Eigen::VectorXd out_above = node_->clamp_joints(above);
  EXPECT_TRUE(out_below.isApprox(node_->model_.lowerPositionLimit));
  EXPECT_TRUE(out_above.isApprox(node_->model_.upperPositionLimit));
}
```

- [ ] **Step 6: ビルド & テスト**

```bash
cd ros2_ws
nix develop --command bash -c "colcon build --packages-select teleop_ik --symlink-install && \
  source install/setup.bash && \
  colcon test --packages-select teleop_ik --event-handlers console_direct+"
```

期待: 全テスト PASS.

- [ ] **Step 7: コミット**

```bash
git add ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp \
        ros2_ws/src/teleop_ik/src/ik_node.cpp \
        ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp
git commit -m "feat(teleop_ik): clamp_joints reads URDF position limits"
```

---

## Task 11: `solve_ik` のテスト先行

**Files:**
- Modify: `ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp`
- Modify: `ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp`

- [ ] **Step 1: ヘッダに `solve_ik` 宣言を追加**

```cpp
  // ヘッダ内 public:
  std::optional<Eigen::VectorXd> solve_ik(
      const Eigen::Vector3d & target_position, const Eigen::VectorXd & q_seed);
```

- [ ] **Step 2: テスト追加**

`test_ik_node_helpers.cpp` の末尾に追加:

```cpp
TEST_F(TeleopIKHelpersTest, SolveIkReturnsNulloptForUnreachableTarget)
{
  const Eigen::Vector3d unreachable(10.0, 10.0, 10.0);
  EXPECT_FALSE(node_->solve_ik(unreachable, node_->q_current_).has_value());
}

TEST_F(TeleopIKHelpersTest, SolveIkReachesCurrentPosition)
{
  // 現状の EE 位置を目標にして解く. 1 回の反復で tol を満たすはず.
  // ただし URDF 1 関節モデルでは EE フレームが無いため、フレームを追加した
  // テスト用 URDF を使う. → 実装時に URDF を拡張する.
  // ここでは枠だけ用意し、実装は Task 12 で緑化する.
  GTEST_SKIP() << "Pending Task 12 (frame-aware URDF).";
}
```

- [ ] **Step 3: ビルド (失敗することを確認)**

```bash
cd ros2_ws
nix develop --command bash -c "colcon build --packages-select teleop_ik --symlink-install 2>&1 | tail -20"
```

期待: `solve_ik` 未定義で link 失敗.

- [ ] **Step 4: コミット (赤)**

```bash
git add ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp \
        ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp
git commit -m "test(teleop_ik): add solve_ik gtest (red)"
```

---

## Task 12: `solve_ik` 実装 (緑化)

- [ ] **Step 1: ヘッダに必要な include を追加**

`ik_node.hpp` の先頭に以下を追加 (include は Task 10 である程度入っているが、`computeFrameJacobian` 等を明示 include):

```cpp
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
```

クラス内 `public:` に `solve_ik` 宣言を追加:

```cpp
  std::optional<Eigen::VectorXd> solve_ik(
      const Eigen::Vector3d & target_position, const Eigen::VectorXd & q_seed);
```

> `model_` / `data_` / `ee_frame_id_` / `position_joint_ids_` は Task 10 で
> 既に宣言済み. ここでは宣言しない.

- [ ] **Step 2: `solve_ik` を実装**

`ik_node.cpp` に追加:

```cpp
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

std::optional<Eigen::VectorXd> TeleopIKNode::solve_ik(
    const Eigen::Vector3d & target_position, const Eigen::VectorXd & q_seed)
{
  const double damping = 1e-6;
  const int max_iter = 100;
  const double tol = 1e-4;
  const double dt = 0.2;

  // 位置制御に用いる v-index を position_joint_ids_ から取得
  std::vector<pinocchio::Index> position_velocity_indexes;
  position_velocity_indexes.reserve(position_joint_ids_.size());
  for (const auto jid : position_joint_ids_) {
    if (jid == static_cast<pinocchio::JointIndex>(-1)) continue;
    position_velocity_indexes.push_back(model_.joints[jid].idx_v());
  }

  Eigen::VectorXd q = clamp_joints(q_seed);
  for (int i = 0; i < max_iter; ++i) {
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    const Eigen::Vector3d err = target_position - data_.oMf[ee_frame_id_].translation();
    if (err.norm() < tol) {
      return clamp_joints(q);
    }
    Eigen::MatrixXd J_full = pinocchio::computeFrameJacobian(
        model_, data_, q, ee_frame_id_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
    // 位置 3 行 × position_velocity_indexes 列 だけ抜き出す
    Eigen::MatrixXd J(3, position_velocity_indexes.size());
    for (Eigen::Index c = 0; c < static_cast<Eigen::Index>(position_velocity_indexes.size()); ++c) {
      J.col(c) = J_full.topRows(3).col(position_velocity_indexes[c]);
    }
    const Eigen::Matrix3d JJt = J * J.transpose() + damping * Eigen::Matrix3d::Identity();
    const Eigen::VectorXd dq_pos = J.transpose() * JJt.ldlt().solve(err);
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(model_.nv);
    for (size_t k = 0; k < position_velocity_indexes.size(); ++k) {
      dq[position_velocity_indexes[k]] = dq_pos[k];
    }
    q = clamp_joints(pinocchio::integrate(model_, q, dq * dt));
  }
  RCLCPP_WARN(get_logger(), "IK did not converge within max iterations");
  return std::nullopt;
}
```

- [ ] **Step 3: テストの `SolveIkReachesCurrentPosition` を緑化**

`test_ik_node_helpers.cpp` の `GTEST_SKIP` を以下に置換:

```cpp
TEST_F(TeleopIKHelpersTest, SolveIkReachesCurrentPosition)
{
  pinocchio::forwardKinematics(node_->model_, node_->data_, node_->q_current_);
  pinocchio::updateFramePlacements(node_->model_, node_->data_);
  const Eigen::Vector3d current_ee = node_->data_.oMf[node_->ee_frame_id_].translation();
  const auto result = node_->solve_ik(current_ee, node_->q_current_);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR((*result - node_->q_current_).norm(), 0.0, 1e-3);
}
```

- [ ] **Step 4: ビルド & テスト**

```bash
cd ros2_ws
nix develop --command bash -c "colcon build --packages-select teleop_ik --symlink-install && \
  source install/setup.bash && \
  colcon test --packages-select teleop_ik --event-handlers console_direct+"
```

期待: `test_ik_node_helpers_cpp` の全テストが PASS.

- [ ] **Step 5: コミット**

```bash
git add ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp \
        ros2_ws/src/teleop_ik/src/ik_node.cpp \
        ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp
git commit -m "feat(teleop_ik): implement solve_ik (damped LS CLIK)"
```

---

## Task 13: `ik_node` のセッション管理 & 統合コールバックを実装

`on_active`, `on_target_with_input`, `on_gripper`, `on_joint_states`, および ARM/Gripper JointTrajectory publish, 50 Hz timer は後続 Task で ROS 2 ノードとして配線する. この Task ではその前段として、テストから直接呼べるよう `on_target_with_input` のロジック (スティック積分含む) を `TeleopIKNode` のメンバとして実装する.

**Files:**
- Modify: `ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp`
- Modify: `ros2_ws/src/teleop_ik/src/ik_node.cpp`
- Modify: `ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp`

- [ ] **Step 1: ヘッダに必要な型を追加**

```cpp
#include <array>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

  // セッション状態 (active 中のみ有効).
  bool active_ = false;
  Eigen::Vector3d arm_init_pos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d unity_anchor_pos_ = Eigen::Vector3d::Zero();
  bool unity_anchor_set_ = false;
  Eigen::Vector2d wrist_init_pos_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d integrated_stick_ = Eigen::Vector2d::Zero();
  std::optional<double> last_msg_stamp_;
  Eigen::VectorXd q_solution_;
  // 関節 ID は Task 10 で宣言済み. ここでは重複宣言しない.

 public:
  void on_active(bool active);
  void on_joint_state(const std::string & name, double position);
  void on_target_with_input(
      const geometry_msgs::msg::Pose & pose,
      float stick_x, float stick_y,
      const builtin_interfaces::msg::Time & stamp,
      double position_scale,
      double stick_velocity_scale,
      double stick_deadzone,
      double stick_max_delta_per_msg,
      double stick_fallback_dt,
      bool unity_conversion);
  void on_gripper(double value);
  trajectory_msgs::msg::JointTrajectory make_arm_trajectory(
      const Eigen::VectorXd & q, double trajectory_time_from_start) const;
  trajectory_msgs::msg::JointTrajectory make_gripper_trajectory(
      double angle, double trajectory_time_from_start) const;
```

- [ ] **Step 2: 実装を書く**

`ik_node.cpp` の末尾に以下を追加 (実装は Python 版と数式同一):

```cpp
void TeleopIKNode::on_active(bool active)
{
  if (active) {
    if (!active_) {
      // セッション開始
      pinocchio::forwardKinematics(model_, data_, q_current_);
      pinocchio::updateFramePlacements(model_, data_);
      arm_init_pos_ = data_.oMf[ee_frame_id_].translation();
      unity_anchor_set_ = false;
      q_solution_ = q_current_;
      wrist_init_pos_.setZero();
      integrated_stick_.setZero();
      last_msg_stamp_.reset();
      active_ = true;
    } else {
      // 再アクティブ化: スティックのみリセット
      integrated_stick_.setZero();
      last_msg_stamp_.reset();
    }
  } else if (active_) {
    active_ = false;
  }
}

void TeleopIKNode::on_joint_state(const std::string & name, double position)
{
  if (!model_.existJointName(name)) {
    return;
  }
  const auto jid = model_.getJointId(name);
  const auto idx_q = model_.joints[jid].idx_q();
  if (idx_q >= 0 && static_cast<Eigen::Index>(idx_q) < q_current_.size()) {
    q_current_[idx_q] = position;
  }
}

void TeleopIKNode::on_target_with_input(
    const geometry_msgs::msg::Pose & pose,
    float stick_x, float stick_y,
    const builtin_interfaces::msg::Time & stamp,
    double position_scale,
    double stick_velocity_scale,
    double stick_deadzone,
    double stick_max_delta_per_msg,
    double stick_fallback_dt,
    bool unity_conversion)
{
  if (!active_) {
    return;
  }

  Eigen::Vector3d ros_pos;
  if (unity_conversion) {
    ros_pos = teleop_ik::unity_position_to_ros(pose.position.x, pose.position.y, pose.position.z, position_scale);
  } else {
    ros_pos << pose.position.x * position_scale,
               pose.position.y * position_scale,
               pose.position.z * position_scale;
  }

  if (!unity_anchor_set_) {
    // 初回: アンカーを記録して return
    unity_anchor_pos_ = ros_pos;
    unity_anchor_set_ = true;
    last_msg_stamp_ = stamp_to_time(stamp);
    return;
  }

  // 2 回目以降
  const Eigen::Vector3d delta = ros_pos - unity_anchor_pos_;
  const Eigen::Vector3d target_pos = arm_init_pos_ + delta;

  // dt
  const auto now = stamp_to_time(stamp);
  double delta_t;
  if (!last_msg_stamp_.has_value() || !now.has_value()) {
    delta_t = stick_fallback_dt;
  } else {
    delta_t = *now - *last_msg_stamp_;
    if (delta_t <= 0.0 || delta_t > 0.5) {
      delta_t = stick_fallback_dt;
    }
  }
  last_msg_stamp_ = now;

  // スティック deadzone
  const auto [vx, vy] = apply_stick_deadzone(stick_x, stick_y, stick_deadzone);
  // 1 メッセージあたり cap
  const double cap_v = stick_max_delta_per_msg / std::max(stick_velocity_scale * delta_t, 1e-6);
  const double vx_c = std::clamp(vx, -cap_v, cap_v);
  const double vy_c = std::clamp(vy, -cap_v, cap_v);
  integrated_stick_.x() += vx_c * stick_velocity_scale * delta_t;
  integrated_stick_.y() += vy_c * stick_velocity_scale * delta_t;

  // q_seed: 手首を目標角度に固定
  Eigen::VectorXd q_seed = q_solution_;
  if (wrist_joint_ids_[0] != static_cast<pinocchio::JointIndex>(-1)) {
    const auto idx_q_0 = model_.joints[wrist_joint_ids_[0]].idx_q();
    q_seed[idx_q_0] = wrist_init_pos_.y() + integrated_stick_.y();  // stick_y → joint 4
  }
  if (wrist_joint_ids_[1] != static_cast<pinocchio::JointIndex>(-1)) {
    const auto idx_q_1 = model_.joints[wrist_joint_ids_[1]].idx_q();
    q_seed[idx_q_1] = wrist_init_pos_.x() + integrated_stick_.x();  // stick_x → joint 5
  }
  q_seed = clamp_joints(q_seed);

  if (auto result = solve_ik(target_pos, q_seed); result.has_value()) {
    q_solution_ = *result;
  }
}

void TeleopIKNode::on_gripper(double value)
{
  if (!active_) {
    return;
  }
  const double clamped = std::clamp(value, 0.0, 1.0);
  (void)clamped;  // 実装は ARM ノード側で publish するため、ここでは何もしない.
  // 仕様: gripper 値は arm trajectory とは別の JointTrajectory で publish する.
  // publish 呼び出しは ROS 2 wiring Task で配線する.
}

trajectory_msgs::msg::JointTrajectory TeleopIKNode::make_arm_trajectory(
    const Eigen::VectorXd & q, double trajectory_time_from_start) const
{
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = {"1", "2", "3", "4", "5"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  for (const auto jid : arm_joint_ids_) {
    if (jid == static_cast<pinocchio::JointIndex>(-1)) continue;
    const auto idx_q = model_.joints[jid].idx_q();
    point.positions.push_back(q[idx_q]);
  }
  const int sec = static_cast<int>(trajectory_time_from_start);
  const int nanosec = static_cast<int>((trajectory_time_from_start - sec) * 1e9);
  point.time_from_start = builtin_interfaces::msg::Duration{static_cast<int32_t>(sec), static_cast<uint32_t>(nanosec)};
  traj.points.push_back(point);
  return traj;
}

trajectory_msgs::msg::JointTrajectory TeleopIKNode::make_gripper_trajectory(
    double angle, double trajectory_time_from_start) const
{
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = {"6"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.push_back(angle);
  const int sec = static_cast<int>(trajectory_time_from_start);
  const int nanosec = static_cast<int>((trajectory_time_from_start - sec) * 1e9);
  point.time_from_start = builtin_interfaces::msg::Duration{static_cast<int32_t>(sec), static_cast<uint32_t>(nanosec)};
  traj.points.push_back(point);
  return traj;
}
```

- [ ] **Step 3: テスト追加 (スティック dt 積分)**

`test_ik_node_helpers.cpp` の末尾に追加:

```cpp
TEST_F(TeleopIKHelpersTest, OnTargetWithInputIntegratesStickPerMessage)
{
  node_->active_ = true;
  node_->unity_anchor_set_ = true;
  node_->unity_anchor_pos_.setZero();
  node_->arm_init_pos_.setZero();
  node_->q_solution_ = node_->q_current_;
  node_->wrist_init_pos_.setZero();
  node_->integrated_stick_.setZero();
  node_->last_msg_stamp_.reset();

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  builtin_interfaces::msg::Time stamp;
  node_->on_target_with_input(pose, 1.0f, 0.5f, stamp,
      /*position_scale=*/1.0,
      /*stick_velocity_scale=*/1.0,
      /*stick_deadzone=*/0.0,
      /*stick_max_delta_per_msg=*/10.0,
      /*stick_fallback_dt=*/0.1,
      /*unity_conversion=*/false);
  EXPECT_NEAR(node_->integrated_stick_.x(), 0.1, 1e-9);
  EXPECT_NEAR(node_->integrated_stick_.y(), 0.05, 1e-9);
}
```

- [ ] **Step 4: ビルド & テスト**

```bash
cd ros2_ws
nix develop --command bash -c "colcon build --packages-select teleop_ik --symlink-install && \
  source install/setup.bash && \
  colcon test --packages-select teleop_ik --event-handlers console_direct+"
```

期待: 全テスト PASS.

- [ ] **Step 5: コミット**

```bash
git add ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp \
        ros2_ws/src/teleop_ik/src/ik_node.cpp \
        ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp
git commit -m "feat(teleop_ik): add session state and on_target_with_input logic"
```

---

## Task 14: `ik_node` を ROS 2 ノードとして配線

`on_active` / `on_joint_state` / `on_target_with_input` / `on_gripper` を Subscription に紐付け, パラメータ宣言, publisher 生成, `main` を実装する.

**Files:**
- Modify: `ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp`
- Modify: `ros2_ws/src/teleop_ik/src/ik_node.cpp`
- Modify: `ros2_ws/src/teleop_ik/CMakeLists.txt`

- [ ] **Step 1: ヘッダに ROS 2 メンバを追加**

```cpp
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <teleop_ik/msg/target_pose_with_input.hpp>

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_active_;
  rclcpp::Subscription<teleop_ik::msg::TargetPoseWithInput>::SharedPtr sub_target_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_gripper_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  // Publishers
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_gripper_;

  // 実機用 callback (ラッパ)
  void on_active_msg(const std_msgs::msg::Bool::SharedPtr msg);
  void on_target_msg(const teleop_ik::msg::TargetPoseWithInput::SharedPtr msg);
  void on_gripper_msg(const std_msgs::msg::Float64::SharedPtr msg);
  void on_joint_states_msg(const sensor_msgs::msg::JointState::SharedPtr msg);
```

- [ ] **Step 2: コンストラクタで xacro 読み込み & パラメータ宣言 & ROS 配線**

`ik_node.cpp` の `TeleopIKNode::TeleopIKNode()` を以下に置換:

```cpp
TeleopIKNode::TeleopIKNode() : rclcpp::Node("teleop_ik_node")
{
  // パラメータ宣言
  this->declare_parameter<std::string>("urdf_path", "");
  this->declare_parameter<std::string>("end_effector_frame", "gripper");
  this->declare_parameter<double>("position_scale", 1.0);
  this->declare_parameter<double>("ik_damping", 1e-6);
  this->declare_parameter<int>("ik_max_iterations", 100);
  this->declare_parameter<double>("ik_tolerance", 1e-4);
  this->declare_parameter<double>("trajectory_time_from_start", 0.1);
  this->declare_parameter<bool>("unity_conversion", true);
  this->declare_parameter<double>("stick_velocity_scale", 1.5);
  this->declare_parameter<double>("stick_deadzone", 0.1);
  this->declare_parameter<double>("stick_max_delta_per_msg", 0.2);
  this->declare_parameter<double>("stick_fallback_dt", 0.0111);

  // xacro 処理 & モデル構築
  const auto urdf_path = this->get_parameter("urdf_path").as_string();
  if (urdf_path.empty()) {
    RCLCPP_FATAL(get_logger(), "Parameter 'urdf_path' is required");
    throw std::runtime_error("Parameter 'urdf_path' is required");
  }
  const std::string urdf_xml = process_xacro(urdf_path);
  pinocchio::urdf::buildModelFromXML(urdf_xml, model_);
  data_ = pinocchio::Data(model_);
  q_current_ = pinocchio::neutral(model_);

  const auto ee_frame = this->get_parameter("end_effector_frame").as_string();
  if (!model_.existFrame(ee_frame)) {
    RCLCPP_FATAL(get_logger(), "Frame '%s' not found in URDF", ee_frame.c_str());
    throw std::runtime_error("Frame '" + ee_frame + "' not found in URDF");
  }
  ee_frame_id_ = model_.getFrameId(ee_frame);

  // arm / wrist joint ids
  for (size_t i = 0; i < 5; ++i) {
    const std::string name = std::to_string(i + 1);
    if (!model_.existJointName(name)) {
      RCLCPP_FATAL(get_logger(), "Joint '%s' not found in URDF", name.c_str());
      throw std::runtime_error("Joint '" + name + "' not found in URDF");
    }
    arm_joint_ids_[i] = model_.getJointId(name);
  }
  for (size_t i = 0; i < 3; ++i) {
    position_joint_ids_[i] = arm_joint_ids_[i];
  }
  wrist_joint_ids_[0] = arm_joint_ids_[3];
  wrist_joint_ids_[1] = arm_joint_ids_[4];

  // QoS
  rclcpp::QoS target_qos(rclcpp::KeepLast(10));
  target_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  target_qos.durability(rclcpp::DurabilityPolicy::Volatile);

  // Subscriptions
  sub_active_ = this->create_subscription<std_msgs::msg::Bool>(
      "/teleop/active", 10, std::bind(&TeleopIKNode::on_active_msg, this, std::placeholders::_1));
  sub_target_ = this->create_subscription<teleop_ik::msg::TargetPoseWithInput>(
      "/teleop/target", target_qos, std::bind(&TeleopIKNode::on_target_msg, this, std::placeholders::_1));
  sub_gripper_ = this->create_subscription<std_msgs::msg::Float64>(
      "/teleop/gripper", 10, std::bind(&TeleopIKNode::on_gripper_msg, this, std::placeholders::_1));
  sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/follower/joint_states", 10, std::bind(&TeleopIKNode::on_joint_states_msg, this, std::placeholders::_1));

  // Publishers
  pub_arm_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/follower/arm_controller/joint_trajectory", 10);
  pub_gripper_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/follower/gripper_controller/joint_trajectory", 10);
}
```

- [ ] **Step 3: ROS 2 callback ラッパを実装**

`ik_node.cpp` に追加:

```cpp
void TeleopIKNode::on_active_msg(const std_msgs::msg::Bool::SharedPtr msg)
{
  on_active(msg->data);
}

void TeleopIKNode::on_target_msg(const teleop_ik::msg::TargetPoseWithInput::SharedPtr msg)
{
  on_target_with_input(
      msg->pose, msg->stick_x, msg->stick_y, msg->header.stamp,
      this->get_parameter("position_scale").as_double(),
      this->get_parameter("stick_velocity_scale").as_double(),
      this->get_parameter("stick_deadzone").as_double(),
      this->get_parameter("stick_max_delta_per_msg").as_double(),
      this->get_parameter("stick_fallback_dt").as_double(),
      this->get_parameter("unity_conversion").as_bool());

  if (q_solution_.size() == 0) {
    return;
  }
  const double t = this->get_parameter("trajectory_time_from_start").as_double();
  pub_arm_->publish(make_arm_trajectory(q_solution_, t));
}

void TeleopIKNode::on_gripper_msg(const std_msgs::msg::Float64::SharedPtr msg)
{
  on_gripper(msg->data);
  if (!active_) {
    return;
  }
  const double value = std::clamp(msg->data, 0.0, 1.0);
  const double lower = -0.174533;
  const double upper = 1.74533;
  const double angle = lower + value * (upper - lower);
  const double t = this->get_parameter("trajectory_time_from_start").as_double();
  pub_gripper_->publish(make_gripper_trajectory(angle, t));
}

void TeleopIKNode::on_joint_states_msg(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (i < msg->position.size()) {
      on_joint_state(msg->name[i], msg->position[i]);
    }
  }
}
```

- [ ] **Step 4: xacro 処理 & main を実装**

`ik_node.cpp` の先頭に include を追加:

```cpp
#include <cstdio>
#include <array>
#include <memory>

#include "teleop_ik/coordinate_utils.hpp"
```

xacro 処理:

```cpp
static std::string process_xacro(const std::string & xacro_path)
{
  std::string cmd = "xacro " + xacro_path;
  std::array<char, 4096> buf;
  std::string out;
  FILE * pipe = popen(cmd.c_str(), "r");
  if (!pipe) {
    throw std::runtime_error("popen(xacro) failed");
  }
  while (fgets(buf.data(), buf.size(), pipe) != nullptr) {
    out += buf.data();
  }
  const int rc = pclose(pipe);
  if (rc != 0) {
    throw std::runtime_error("xacro CLI failed with code " + std::to_string(rc));
  }
  return out;
}
```

main:

```cpp
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<TeleopIKNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "teleop_ik_node: %s\n", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 5: CMakeLists.txt に executable 追加**

`ament_target_dependencies(teleop_ik_core Eigen3)` の後に追加:

```cmake
add_executable(teleop_ik_node src/ik_node.cpp)
target_include_directories(teleop_ik_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(teleop_ik_node
  rclcpp geometry_msgs std_msgs sensor_msgs trajectory_msgs builtin_interfaces)
target_link_libraries(teleop_ik_node teleop_ik_core pinocchio::pinocchio)
install(TARGETS teleop_ik_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS teleop_ik_core
  EXPORT export_teleop_ikTargets
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
```

- [ ] **Step 6: ビルド & 全テスト**

```bash
cd ros2_ws
nix develop --command bash -c "colcon build --packages-select teleop_ik --symlink-install && \
  source install/setup.bash && \
  colcon test --packages-select teleop_ik --event-handlers console_direct+"
```

期待: 全 gtest + pytest が PASS.

- [ ] **Step 7: ノード起動テスト (dry-run)**

```bash
cd ros2_ws
nix develop --command bash -c "source install/setup.bash && \
  ros2 run teleop_ik teleop_ik_node --ros-args \
    -p urdf_path:=\$(ros2 pkg prefix lerobot_description)/share/lerobot_description/urdf/so101.urdf.xacro 2>&1 | head -10"
```

期待: "Pinocchio model loaded" ログが出ること. (URDF パスは環境依存なのでエラーになったら調整.)

- [ ] **Step 8: コミット**

```bash
git add ros2_ws/src/teleop_ik/include/teleop_ik/ik_node.hpp \
        ros2_ws/src/teleop_ik/src/ik_node.cpp \
        ros2_ws/src/teleop_ik/CMakeLists.txt
git commit -m "feat(teleop_ik): wire TeleopIKNode as rclcpp executable"
```

---

## Task 15: `gamepad_node` を実装 (lib + executable)

**Files:**
- Create: `ros2_ws/src/teleop_ik/include/teleop_ik/gamepad_node.hpp`
- Create: `ros2_ws/src/teleop_ik/src/gamepad_node.cpp`
- Create: `ros2_ws/src/teleop_ik/test/test_gamepad_node.cpp`
- Modify: `ros2_ws/src/teleop_ik/CMakeLists.txt`

- [ ] **Step 1: ヘッダ作成**

```cpp
// teleop_ik/include/teleop_ik/gamepad_node.hpp
#ifndef TELEOP_IK__GAMEPAD_NODE_HPP_
#define TELEOP_IK__GAMEPAD_NODE_HPP_

#include <chrono>
#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <teleop_ik/msg/target_pose_with_input.hpp>

namespace teleop_ik
{

class GamepadTeleopNode : public rclcpp::Node
{
 public:
  GamepadTeleopNode();

  // テスト用.
  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg);
  void timer_tick();

  // 状態 (テストから触る).
  bool active_ = false;
  bool prev_toggle_pressed_ = false;
  double target_x_ = 0.0;
  double target_y_ = 0.0;
  double target_z_ = 0.0;
  double gripper_value_ = 0.5;
  bool joy_received_ = false;
  std::optional<sensor_msgs::msg::Joy> latest_joy_;

 private:
  double linear_speed_;
  double vertical_speed_;
  double deadzone_;
  double dt_;
  int axis_x_;
  int axis_y_;
  int axis_z_;
  int btn_gripper_open_;
  int btn_gripper_close_;
  int btn_toggle_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  rclcpp::Publisher<teleop_ik::msg::TargetPoseWithInput>::SharedPtr pub_target_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_active_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_gripper_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace teleop_ik

#endif  // TELEOP_IK__GAMEPAD_NODE_HPP_
```

- [ ] **Step 2: 実装作成**

```cpp
// teleop_ik/src/gamepad_node.cpp
#include "teleop_ik/gamepad_node.hpp"

#include <algorithm>

namespace teleop_ik
{

namespace
{
double apply_deadzone(double v, double dz) { return std::abs(v) < dz ? 0.0 : v; }
double safe_axis(const sensor_msgs::msg::Joy & j, int idx) {
  return idx < static_cast<int>(j.axes.size()) ? j.axes[idx] : 0.0;
}
bool safe_button(const sensor_msgs::msg::Joy & j, int idx) {
  return idx < static_cast<int>(j.buttons.size()) ? j.buttons[idx] != 0 : false;
}
}  // namespace

GamepadTeleopNode::GamepadTeleopNode() : rclcpp::Node("gamepad_teleop_node")
{
  this->declare_parameter<double>("publish_rate", 50.0);
  this->declare_parameter<double>("linear_speed", 0.05);
  this->declare_parameter<double>("vertical_speed", 0.05);
  this->declare_parameter<double>("deadzone", 0.15);
  this->declare_parameter<int>("axis_x", 1);
  this->declare_parameter<int>("axis_y", 0);
  this->declare_parameter<int>("axis_z", 3);
  this->declare_parameter<int>("button_gripper_open", 5);
  this->declare_parameter<int>("button_gripper_close", 4);
  this->declare_parameter<int>("button_toggle_active", 0);

  linear_speed_ = this->get_parameter("linear_speed").as_double();
  vertical_speed_ = this->get_parameter("vertical_speed").as_double();
  deadzone_ = this->get_parameter("deadzone").as_double();
  axis_x_ = this->get_parameter("axis_x").as_int();
  axis_y_ = this->get_parameter("axis_y").as_int();
  axis_z_ = this->get_parameter("axis_z").as_int();
  btn_gripper_open_ = this->get_parameter("button_gripper_open").as_int();
  btn_gripper_close_ = this->get_parameter("button_gripper_close").as_int();
  btn_toggle_ = this->get_parameter("button_toggle_active").as_int();

  const double rate = this->get_parameter("publish_rate").as_double();
  dt_ = 1.0 / rate;

  sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&GamepadTeleopNode::on_joy, this, std::placeholders::_1));
  pub_target_ = this->create_publisher<teleop_ik::msg::TargetPoseWithInput>("/teleop/target", 10);
  pub_active_ = this->create_publisher<std_msgs::msg::Bool>("/teleop/active", 10);
  pub_gripper_ = this->create_publisher<std_msgs::msg::Float64>("/teleop/gripper", 10);

  timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt_), std::bind(&GamepadTeleopNode::timer_tick, this));
}

void GamepadTeleopNode::on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (!joy_received_) {
    joy_received_ = true;
  }
  latest_joy_ = *msg;
}

void GamepadTeleopNode::timer_tick()
{
  if (!latest_joy_.has_value()) {
    return;
  }
  const auto & joy = *latest_joy_;

  // Cross トグル (rising edge)
  const bool toggle_pressed = safe_button(joy, btn_toggle_);
  if (toggle_pressed && !prev_toggle_pressed_) {
    active_ = !active_;
    pub_active_->publish(std_msgs::msg::Bool().set__data(active_));
    if (active_) {
      target_x_ = target_y_ = target_z_ = 0.0;
      gripper_value_ = 0.5;
    }
  }
  prev_toggle_pressed_ = toggle_pressed;

  if (!active_) {
    return;
  }

  const double vx = apply_deadzone(safe_axis(joy, axis_x_), deadzone_);
  const double vy = apply_deadzone(safe_axis(joy, axis_y_), deadzone_);
  const double vz = apply_deadzone(safe_axis(joy, axis_z_), deadzone_);
  target_x_ += vx * linear_speed_ * dt_;
  target_y_ += vy * linear_speed_ * dt_;
  target_z_ += vz * vertical_speed_ * dt_;

  constexpr double gripper_speed = 2.0;
  if (safe_button(joy, btn_gripper_open_)) {
    gripper_value_ = std::min(1.0, gripper_value_ + gripper_speed * dt_);
  }
  if (safe_button(joy, btn_gripper_close_)) {
    gripper_value_ = std::max(0.0, gripper_value_ - gripper_speed * dt_);
  }

  auto msg = teleop_ik::msg::TargetPoseWithInput();
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "world";
  msg.pose.position.x = target_x_;
  msg.pose.position.y = target_y_;
  msg.pose.position.z = target_z_;
  msg.pose.orientation.w = 1.0;
  msg.stick_x = 0.0f;
  msg.stick_y = 0.0f;
  pub_target_->publish(msg);

  pub_gripper_->publish(std_msgs::msg::Float64().set__data(gripper_value_));
}

}  // namespace teleop_ik

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<teleop_ik::GamepadTeleopNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "gamepad_teleop_node: %s\n", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 3: テスト作成**

```cpp
// teleop_ik/test/test_gamepad_node.cpp
#include <gtest/gtest.h>

#include "teleop_ik/gamepad_node.hpp"

namespace
{

class TestNode : public teleop_ik::GamepadTeleopNode
{
 public:
  using teleop_ik::GamepadTeleopNode::GamepadTeleopNode;
};

}  // namespace

TEST(GamepadNode, TogglesActiveOnRisingEdge)
{
  // rclcpp::init/shutdown は rclcpp::init がノード生成に必要.
  // ただしパラメータ宣言だけなら不要. ここでは手動で init.
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<teleop_ik::GamepadTeleopNode>();
  EXPECT_FALSE(node->active_);

  // 直接 timer_tick を呼ぶ.
  sensor_msgs::msg::Joy joy;
  joy.axes = {0.0, 0.0, 0.0, 0.0};
  joy.buttons.resize(16);
  // Cross を ON
  joy.buttons[0] = 1;
  node->latest_joy_ = joy;
  node->timer_tick();
  EXPECT_TRUE(node->active_);

  // Cross を離す
  joy.buttons[0] = 0;
  node->latest_joy_ = joy;
  node->timer_tick();
  // prev_toggle_pressed_ = false になる

  // Cross を再度 ON
  joy.buttons[0] = 1;
  node->latest_joy_ = joy;
  node->timer_tick();
  EXPECT_FALSE(node->active_);

  rclcpp::shutdown();
}
```

- [ ] **Step 4: CMakeLists.txt に executable とテスト追加**

`ament_target_dependencies(teleop_ik_node ...)` の後に追加:

```cmake
add_executable(gamepad_teleop_node src/gamepad_node.cpp)
target_include_directories(gamepad_teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(gamepad_teleop_node
  rclcpp std_msgs sensor_msgs)
target_link_libraries(gamepad_teleop_node pinocchio::pinocchio)
install(TARGETS gamepad_teleop_node RUNTIME DESTINATION lib/${PROJECT_NAME})
```

`ament_add_gtest(test_ik_node_helpers_cpp ...)` の後に追加:

```cmake
  ament_add_gtest(test_gamepad_node_cpp test/test_gamepad_node.cpp)
  ament_target_dependencies(test_gamepad_node_cpp rclcpp sensor_msgs std_msgs)
```

`install(TARGETS teleop_ik_core ...)` を以下に置換:

```cmake
install(TARGETS
  teleop_ik_core teleop_ik_node gamepad_teleop_node
  EXPORT export_teleop_ikTargets
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
```

- [ ] **Step 5: ビルド & テスト**

```bash
cd ros2_ws
nix develop --command bash -c "colcon build --packages-select teleop_ik --symlink-install && \
  source install/setup.bash && \
  colcon test --packages-select teleop_ik --event-handlers console_direct+"
```

期待: 全 gtest + pytest PASS.

- [ ] **Step 6: コミット**

```bash
git add ros2_ws/src/teleop_ik/include/teleop_ik/gamepad_node.hpp \
        ros2_ws/src/teleop_ik/src/gamepad_node.cpp \
        ros2_ws/src/teleop_ik/test/test_gamepad_node.cpp \
        ros2_ws/src/teleop_ik/CMakeLists.txt
git commit -m "feat(teleop_ik): implement gamepad_teleop_node (rclcpp)"
```

---

## Task 16: `test_qos.cpp` を追加 (QoS プロファイル検査)

**Files:**
- Create: `ros2_ws/src/teleop_ik/test/test_qos.cpp`
- Modify: `ros2_ws/src/teleop_ik/CMakeLists.txt`

- [ ] **Step 1: テスト作成**

```cpp
// teleop_ik/test/test_qos.cpp
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "teleop_ik/ik_node.hpp"
#include "teleop_ik/gamepad_node.hpp"

namespace
{
rclcpp::QoS::Reliability get_sub_reliability(
    rclcpp::Node::SharedPtr probe, const std::string & topic)
{
  const auto endpoints = probe->get_subscriptions_info_by_topic(topic);
  EXPECT_EQ(endpoints.size(), 1u);
  return endpoints[0].qos_profile().reliability();
}
}  // namespace

class QosTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    ik_node_ = std::make_shared<teleop_ik::TeleopIKNode>();
    probe_ = std::make_shared<rclcpp::Node>("qos_probe");
  }
  void TearDown() override
  {
    probe_.reset();
    ik_node_.reset();
    rclcpp::shutdown();
  }
  rclcpp::Node::SharedPtr ik_node_;
  rclcpp::Node::SharedPtr probe_;
};

TEST_F(QosTest, TargetSubscriptionIsBestEffort)
{
  EXPECT_EQ(get_sub_reliability(probe_, "/teleop/target"),
            rclcpp::Reliability::BestEffort);
}

TEST_F(QosTest, ActiveSubscriptionIsReliable)
{
  EXPECT_EQ(get_sub_reliability(probe_, "/teleop/active"),
            rclcpp::Reliability::Reliable);
}

TEST_F(QosTest, GripperSubscriptionIsReliable)
{
  EXPECT_EQ(get_sub_reliability(probe_, "/teleop/gripper"),
            rclcpp::Reliability::Reliable);
}
```

- [ ] **Step 2: CMakeLists.txt に追加**

`ament_add_gtest(test_gamepad_node_cpp ...)` の後に追加:

```cmake
  ament_add_gtest(test_qos_cpp test/test_qos.cpp)
  ament_target_dependencies(test_qos_cpp rclcpp)
  target_link_libraries(test_qos_cpp teleop_ik_core)
```

- [ ] **Step 3: ビルド & テスト**

```bash
cd ros2_ws
nix develop --command bash -c "colcon build --packages-select teleop_ik --symlink-install && \
  source install/setup.bash && \
  colcon test --packages-select teleop_ik --event-handlers console_direct+"
```

期待: `test_qos_cpp` 含む全テスト PASS.

- [ ] **Step 4: コミット**

```bash
git add ros2_ws/src/teleop_ik/test/test_qos.cpp \
        ros2_ws/src/teleop_ik/CMakeLists.txt
git commit -m "test(teleop_ik): add QoS profile gtest"
```

---

## Task 17: `test_packaging.cpp` を追加 (install 成果物検査)

**Files:**
- Create: `ros2_ws/src/teleop_ik/test/test_packaging.cpp`
- Modify: `ros2_ws/src/teleop_ik/CMakeLists.txt`

- [ ] **Step 1: テスト作成**

```cpp
// teleop_ik/test/test_packaging.cpp
#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

namespace fs = std::filesystem;

TEST(Packaging, InstallLibDirectoryContainsExecutables)
{
  const auto share = ament_index_cpp::get_package_share_directory("teleop_ik");
  // install/<prefix>/lib/teleop_ik/
  const fs::path lib_dir = fs::path(share).parent_path().parent_path() / "lib" / "teleop_ik";
  EXPECT_TRUE(fs::exists(lib_dir / "teleop_ik_node"));
  EXPECT_TRUE(fs::exists(lib_dir / "gamepad_teleop_node"));
}
```

- [ ] **Step 2: CMakeLists.txt に追加**

```cmake
  ament_add_gtest(test_packaging_cpp test/test_packaging.cpp)
  ament_target_dependencies(test_packaging_cpp ament_index_cpp)
```

- [ ] **Step 3: ビルド & テスト**

```bash
cd ros2_ws
nix develop --command bash -c "colcon build --packages-select teleop_ik --symlink-install && \
  source install/setup.bash && \
  colcon test --packages-select teleop_ik --event-handlers console_direct+"
```

期待: 全テスト PASS.

- [ ] **Step 4: コミット**

```bash
git add ros2_ws/src/teleop_ik/test/test_packaging.cpp \
        ros2_ws/src/teleop_ik/CMakeLists.txt
git commit -m "test(teleop_ik): add install-layout gtest"
```

---

## Task 18: `ik_node` の詳細テストを実 URDF ベースで再実装 & Python テストファイル整理

`test_ik_node_helpers.cpp` のテストを `lerobot_description` の実 URDF を使って Python 版 `test_ik_node.py` と等価に網羅する. 同時に Python 版テスト (pytest) は削除する.

**Files:**
- Modify: `ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp` (リネームではなく内容を拡張)
- Delete: Python テストファイル群

- [ ] **Step 1: 実 URDF を使う fixture を追加**

`test_ik_node_helpers.cpp` の冒頭に include を追加:

```cpp
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/parsers/urdf.hpp>
```

ヘルパ関数を追加:

```cpp
std::string load_real_urdf_xml()
{
  const auto share = ament_index_cpp::get_package_share_directory("lerobot_description");
  const std::string xacro_path = (std::filesystem::path(share) / "urdf" / "so101.urdf.xacro").string();
  // xacro CLI 起動 (process_xacro と同じ)
  std::string cmd = "xacro " + xacro_path;
  std::array<char, 4096> buf;
  std::string out;
  FILE * pipe = popen(cmd.c_str(), "r");
  ASSERT_NE(pipe, nullptr);
  while (fgets(buf.data(), buf.size(), pipe) != nullptr) {
    out += buf.data();
  }
  pclose(pipe);
  return out;
}
```

新規テストクラス:

```cpp
class TeleopIKRealUrdfTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    node_ = teleop_ik::TeleopIKNode::make_for_test(load_real_urdf_xml(), "gripper");
    ASSERT_NE(node_, nullptr);
  }
  std::unique_ptr<teleop_ik::TeleopIKNode> node_;
};
```

実 URDF を使ったテストを `TeleopIKRealUrdfTest` フィクスチャで追加 (内容は Python 版 `test_ik_node.py` と等価):

```cpp
TEST_F(TeleopIKRealUrdfTest, ClampJointsUsesUrdfPositionLimits)
{
  Eigen::VectorXd below = node_->model_.lowerPositionLimit - Eigen::VectorXd::Ones(node_->model_.nq);
  Eigen::VectorXd above = node_->model_.upperPositionLimit + Eigen::VectorXd::Ones(node_->model_.nq);
  EXPECT_TRUE(node_->clamp_joints(below).isApprox(node_->model_.lowerPositionLimit));
  EXPECT_TRUE(node_->clamp_joints(above).isApprox(node_->model_.upperPositionLimit));
}

TEST_F(TeleopIKRealUrdfTest, SolveIkReturnsNulloptForUnreachableTarget)
{
  Eigen::Vector3d unreachable(10.0, 10.0, 10.0);
  EXPECT_FALSE(node_->solve_ik(unreachable, node_->q_current_).has_value());
}

TEST_F(TeleopIKRealUrdfTest, SolveIkConvergesForReachablePositionTarget)
{
  pinocchio::forwardKinematics(node_->model_, node_->data_, node_->q_current_);
  pinocchio::updateFramePlacements(node_->model_, node_->data_);
  const Eigen::Vector3d current = node_->data_.oMf[node_->ee_frame_id_].translation();
  const Eigen::Vector3d target = current + Eigen::Vector3d(0.0, -0.01, 0.0);
  const auto result = node_->solve_ik(target, node_->q_current_);
  ASSERT_TRUE(result.has_value());
  EXPECT_GE(result->minCoeff(), node_->model_.lowerPositionLimit.minCoeff() - 1e-9);
  EXPECT_LE(result->maxCoeff(), node_->model_.upperPositionLimit.maxCoeff() + 1e-9);
}
```

- [ ] **Step 2: Python テストファイルを削除**

```bash
git rm ros2_ws/src/teleop_ik/test/test_coordinate_utils.py \
       ros2_ws/src/teleop_ik/test/test_ik_node.py \
       ros2_ws/src/teleop_ik/test/test_target_msg.py \
       ros2_ws/src/teleop_ik/test/test_qos.py \
       ros2_ws/src/teleop_ik/test/test_packaging.py
```

- [ ] **Step 3: ビルド & テスト**

```bash
cd ros2_ws
nix develop --command bash -c "colcon build --packages-select teleop_ik --symlink-install && \
  source install/setup.bash && \
  colcon test --packages-select teleop_ik --event-handlers console_direct+"
```

期待: 全 gtest PASS. 旧 pytest は消えているので実行されない.

- [ ] **Step 4: コミット**

```bash
git add ros2_ws/src/teleop_ik/test/test_ik_node_helpers.cpp
git commit -m "test(teleop_ik): cover solve_ik with real URDF; remove pytest"
```

---

## Task 19: Python ソースを完全削除 & `package.xml` 整理

**Files:**
- Delete: `ros2_ws/src/teleop_ik/teleop_ik/*.py`
- Modify: `ros2_ws/src/teleop_ik/package.xml`

- [ ] **Step 1: Python ソース削除**

```bash
git rm -r ros2_ws/src/teleop_ik/teleop_ik
```

- [ ] **Step 2: `package.xml` から Python 依存を削除**

`ros2_ws/src/teleop_ik/package.xml` を以下に置換:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
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

- [ ] **Step 3: CMakeLists.txt から `ament_cmake_pytest` 関連を削除**

CMakeLists.txt を開き、Python セットアップ・console script・`ament_cmake_pytest` の行をすべて削除する. 残すのは冒頭 `cmake_minimum_required` / `project`, `find_package(ament_cmake REQUIRED)`, `find_package(Python3 ...)` の削除, `rosidl_generate_interfaces`, C++ 関連ブロック, `ament_package()` のみ.

最終形 (目安):

```cmake
cmake_minimum_required(VERSION 3.16)
project(teleop_ik)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
  DEPENDENCIES geometry_msgs std_msgs
)

# lib
add_library(teleop_ik_core SHARED src/coordinate_utils.cpp)
target_include_directories(teleop_ik_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(teleop_ik_core Eigen3)
target_link_libraries(teleop_ik_core pinocchio::pinocchio)

# executables
add_executable(teleop_ik_node src/ik_node.cpp)
target_include_directories(teleop_ik_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(teleop_ik_node
  rclcpp geometry_msgs std_msgs sensor_msgs trajectory_msgs builtin_interfaces)
target_link_libraries(teleop_ik_node teleop_ik_core pinocchio::pinocchio)

add_executable(gamepad_teleop_node src/gamepad_node.cpp)
target_include_directories(gamepad_teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(gamepad_teleop_node rclcpp std_msgs sensor_msgs)
target_link_libraries(gamepad_teleop_node pinocchio::pinocchio)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(TARGETS
  teleop_ik_core teleop_ik_node gamepad_teleop_node
  EXPORT export_teleop_ikTargets
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_coordinate_utils_cpp test/test_coordinate_utils.cpp)
  target_link_libraries(test_coordinate_utils_cpp teleop_ik_core)
  ament_add_gtest(test_target_msg_cpp test/test_target_msg.cpp)
  ament_add_gtest(test_ik_node_helpers_cpp test/test_ik_node_helpers.cpp)
  target_link_libraries(test_ik_node_helpers_cpp teleop_ik_core)
  ament_add_gtest(test_gamepad_node_cpp test/test_gamepad_node.cpp)
  ament_target_dependencies(test_gamepad_node_cpp rclcpp sensor_msgs std_msgs)
  ament_add_gtest(test_qos_cpp test/test_qos.cpp)
  ament_target_dependencies(test_qos_cpp rclcpp)
  target_link_libraries(test_qos_cpp teleop_ik_core)
  ament_add_gtest(test_packaging_cpp test/test_packaging.cpp)
  ament_target_dependencies(test_packaging_cpp ament_index_cpp)
endif()

ament_package()
```

- [ ] **Step 4: ビルド & テスト (最終)**

```bash
cd ros2_ws
rm -rf build install log
nix develop --command bash -c "colcon build --symlink-install && \
  source install/setup.bash && \
  colcon test --packages-select teleop_ik --event-handlers console_direct+"
```

期待: 全 gtest PASS. 警告 0.

- [ ] **Step 5: コミット**

```bash
git add -A
git commit -m "refactor(teleop_ik): remove Python sources and finalize C++ build"
```

---

## Task 20: `python3Packages.pinocchio` の不要確認と削除 (Nix)

**Files:**
- Modify: `ros2_ws/nix/shell.nix` (必要に応じて)

- [ ] **Step 1: 他パッケージでの使用を `grep` 確認**

```bash
grep -r pinocchio ros2_ws/src --include='*.xml' --include='*.py' --include='*.cpp' --include='*.hpp'
```

期待: `teleop_ik/package.xml` のみがヒット (Python 版と C++ 版の宣言). C++ 版には `pkgs.pinocchio` (`pinocchio` のみ) を使うので, `python3Packages.pinocchio` は不要.

- [ ] **Step 2: `python3Packages.pinocchio` と `python3Packages.coal` を削除 (任意)**

`shell.nix` から以下を削除:

```nix
          python3Packages.coal
          python3Packages.pinocchio
```

- [ ] **Step 3: 評価が通ることを確認**

```bash
nix flake show --json 2>/dev/null | head -100
```

期待: エラーなし.

- [ ] **Step 4: 起動してビルド可能か確認**

```bash
cd ros2_ws
nix develop --command bash -c "colcon build --symlink-install && source install/setup.bash && colcon test --packages-select teleop_ik --event-handlers console_direct+"
```

期待: 全 gtest PASS.

- [ ] **Step 5: コミット**

```bash
git add ros2_ws/nix/shell.nix
git commit -m "build(nix): drop unused python3Packages.pinocchio and coal"
```

---

## Task 21: launch ファイルと YAML の無変更確認 & 起動 dry-run

**Files:**
- Read-only 確認: `ros2_ws/src/teleop_ik/launch/*.py`, `ros2_ws/src/teleop_ik/config/*.yaml`

- [ ] **Step 1: launch ファイルが C++ ノードを参照していることを確認**

```bash
grep -n "executable" ros2_ws/src/teleop_ik/launch/*.py
```

期待: `executable="ik_node"` と `executable="gamepad_node"` がヒット. そのまま動く.

- [ ] **Step 2: `teleop_ik.launch.py` を dry-run**

```bash
cd ros2_ws
nix develop --command bash -c "source install/setup.bash && \
  ros2 launch teleop_ik teleop_ik.launch.py urdf_path:=\$(ros2 pkg prefix lerobot_description)/share/lerobot_description/urdf/so101.urdf.xacro 2>&1 | head -20"
```

期待: ノードが起動し、URDF 読み込みと Pinocchio モデル構築のログが出ること. (数秒で Ctrl-C.)

- [ ] **Step 3: パラメータ公開の確認**

```bash
cd ros2_ws
nix develop --command bash -c "source install/setup.bash && \
  ros2 param list /teleop_ik_node 2>&1 | sort"
```

期待: 以下のパラメータが見える:
- `end_effector_frame`
- `position_scale`
- `ik_damping`
- `ik_max_iterations`
- `ik_tolerance`
- `trajectory_time_from_start`
- `unity_conversion`
- `stick_velocity_scale`
- `stick_deadzone`
- `stick_max_delta_per_msg`
- `stick_fallback_dt`
- `urdf_path`

- [ ] **Step 4: トピック一覧の確認**

```bash
nix develop --command bash -c "source install/setup.bash && \
  ros2 topic list 2>&1 | grep -E '(teleop|follower/arm|follower/gripper)'"
```

期待:
- `/teleop/active`
- `/teleop/target`
- `/teleop/gripper`
- `/follower/arm_controller/joint_trajectory`
- `/follower/gripper_controller/joint_trajectory`

- [ ] **Step 5: 確認のみ (変更なし) ならコミットなし**

OK なら次へ. NG なら launch / パラメータ名を修正.

---

## Task 22: PR 作成

- [ ] **Step 1: 差分全体確認**

```bash
git log --oneline main..HEAD
git diff --stat main..HEAD
```

期待: 2〜3 ダースのコミット (各 Task 1〜2 個). 差分が膨大になりすぎていないこと.

- [ ] **Step 2: 最終ビルド & テスト**

```bash
cd ros2_ws
nix develop --command bash -c "colcon build --symlink-install && \
  source install/setup.bash && \
  colcon test --packages-select teleop_ik --event-handlers console_direct+"
```

期待: 全 gtest PASS, 警告 0.

- [ ] **Step 3: `nix flake check` (可能な場合)**

```bash
nix flake check --no-build 2>&1 | tail -20
```

期待: エラーなし. (Linux のみ実行可能.)

- [ ] **Step 4: PR 作成**

```bash
git push -u origin feat/teleop-ik-cpp
gh pr create \
  --base main \
  --title "refactor(teleop_ik): rewrite from Python to C++" \
  --body "## 概要
- ros2_ws/src/teleop_ik を Python から C++ (rclcpp + Pinocchio 4.0) に書き換え
- Python 版ソース・pytest は完全削除
- テストは gtest (ament_cmake_gtest) に全面移植
- ROS 2 インターフェース契約 (ノード名・トピック・パラメータ) は完全互換
- launch / YAML 設定は無変更で動作

## Spec / Plan
- docs/superpowers/specs/2026-06-23-teleop-ik-cpp-rewrite-design.md
- docs/superpowers/plans/2026-06-23-teleop-ik-cpp-rewrite.md

## 検証
- colcon build 成功 (Linux / nix develop .#ros)
- colcon test 全 gtest PASS
- 起動 dry-run で /teleop/* トピックとパラメータ公開を確認
- 実機テストは未実施 (Linux 実機が必要)

## チェックリスト
- [x] colcon build 通過
- [x] colcon test 全 gtest 通過
- [x] launch ファイル無変更で動作
- [x] Python 依存削除"
```

- [ ] **Step 5: CI 結果を確認**

`gh pr checks` で CI ステータスを確認. 失敗があれば修正して push.

---

## 補足: 想定外の状況への対処

- **`pinocchio` CMake config が見つからない**: `find_package(pinocchio REQUIRED)` が失敗する場合, `pkg_check_modules(PINOCCHIO REQUIRED IMPORTED_TARGET pinocchio)` に切り替え, `pinocchio::pinocchio` を `PkgConfig::PINOCCHIO` に置換する.
- **`xacro` CLI が PATH にない**: Nix シェルで `which xacro` を確認し, なければ `shell.nix` に `python3Packages.xacro` が入っているか (Task 2 で `pinocchio` を入れただけでは python xacro は消えていないので問題ない).
- **ROS 2 QoS `Reliability` 列挙の値**: rclcpp のバージョンによって `Reliability::Reliable` / `BestEffort` を使う. `Reliable` には `reliable` 値を入れないこと.
- **`popen` のバッファ溢れ**: 4 KB バッファで問題ないが, 巨大な URDF では `fread` ループに切り替え.
- **`Eigen` の `Vector2d` を `q_seed[idx_q]` に代入するときの暗黙変換**: `q_seed` は `Eigen::VectorXd` なので `q_seed[idx_q_0] = wrist_init_pos_.y() + ...` で OK. 明示キャストは不要.
