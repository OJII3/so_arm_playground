// teleop_ik/test/test_ik_node_helpers.cpp
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include <Eigen/Core>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <rclcpp/rclcpp.hpp>

#include "teleop_ik/ik_node.hpp"

namespace
{

class RclcppEnvironment : public ::testing::Environment
{
 public:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

const auto g_env_register = ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

std::string load_urdf_xml()
{
  const char * path = std::getenv("TELEOP_IK_TEST_URDF_PATH");
  if (path == nullptr || path[0] == '\0') {
    return "";
  }
  std::ifstream f(path);
  if (!f.good()) {
    return "";
  }
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

class TeleopIKHelpersTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    const std::string urdf = load_urdf_xml();
    ASSERT_FALSE(urdf.empty()) << "Set TELEOP_IK_TEST_URDF_PATH to expanded URDF XML";
    node_ = teleop_ik::TeleopIKNode::make_for_test(urdf, "gripper");
    ASSERT_NE(node_, nullptr);
  }
  std::unique_ptr<teleop_ik::TeleopIKNode> node_;
};

}  // namespace

TEST_F(TeleopIKHelpersTest, ClampJointsClipsToModelLimits)
{
  const Eigen::VectorXd below = node_->model_.lowerPositionLimit.array() - 1.0;
  const Eigen::VectorXd above = node_->model_.upperPositionLimit.array() + 1.0;
  const Eigen::VectorXd out_below = node_->clamp_joints(below);
  const Eigen::VectorXd out_above = node_->clamp_joints(above);
  EXPECT_TRUE(out_below.isApprox(node_->model_.lowerPositionLimit));
  EXPECT_TRUE(out_above.isApprox(node_->model_.upperPositionLimit));
}

TEST_F(TeleopIKHelpersTest, ApplyStickDeadzoneZerosWithinZone)
{
  EXPECT_EQ(
      node_->apply_stick_deadzone(0.05, 0.05, 0.1),
      (std::pair<double, double>{0.0, 0.0}));
  // mag=0.5, deadzone=0.1 → scale=(0.5-0.1)/(0.5*0.9)=0.8888
  // → x=0.5*0.8888=0.4444
  const auto [x, y] = node_->apply_stick_deadzone(0.5, 0.0, 0.1);
  EXPECT_NEAR(x, 0.4444, 1e-3);
  EXPECT_NEAR(y, 0.0, 1e-9);
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

TEST_F(TeleopIKHelpersTest, SolveIkReturnsNulloptForUnreachableTarget)
{
  const Eigen::Vector3d unreachable(10.0, 10.0, 10.0);
  EXPECT_FALSE(node_->solve_ik(unreachable, node_->q_current_, 1e-6, 100, 1e-4).has_value());
}

TEST_F(TeleopIKHelpersTest, SolveIkReachesCurrentPosition)
{
  pinocchio::forwardKinematics(node_->model_, node_->data_, node_->q_current_);
  pinocchio::updateFramePlacements(node_->model_, node_->data_);
  const Eigen::Vector3d current_ee = node_->data_.oMf[node_->ee_frame_id_].translation();
  const auto result = node_->solve_ik(current_ee, node_->q_current_, 1e-6, 100, 1e-4);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR((*result - node_->q_current_).norm(), 0.0, 1e-3);
}

TEST_F(TeleopIKHelpersTest, SolveIkConvergesForReachablePositionTarget)
{
  pinocchio::forwardKinematics(node_->model_, node_->data_, node_->q_current_);
  pinocchio::updateFramePlacements(node_->model_, node_->data_);
  const Eigen::Vector3d current = node_->data_.oMf[node_->ee_frame_id_].translation();
  const Eigen::Vector3d target = current + Eigen::Vector3d(0.0, -0.01, 0.0);
  const auto result = node_->solve_ik(target, node_->q_current_, 1e-6, 100, 1e-4);
  ASSERT_TRUE(result.has_value());
  EXPECT_GE(result->minCoeff(), node_->model_.lowerPositionLimit.minCoeff() - 1e-9);
  EXPECT_LE(result->maxCoeff(), node_->model_.upperPositionLimit.maxCoeff() + 1e-9);
  // 実位置に到達したか検証.
  pinocchio::forwardKinematics(node_->model_, node_->data_, *result);
  pinocchio::updateFramePlacements(node_->model_, node_->data_);
  const Eigen::Vector3d actual = node_->data_.oMf[node_->ee_frame_id_].translation();
  EXPECT_LT((actual - target).norm(), 1e-4);
}

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

// ---- 統合 callback 系 ----

namespace
{
struct CallbacksFixture : public TeleopIKHelpersTest
{
  void SetUp() override
  {
    TeleopIKHelpersTest::SetUp();
    node_->active_ = true;
    node_->unity_anchor_set_ = true;
    node_->unity_anchor_pos_.setZero();
    node_->arm_init_pos_.setZero();
    node_->q_solution_ = node_->q_current_;
    node_->wrist_init_pos_.setZero();
    node_->integrated_stick_.setZero();
    node_->last_msg_stamp_.reset();
  }
  bool call_on_target_with_input(
      const geometry_msgs::msg::Pose & pose, float sx, float sy)
  {
    builtin_interfaces::msg::Time stamp;
    return node_->on_target_with_input(
        pose, sx, sy, stamp,
        /*position_scale=*/1.0,
        /*stick_velocity_scale=*/1.0,
        /*stick_deadzone=*/0.0,
        /*stick_max_delta_per_msg=*/10.0,
        /*stick_fallback_dt=*/0.1,
        /*unity_conversion=*/false,
        /*ik_damping=*/1e-6,
        /*ik_max_iterations=*/100,
        /*ik_tolerance=*/1e-4);
  }
};
}  // namespace

TEST_F(CallbacksFixture, OnTargetWithInputIntegratesStickPerMessage)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  call_on_target_with_input(pose, 1.0f, 0.5f);
  EXPECT_NEAR(node_->integrated_stick_.x(), 0.1, 1e-9);
  EXPECT_NEAR(node_->integrated_stick_.y(), 0.05, 1e-9);
}

TEST_F(CallbacksFixture, FirstMessageSetsAnchorWithoutIntegration)
{
  // anchor 未設定 (unity_anchor_set_=false) で呼び出すと,
  // スティック積分されず, integrated_stick はゼロのまま, IK も解かない.
  node_->unity_anchor_set_ = false;
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  const bool solved = call_on_target_with_input(pose, 1.0f, 0.5f);
  EXPECT_FALSE(solved);
  EXPECT_TRUE(node_->unity_anchor_set_);
  EXPECT_NEAR(node_->integrated_stick_.x(), 0.0, 1e-9);
  EXPECT_NEAR(node_->integrated_stick_.y(), 0.0, 1e-9);
}

TEST_F(CallbacksFixture, TargetUsesPreviousSuccessfulSolutionAsNextSeed)
{
  // 1 回目: anchor 設定 (pose = origin).
  geometry_msgs::msg::Pose pose_anchor;
  pose_anchor.position.x = 0.0;
  pose_anchor.position.y = 0.0;
  pose_anchor.position.z = 0.0;
  EXPECT_FALSE(call_on_target_with_input(pose_anchor, 0.0f, 0.0f));
  EXPECT_TRUE(node_->unity_anchor_set_);

  // 2 回目: 異なる target で IK を解かせ, q_solution_ が更新される.
  // on_target_with_input は内部で solve_ik を呼び, 結果は q_seed (q_solution_)
  // に書き戻される. 2 回目の seed には 1 回目の解が反映されているはず.
  const auto first_solution = node_->q_solution_;

  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 0.0;
  pose2.position.y = -0.05;  // 0 から動かす → IK が異なる解に収束する
  pose2.position.z = 0.0;
  EXPECT_TRUE(call_on_target_with_input(pose2, 0.0f, 0.0f));
  // q_solution_ が更新され, 前回解と差分がある.
  EXPECT_GT((node_->q_solution_ - first_solution).norm(), 1e-6);
}

TEST_F(CallbacksFixture, WristResetsOnSessionStart)
{
  // active_ を一旦 false に戻し, on_active(true) でセッション開始処理が
  // 走るようにしてから wrist 初期角のリセットを検証する.
  node_->active_ = false;
  node_->wrist_init_pos_ = Eigen::Vector2d(1.23, -0.45);
  // joint 4, 5 を q_current_ に直接設定.
  const auto idx_q_4 = node_->model_.joints[node_->model_.getJointId("4")].idx_q();
  const auto idx_q_5 = node_->model_.joints[node_->model_.getJointId("5")].idx_q();
  node_->q_current_[idx_q_4] = 0.3;
  node_->q_current_[idx_q_5] = -0.4;

  std_msgs::msg::Bool true_msg;
  true_msg.data = true;
  node_->on_active_msg(std::make_shared<std_msgs::msg::Bool>(true_msg));

  // wrist_init_pos_ は on_target_with_input のスティック軸マッピングと整合.
  // joint 4 (stick_y) → wrist_init_pos_.y()
  // joint 5 (stick_x) → wrist_init_pos_.x()
  EXPECT_NEAR(node_->wrist_init_pos_.y(), 0.3, 1e-9);
  EXPECT_NEAR(node_->wrist_init_pos_.x(), -0.4, 1e-9);
  EXPECT_NEAR(node_->integrated_stick_.x(), 0.0, 1e-9);
  EXPECT_NEAR(node_->integrated_stick_.y(), 0.0, 1e-9);
  EXPECT_TRUE(node_->active_);
}

TEST_F(CallbacksFixture, StickDeadzoneBlocksSmallInputs)
{
  // deadzone = 0.1, 入力 0.05 → 積分されない.
  node_->integrated_stick_.setZero();
  builtin_interfaces::msg::Time stamp;
  node_->on_target_with_input(
      []() {
        geometry_msgs::msg::Pose p;
        p.position.x = 0.0;
        p.position.y = 0.0;
        p.position.z = 0.0;
        return p;
      }(),
      0.05f, -0.05f, stamp,
      /*position_scale=*/1.0,
      /*stick_velocity_scale=*/1.0,
      /*stick_deadzone=*/0.1,
      /*stick_max_delta_per_msg=*/10.0,
      /*stick_fallback_dt=*/0.1,
      /*unity_conversion=*/false,
      /*ik_damping=*/1e-6,
      /*ik_max_iterations=*/100,
      /*ik_tolerance=*/1e-4);
  EXPECT_EQ(node_->integrated_stick_.x(), 0.0);
  EXPECT_EQ(node_->integrated_stick_.y(), 0.0);
}

TEST_F(CallbacksFixture, StickMaxDeltaPerMsgClampsIntegration)
{
  // stick=1.0, dt=0.1 (fallback), scale=1.0, cap=0.05 → 1 メッセージで 0.05 まで.
  builtin_interfaces::msg::Time stamp;
  node_->on_target_with_input(
      []() {
        geometry_msgs::msg::Pose p;
        p.position.x = 0.0;
        p.position.y = 0.0;
        p.position.z = 0.0;
        return p;
      }(),
      1.0f, 1.0f, stamp,
      /*position_scale=*/1.0,
      /*stick_velocity_scale=*/1.0,
      /*stick_deadzone=*/0.0,
      /*stick_max_delta_per_msg=*/0.05,
      /*stick_fallback_dt=*/1.0,
      /*unity_conversion=*/false,
      /*ik_damping=*/1e-6,
      /*ik_max_iterations=*/100,
      /*ik_tolerance=*/1e-4);
  EXPECT_NEAR(node_->integrated_stick_.x(), 0.05, 1e-9);
  EXPECT_NEAR(node_->integrated_stick_.y(), 0.05, 1e-9);
}

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

TEST_F(CallbacksFixture, OnTargetWithInputInjectsFkForJoint5)
{
  // セッション開始で q_current_ の joint 5 値を wrist_init_pos_.x() に保存.
  node_->active_ = false;
  const auto idx_q_5 = node_->model_.joints[node_->model_.getJointId("5")].idx_q();
  node_->q_current_[idx_q_5] = 0.5;
  std_msgs::msg::Bool true_msg;
  true_msg.data = true;
  node_->on_active_msg(std::make_shared<std_msgs::msg::Bool>(true_msg));
  ASSERT_NEAR(node_->wrist_init_pos_.x(), 0.5, 1e-9);

  // 1 回目: anchor 設定 (pose = origin).
  geometry_msgs::msg::Pose pose_anchor;
  pose_anchor.position.x = 0.0;
  pose_anchor.position.y = 0.0;
  pose_anchor.position.z = 0.0;
  EXPECT_FALSE(call_on_target_with_input(pose_anchor, 0.0f, 0.0f));

  // 2 回目: stick_x = 1.0 で 1 メッセージ分だけ積分.
  // CallbacksFixture では stick_velocity_scale=1.0, stick_fallback_dt=0.1, cap=10.0.
  // → integrated_stick_.x() = 0.1
  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 0.0;
  pose2.position.y = -0.05;
  pose2.position.z = 0.0;
  EXPECT_TRUE(call_on_target_with_input(pose2, 1.0f, 0.0f));

  // q_solution_[idx_q_5] = wrist_init_pos_.x() + integrated_stick_.x()
  //                       = 0.5 + 0.1 = 0.6 が publish される.
  EXPECT_NEAR(node_->q_solution_[idx_q_5], 0.6, 1e-6);
}

TEST_F(CallbacksFixture, OnTargetWithInputClampsFkJoint5ToLimit)
{
  // joint 5 の upper limit を取得し, wrist_init_pos_.x() を limit - 0.1 にセット.
  // integrated_stick_.x() を +1.0 に直接セット (limit を超える量).
  // → q_solution_[idx_q_5] は upperPositionLimit に clamp されるはず.
  node_->active_ = false;
  const auto idx_q_5 = node_->model_.joints[node_->model_.getJointId("5")].idx_q();
  const double upper_5 = node_->model_.upperPositionLimit[idx_q_5];
  node_->q_current_[idx_q_5] = upper_5 - 0.1;
  std_msgs::msg::Bool true_msg;
  true_msg.data = true;
  node_->on_active_msg(std::make_shared<std_msgs::msg::Bool>(true_msg));

  // 1 回目: anchor 設定.
  geometry_msgs::msg::Pose pose_anchor;
  pose_anchor.position.x = 0.0;
  pose_anchor.position.y = 0.0;
  pose_anchor.position.z = 0.0;
  EXPECT_FALSE(call_on_target_with_input(pose_anchor, 0.0f, 0.0f));

  // 2 回目: integrated_stick_ を limit を超える量に強制セットして IK を走らせる.
  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 0.0;
  pose2.position.y = -0.05;
  pose2.position.z = 0.0;
  node_->integrated_stick_.x() = 1.0;  // upper - 0.1 + 1.0 = upper + 0.9 → clamp
  EXPECT_TRUE(call_on_target_with_input(pose2, 0.0f, 0.0f));

  // q_solution_[idx_q_5] は upperPositionLimit にクランプされる.
  EXPECT_NEAR(node_->q_solution_[idx_q_5], upper_5, 1e-9);
}
