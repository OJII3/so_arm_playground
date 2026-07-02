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
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <teleop_ik/msg/reset_command.hpp>
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

TEST_F(TeleopIKHelpersTest, SolveIkKeepsWristJointsFixed)
{
  // joint 4, 5 はどちらもソルバ外 (FK 制御) なので seed 値が保持される.
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
  // joint 4, 5 はソルバ外なので seed 値が保持される.
  EXPECT_NEAR((*result)[idx_q_4], 0.2, 1e-9);
  EXPECT_NEAR((*result)[idx_q_5], -0.3, 1e-9);
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
      const geometry_msgs::msg::Pose & pose, float sx, float sy,
      bool ik_active = true)
  {
    builtin_interfaces::msg::Time stamp;
    return node_->on_target_with_input(
        pose, sx, sy, stamp,
        /*ik_active=*/ik_active,
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

struct ResetFixture : public TeleopIKHelpersTest
{
  void SetUp() override
  {
    const char * path = std::getenv("TELEOP_IK_TEST_URDF_PATH");
    ASSERT_NE(path, nullptr) << "Set TELEOP_IK_TEST_URDF_PATH to expanded URDF file path";
    rclcpp::NodeOptions opts;
    opts.parameter_overrides().push_back(rclcpp::Parameter("urdf_path", std::string(path)));
    for (size_t i = 0; i < 6; ++i) {
      const std::string name = "home_j" + std::to_string(i + 1) + "_rad";
      opts.parameter_overrides().push_back(
          rclcpp::Parameter(name, 0.1 * static_cast<double>(i + 1)));
    }
    opts.parameter_overrides().push_back(
        rclcpp::Parameter("reset_duration_sec", 1.5));
    node_ = std::make_shared<teleop_ik::TeleopIKNode>(opts);
    ASSERT_NE(node_, nullptr);
  }
  std::shared_ptr<teleop_ik::TeleopIKNode> node_;
};

struct CapturedTrajectories
{
  std::vector<trajectory_msgs::msg::JointTrajectory> arm;
  std::vector<trajectory_msgs::msg::JointTrajectory> gripper;
};

CapturedTrajectories capture_reset(
    const std::shared_ptr<teleop_ik::TeleopIKNode> & node,
    const teleop_ik::msg::ResetCommand & msg)
{
  auto probe = std::make_shared<rclcpp::Node>("reset_probe");
  CapturedTrajectories out;
  auto arm_sub = probe->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/follower/arm_controller/joint_trajectory", 10,
      [&](trajectory_msgs::msg::JointTrajectory::SharedPtr m) { out.arm.push_back(*m); });
  auto gripper_sub = probe->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/follower/gripper_controller/joint_trajectory", 10,
      [&](trajectory_msgs::msg::JointTrajectory::SharedPtr m) { out.gripper.push_back(*m); });

  auto wait_for = [&](auto predicate) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(probe);
      if (predicate()) return true;
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    return predicate();
  };

  wait_for([&]() {
    return probe->count_publishers("/follower/arm_controller/joint_trajectory") > 0 &&
           probe->count_publishers("/follower/gripper_controller/joint_trajectory") > 0;
  });

  node->on_reset_msg(std::make_shared<teleop_ik::msg::ResetCommand>(msg));

  wait_for([&]() {
    return out.arm.size() >= 1 && out.gripper.size() >= 1;
  });

  rclcpp::spin_some(probe);

  return out;
}
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
      /*ik_active=*/true,
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
      /*ik_active=*/true,
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

TEST_F(TeleopIKHelpersTest, SolveIkHasThreePositionJoints)
{
  // IK ソルバ対象: joint 1, 2, 3 (3DOF, 3D 位置ターゲット).
  EXPECT_EQ(node_->position_joint_ids_.size(), 3u);
}

TEST_F(TeleopIKHelpersTest, SolveIkKeepsJoint4Fixed)
{
  // joint 4 は IK ソルバ外 (FK 制御) なので seed 値が保持される.
  const auto jid_4 = node_->model_.getJointId("4");
  EXPECT_EQ(
      std::find(
          node_->position_joint_ids_.begin(),
          node_->position_joint_ids_.end(),
          jid_4),
      node_->position_joint_ids_.end());

  const auto idx_q_4 = node_->model_.joints[jid_4].idx_q();
  Eigen::VectorXd seed = node_->q_current_;
  seed[idx_q_4] = 0.3;
  pinocchio::forwardKinematics(node_->model_, node_->data_, seed);
  pinocchio::updateFramePlacements(node_->model_, node_->data_);
  const Eigen::Vector3d target = node_->data_.oMf[node_->ee_frame_id_].translation() +
    Eigen::Vector3d(0.0, -0.005, 0.0);
  const auto result = node_->solve_ik(target, seed, 1e-6, 100, 1e-4);
  ASSERT_TRUE(result.has_value());
  // joint 4 はソルバ外なので seed 値が保持される.
  EXPECT_NEAR((*result)[idx_q_4], 0.3, 1e-9);
}

TEST_F(CallbacksFixture, IkInactiveFreezesPositionAndMovesWrist)
{
  node_->unity_anchor_set_ = false;

  geometry_msgs::msg::Pose anchor_pose;
  anchor_pose.position.x = 0.0;
  anchor_pose.position.y = 0.0;
  anchor_pose.position.z = 0.0;
  EXPECT_FALSE(call_on_target_with_input(anchor_pose, 0.0f, 0.0f, true));
  EXPECT_TRUE(node_->unity_anchor_set_);

  const auto & model = node_->model_;
  const auto jid_4 = model.getJointId("4");
  const auto jid_5 = model.getJointId("5");
  ASSERT_NE(jid_4, pinocchio::JointIndex(-1));
  ASSERT_NE(jid_5, pinocchio::JointIndex(-1));
  const auto idx_q_4 = model.joints[jid_4].idx_q();
  const auto idx_q_5 = model.joints[jid_5].idx_q();
  ASSERT_GE(idx_q_4, 0);
  ASSERT_GE(idx_q_5, 0);

  pinocchio::forwardKinematics(node_->model_, node_->data_, node_->q_solution_);
  pinocchio::updateFramePlacements(node_->model_, node_->data_);
  const Eigen::Vector3d before_pos = node_->data_.oMf[node_->ee_frame_id_].translation();

  geometry_msgs::msg::Pose moved_pose;
  moved_pose.position.x = 0.1;
  moved_pose.position.y = 0.0;
  moved_pose.position.z = 0.0;
  EXPECT_TRUE(call_on_target_with_input(moved_pose, 0.5f, 0.3f, false));

  pinocchio::forwardKinematics(node_->model_, node_->data_, node_->q_solution_);
  pinocchio::updateFramePlacements(node_->model_, node_->data_);
  const Eigen::Vector3d after_pos = node_->data_.oMf[node_->ee_frame_id_].translation();
  EXPECT_LT((after_pos - before_pos).norm(), 1e-4);

  EXPECT_NEAR(node_->integrated_stick_.x(), 0.05, 1e-6);
  EXPECT_NEAR(node_->integrated_stick_.y(), 0.03, 1e-6);

  EXPECT_NEAR(node_->q_solution_[idx_q_4], 0.0 + 0.03, 1e-6);
  EXPECT_NEAR(node_->q_solution_[idx_q_5], 0.0 + 0.05, 1e-6);
}

TEST_F(CallbacksFixture, IkModeReentryResetsAnchorToPreventJump)
{
  // 1st: anchor at origin (ik_active=true)
  geometry_msgs::msg::Pose pose1;
  pose1.position.x = 0.0; pose1.position.y = 0.0; pose1.position.z = 0.0;
  EXPECT_FALSE(call_on_target_with_input(pose1, 0.0f, 0.0f, true));

  // 2nd: enter wrist mode, drift controller to (0, -0.1, 0) - position ignored in wrist mode
  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 0.0; pose2.position.y = -0.1; pose2.position.z = 0.0;
  EXPECT_TRUE(call_on_target_with_input(pose2, 0.0f, 0.0f, false));

  // 3rd: return to IK mode at same drifted position (0, -0.1, 0)
  // The anchor reset should absorb the drift: unity_anchor_pos_ = (0, -0.1, 0)
  // so delta = (0) and target = arm_init (no jump).
  call_on_target_with_input(pose2, 0.0f, 0.0f, true);

  // Verify unity_anchor_pos_ was reset to current ros_pos (0, -0.1, 0)
  EXPECT_NEAR(node_->unity_anchor_pos_.x(), 0.0, 1e-9);
  EXPECT_NEAR(node_->unity_anchor_pos_.y(), -0.1, 1e-9);
  EXPECT_NEAR(node_->unity_anchor_pos_.z(), 0.0, 1e-9);
}

TEST_F(CallbacksFixture, OnTargetWithInputInjectsFkForWristJoints)
{
  // セッション開始で q_current_ の joint 4, 5 値を wrist_init_pos_ に保存.
  node_->active_ = false;
  const auto idx_q_4 = node_->model_.joints[node_->model_.getJointId("4")].idx_q();
  const auto idx_q_5 = node_->model_.joints[node_->model_.getJointId("5")].idx_q();
  node_->q_current_[idx_q_4] = 0.3;
  node_->q_current_[idx_q_5] = 0.5;
  std_msgs::msg::Bool true_msg;
  true_msg.data = true;
  node_->on_active_msg(std::make_shared<std_msgs::msg::Bool>(true_msg));
  ASSERT_NEAR(node_->wrist_init_pos_.y(), 0.3, 1e-9);
  ASSERT_NEAR(node_->wrist_init_pos_.x(), 0.5, 1e-9);

  // 1 回目: anchor 設定 (pose = origin).
  geometry_msgs::msg::Pose pose_anchor;
  pose_anchor.position.x = 0.0;
  pose_anchor.position.y = 0.0;
  pose_anchor.position.z = 0.0;
  EXPECT_FALSE(call_on_target_with_input(pose_anchor, 0.0f, 0.0f));

  // 2 回目: stick_x = 1.0, stick_y = 0.5 で積分.
  // CallbacksFixture では stick_velocity_scale=1.0, stick_fallback_dt=0.1, cap=10.0.
  // → integrated_stick_.x() = 0.1, integrated_stick_.y() = 0.05
  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 0.0;
  pose2.position.y = -0.05;
  pose2.position.z = 0.0;
  EXPECT_TRUE(call_on_target_with_input(pose2, 1.0f, 0.5f));

  // q_solution_[idx_q_4] = wrist_init_pos_.y() + integrated_stick_.y()
  //                       = 0.3 + 0.05 = 0.35
  EXPECT_NEAR(node_->q_solution_[idx_q_4], 0.35, 1e-6);
  // q_solution_[idx_q_5] = wrist_init_pos_.x() + integrated_stick_.x()
  //                       = 0.5 + 0.1 = 0.6
  EXPECT_NEAR(node_->q_solution_[idx_q_5], 0.6, 1e-6);
}

TEST_F(CallbacksFixture, OnTargetWithInputClampsFkForWristJointsToLimit)
{
  // joint 4, 5 の upper limit を超える integrated_stick_ を与え, clamp を検証.
  node_->active_ = false;
  const auto idx_q_4 = node_->model_.joints[node_->model_.getJointId("4")].idx_q();
  const auto idx_q_5 = node_->model_.joints[node_->model_.getJointId("5")].idx_q();
  const double upper_4 = node_->model_.upperPositionLimit[idx_q_4];
  const double upper_5 = node_->model_.upperPositionLimit[idx_q_5];
  node_->q_current_[idx_q_4] = upper_4 - 0.1;
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

  // 2 回目: integrated_stick_ を limit を超える量に強制セット.
  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 0.0;
  pose2.position.y = -0.05;
  pose2.position.z = 0.0;
  node_->integrated_stick_.x() = 1.0;
  node_->integrated_stick_.y() = 1.0;
  EXPECT_TRUE(call_on_target_with_input(pose2, 0.0f, 0.0f));

  // q_solution_ は upperPositionLimit にクランプされる.
  EXPECT_NEAR(node_->q_solution_[idx_q_4], upper_4, 1e-9);
  EXPECT_NEAR(node_->q_solution_[idx_q_5], upper_5, 1e-9);
}

// ---- ResetCommand 経路 ----

TEST_F(ResetFixture, OnResetUsesParamDefaultsForAllNaN)
{
  teleop_ik::msg::ResetCommand msg;
  for (size_t i = 0; i < 6; ++i) {
    msg.home_joints[i] = std::numeric_limits<float>::quiet_NaN();
  }
  msg.duration_sec = 0.0f;

  const auto capt = capture_reset(node_, msg);

  ASSERT_EQ(capt.arm.size(), 1u);
  ASSERT_EQ(capt.arm[0].points.size(), 1u);
  ASSERT_EQ(capt.arm[0].points[0].positions.size(), 5u);
  EXPECT_NEAR(capt.arm[0].points[0].positions[0], 0.1, 1e-6);
  EXPECT_NEAR(capt.arm[0].points[0].positions[1], 0.2, 1e-6);
  EXPECT_NEAR(capt.arm[0].points[0].positions[2], 0.3, 1e-6);
  EXPECT_NEAR(capt.arm[0].points[0].positions[3], 0.4, 1e-6);
  EXPECT_NEAR(capt.arm[0].points[0].positions[4], 0.5, 1e-6);
  ASSERT_EQ(capt.gripper.size(), 1u);
  ASSERT_EQ(capt.gripper[0].points.size(), 1u);
  ASSERT_EQ(capt.gripper[0].points[0].positions.size(), 1u);
  EXPECT_NEAR(capt.gripper[0].points[0].positions[0], 0.6, 1e-6);

  EXPECT_FALSE(node_->active_);
}

TEST_F(ResetFixture, OnResetPartialOverride)
{
  teleop_ik::msg::ResetCommand msg;
  msg.home_joints[0] = 0.7f;
  for (size_t i = 1; i < 6; ++i) {
    msg.home_joints[i] = std::numeric_limits<float>::quiet_NaN();
  }
  msg.duration_sec = 0.0f;

  const auto capt = capture_reset(node_, msg);

  ASSERT_EQ(capt.arm.size(), 1u);
  ASSERT_EQ(capt.arm[0].points.size(), 1u);
  ASSERT_EQ(capt.arm[0].points[0].positions.size(), 5u);
  EXPECT_NEAR(capt.arm[0].points[0].positions[0], 0.7, 1e-6);
  EXPECT_NEAR(capt.arm[0].points[0].positions[1], 0.2, 1e-6);
  EXPECT_NEAR(capt.arm[0].points[0].positions[2], 0.3, 1e-6);
  EXPECT_NEAR(capt.arm[0].points[0].positions[3], 0.4, 1e-6);
  EXPECT_NEAR(capt.arm[0].points[0].positions[4], 0.5, 1e-6);

  EXPECT_FALSE(node_->active_);
}

TEST_F(ResetFixture, OnResetUsesProvidedDuration)
{
  teleop_ik::msg::ResetCommand msg;
  msg.duration_sec = 0.5f;

  const auto capt = capture_reset(node_, msg);

  ASSERT_EQ(capt.arm.size(), 1u);
  ASSERT_EQ(capt.arm[0].points.size(), 1u);
  EXPECT_NEAR(
      rclcpp::Duration(capt.arm[0].points[0].time_from_start).seconds(), 0.5, 1e-6);
  ASSERT_EQ(capt.gripper.size(), 1u);
  ASSERT_EQ(capt.gripper[0].points.size(), 1u);
  EXPECT_NEAR(
      rclcpp::Duration(capt.gripper[0].points[0].time_from_start).seconds(), 0.5, 1e-6);

  EXPECT_FALSE(node_->active_);
}

TEST_F(ResetFixture, OnResetDurationFallsBackOnZeroOrNaN)
{
  {
    teleop_ik::msg::ResetCommand msg;
    msg.duration_sec = 0.0f;

    const auto capt = capture_reset(node_, msg);

    ASSERT_EQ(capt.arm.size(), 1u);
    EXPECT_NEAR(
        rclcpp::Duration(capt.arm[0].points[0].time_from_start).seconds(), 1.5, 1e-6);
  }
  {
    teleop_ik::msg::ResetCommand msg;
    msg.duration_sec = std::numeric_limits<float>::quiet_NaN();

    const auto capt = capture_reset(node_, msg);

    ASSERT_EQ(capt.arm.size(), 1u);
    EXPECT_NEAR(
        rclcpp::Duration(capt.arm[0].points[0].time_from_start).seconds(), 1.5, 1e-6);
  }

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
  teleop_ik::msg::ResetCommand msg;
  for (size_t i = 0; i < 6; ++i) {
    msg.home_joints[i] = std::numeric_limits<float>::quiet_NaN();
  }
  msg.duration_sec = 0.0f;

  const auto capt = capture_reset(node_, msg);

  ASSERT_EQ(capt.arm.size(), 1u);
  ASSERT_EQ(capt.gripper.size(), 1u);
  ASSERT_EQ(capt.arm[0].joint_names.size(), 5u);
  ASSERT_EQ(capt.arm[0].points.size(), 1u);
  EXPECT_EQ(capt.arm[0].points[0].positions.size(), 5u);
  EXPECT_NEAR(capt.arm[0].points[0].positions[0], 0.1, 1e-6);
  EXPECT_NEAR(capt.arm[0].points[0].positions[1], 0.2, 1e-6);
  EXPECT_NEAR(capt.arm[0].points[0].positions[2], 0.3, 1e-6);
  EXPECT_NEAR(capt.arm[0].points[0].positions[3], 0.4, 1e-6);
  EXPECT_NEAR(capt.arm[0].points[0].positions[4], 0.5, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(capt.arm[0].points[0].time_from_start).seconds(), 1.5, 1e-6);
  ASSERT_EQ(capt.gripper[0].joint_names.size(), 1u);
  ASSERT_EQ(capt.gripper[0].points.size(), 1u);
  EXPECT_EQ(capt.gripper[0].points[0].positions.size(), 1u);
  EXPECT_NEAR(capt.gripper[0].points[0].positions[0], 0.6, 1e-6);
}

TEST_F(ResetFixture, OnResetDoesNotTouchSolution)
{
  node_->q_solution_ = node_->q_current_;

  teleop_ik::msg::ResetCommand msg;
  for (size_t i = 0; i < 6; ++i) {
    msg.home_joints[i] = std::numeric_limits<float>::quiet_NaN();
  }
  node_->on_reset_msg(std::make_shared<teleop_ik::msg::ResetCommand>(msg));

  EXPECT_NEAR((node_->q_solution_ - node_->q_current_).norm(), 0.0, 1e-9);
}
