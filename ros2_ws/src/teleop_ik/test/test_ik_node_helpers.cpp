// teleop_ik/test/test_ik_node_helpers.cpp
#include <gtest/gtest.h>

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
  EXPECT_FALSE(node_->solve_ik(unreachable, node_->q_current_).has_value());
}

TEST_F(TeleopIKHelpersTest, SolveIkReachesCurrentPosition)
{
  pinocchio::forwardKinematics(node_->model_, node_->data_, node_->q_current_);
  pinocchio::updateFramePlacements(node_->model_, node_->data_);
  const Eigen::Vector3d current_ee = node_->data_.oMf[node_->ee_frame_id_].translation();
  const auto result = node_->solve_ik(current_ee, node_->q_current_);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR((*result - node_->q_current_).norm(), 0.0, 1e-3);
}

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
