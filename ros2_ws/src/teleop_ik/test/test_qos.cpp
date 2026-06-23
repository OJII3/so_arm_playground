// teleop_ik/test/test_qos.cpp
#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "teleop_ik/gamepad_node.hpp"
#include "teleop_ik/ik_node.hpp"

class RclcppEnvironment : public ::testing::Environment
{
 public:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};
const auto g_env_register = ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

namespace
{
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
}  // namespace

class QosTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    const char * path = std::getenv("TELEOP_IK_TEST_URDF_PATH");
    ASSERT_NE(path, nullptr) << "Set TELEOP_IK_TEST_URDF_PATH to expanded URDF file path";
    gamepad_node_ = std::make_shared<teleop_ik::GamepadTeleopNode>();
    ik_node_ = std::make_shared<teleop_ik::TeleopIKNode>(rclcpp::NodeOptions()
      .parameter_overrides({rclcpp::Parameter("urdf_path", std::string(path))}));
    probe_ = std::make_shared<rclcpp::Node>("qos_probe");
  }
  void TearDown() override
  {
    probe_.reset();
    ik_node_.reset();
    gamepad_node_.reset();
  }
  static rclcpp::ReliabilityPolicy get_sub_reliability(
      rclcpp::Node::SharedPtr probe, const std::string & topic)
  {
    const auto endpoints = probe->get_subscriptions_info_by_topic(topic);
    EXPECT_EQ(endpoints.size(), 1u);
    return endpoints[0].qos_profile().reliability();
  }
  rclcpp::Node::SharedPtr gamepad_node_;
  rclcpp::Node::SharedPtr ik_node_;
  rclcpp::Node::SharedPtr probe_;
};

TEST_F(QosTest, ActiveSubscriptionIsReliable)
{
  EXPECT_EQ(
      get_sub_reliability(probe_, "/teleop/active"),
      rclcpp::ReliabilityPolicy::Reliable);
}

TEST_F(QosTest, TargetSubscriptionIsBestEffort)
{
  EXPECT_EQ(
      get_sub_reliability(probe_, "/teleop/target"),
      rclcpp::ReliabilityPolicy::BestEffort);
}

TEST_F(QosTest, GripperSubscriptionIsReliable)
{
  EXPECT_EQ(
      get_sub_reliability(probe_, "/teleop/gripper"),
      rclcpp::ReliabilityPolicy::Reliable);
}

TEST_F(QosTest, JoySubscriptionIsReliable)
{
  EXPECT_EQ(
      get_sub_reliability(probe_, "/joy"),
      rclcpp::ReliabilityPolicy::Reliable);
}
