// teleop_ik/test/test_qos.cpp
#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "teleop_ik/gamepad_node.hpp"

class RclcppEnvironment : public ::testing::Environment
{
 public:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};
const auto g_env_register = ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

class QosTest : public ::testing::Test
{
 protected:
  void SetUp() override
  {
    gamepad_node_ = std::make_shared<teleop_ik::GamepadTeleopNode>();
    probe_ = std::make_shared<rclcpp::Node>("qos_probe");
  }
  void TearDown() override
  {
    probe_.reset();
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
  rclcpp::Node::SharedPtr probe_;
};

TEST_F(QosTest, JoySubscriptionIsReliable)
{
  // gamepad_node は /joy を RELIABLE で購読する.
  EXPECT_EQ(
      get_sub_reliability(probe_, "/joy"),
      rclcpp::ReliabilityPolicy::Reliable);
}
