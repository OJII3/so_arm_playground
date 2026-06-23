// teleop_ik/test/test_gamepad_node.cpp
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "teleop_ik/gamepad_node.hpp"

namespace
{
class RclcppEnvironment : public ::testing::Environment
{
 public:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};
const auto g_env_register = ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);
}  // namespace

TEST(GamepadNode, TogglesActiveOnRisingEdge)
{
  auto node = std::make_shared<teleop_ik::GamepadTeleopNode>();
  EXPECT_FALSE(node->active_);

  sensor_msgs::msg::Joy joy;
  joy.axes = {0.0, 0.0, 0.0, 0.0};
  joy.buttons.resize(16);
  // Cross ON
  joy.buttons[0] = 1;
  node->latest_joy_ = joy;
  node->timer_tick();
  EXPECT_TRUE(node->active_);

  // Cross OFF
  joy.buttons[0] = 0;
  node->latest_joy_ = joy;
  node->timer_tick();

  // Cross ON again
  joy.buttons[0] = 1;
  node->latest_joy_ = joy;
  node->timer_tick();
  EXPECT_FALSE(node->active_);
}
