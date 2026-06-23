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
