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
