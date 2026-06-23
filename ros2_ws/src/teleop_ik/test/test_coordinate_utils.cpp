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
