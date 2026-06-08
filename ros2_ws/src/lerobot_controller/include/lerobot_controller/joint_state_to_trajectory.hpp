#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <string>
#include <vector>
#include <unordered_map>

namespace lerobot_controller {

class JointStateToTrajectory : public rclcpp::Node {
public:
  JointStateToTrajectory();

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;

  std::vector<std::string> arm_joints_;
  std::vector<std::string> gripper_joints_;
  double time_from_start_sec_;
};

}  // namespace lerobot_controller
