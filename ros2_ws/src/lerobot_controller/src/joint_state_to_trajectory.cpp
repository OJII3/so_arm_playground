#include "lerobot_controller/joint_state_to_trajectory.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace lerobot_controller {

JointStateToTrajectory::JointStateToTrajectory()
    : Node("joint_state_to_trajectory") {

  // Declare parameters
  this->declare_parameter<std::string>("input_topic", "/leader/joint_states");
  this->declare_parameter<std::string>("arm_output_topic", "/follower/arm_controller/joint_trajectory");
  this->declare_parameter<std::string>("gripper_output_topic", "/follower/gripper_controller/joint_trajectory");
  this->declare_parameter<std::vector<std::string>>("arm_joints", {"1", "2", "3", "4", "5"});
  this->declare_parameter<std::vector<std::string>>("gripper_joints", {"6"});
  this->declare_parameter<double>("time_from_start_sec", 0.1);

  // Get parameters
  auto input_topic = this->get_parameter("input_topic").as_string();
  auto arm_output_topic = this->get_parameter("arm_output_topic").as_string();
  auto gripper_output_topic = this->get_parameter("gripper_output_topic").as_string();
  arm_joints_ = this->get_parameter("arm_joints").as_string_array();
  gripper_joints_ = this->get_parameter("gripper_joints").as_string_array();
  time_from_start_sec_ = this->get_parameter("time_from_start_sec").as_double();

  // Create subscriber
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      input_topic,
      10,
      std::bind(&JointStateToTrajectory::joint_state_callback, this, std::placeholders::_1));

  // Create publishers
  arm_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(arm_output_topic, 10);
  gripper_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(gripper_output_topic, 10);

  RCLCPP_INFO(this->get_logger(), "Joint state to trajectory converter started");
  RCLCPP_INFO(this->get_logger(), "  Input: %s", input_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "  Arm output: %s", arm_output_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "  Gripper output: %s", gripper_output_topic.c_str());
}

void JointStateToTrajectory::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  // Create a mapping from joint name to position
  std::unordered_map<std::string, double> joint_dict;
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    joint_dict[msg->name[i]] = msg->position[i];
  }

  // Publish arm trajectory
  std::vector<double> arm_positions;
  arm_positions.reserve(arm_joints_.size());
  bool arm_complete = true;
  for (const auto& joint : arm_joints_) {
    auto it = joint_dict.find(joint);
    if (it != joint_dict.end()) {
      arm_positions.push_back(it->second);
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Joint %s not found in joint_states", joint.c_str());
      arm_complete = false;
      break;
    }
  }

  if (arm_complete && arm_positions.size() == arm_joints_.size()) {
    auto arm_traj = trajectory_msgs::msg::JointTrajectory();
    arm_traj.header.stamp = this->now();
    arm_traj.joint_names = arm_joints_;

    auto point = trajectory_msgs::msg::JointTrajectoryPoint();
    point.positions = arm_positions;
    point.time_from_start.sec = static_cast<int32_t>(time_from_start_sec_);
    point.time_from_start.nanosec = static_cast<uint32_t>((time_from_start_sec_ - static_cast<int32_t>(time_from_start_sec_)) * 1e9);
    arm_traj.points.push_back(point);

    arm_pub_->publish(arm_traj);
  }

  // Publish gripper trajectory
  std::vector<double> gripper_positions;
  gripper_positions.reserve(gripper_joints_.size());
  bool gripper_complete = true;
  for (const auto& joint : gripper_joints_) {
    auto it = joint_dict.find(joint);
    if (it != joint_dict.end()) {
      gripper_positions.push_back(it->second);
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Joint %s not found in joint_states", joint.c_str());
      gripper_complete = false;
      break;
    }
  }

  if (gripper_complete && gripper_positions.size() == gripper_joints_.size()) {
    auto gripper_traj = trajectory_msgs::msg::JointTrajectory();
    gripper_traj.header.stamp = this->now();
    gripper_traj.joint_names = gripper_joints_;

    auto point = trajectory_msgs::msg::JointTrajectoryPoint();
    point.positions = gripper_positions;
    point.time_from_start.sec = static_cast<int32_t>(time_from_start_sec_);
    point.time_from_start.nanosec = static_cast<uint32_t>((time_from_start_sec_ - static_cast<int32_t>(time_from_start_sec_)) * 1e9);
    gripper_traj.points.push_back(point);

    gripper_pub_->publish(gripper_traj);
  }
}

}  // namespace lerobot_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lerobot_controller::JointStateToTrajectory>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
