// teleop_ik/src/ik_node.cpp
#include "teleop_ik/ik_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <utility>

#include <unistd.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/rclcpp.hpp>

#include "teleop_ik/coordinate_utils.hpp"

namespace teleop_ik
{

namespace
{
constexpr double kGripperLower = -0.174533;
constexpr double kGripperUpper = 1.74533;
constexpr double kIkDamping = 1e-6;
constexpr int kIkMaxIterations = 100;
constexpr double kIkTolerance = 1e-4;
constexpr double kIkDt = 0.2;

std::string process_xacro(const std::string & xacro_path)
{
  std::string cmd = "xacro " + xacro_path;
  std::array<char, 4096> buf{};
  std::string out;
  FILE * pipe = popen(cmd.c_str(), "r");
  if (!pipe) {
    throw std::runtime_error("popen(xacro) failed for " + xacro_path);
  }
  while (fgets(buf.data(), buf.size(), pipe) != nullptr) {
    out += buf.data();
  }
  const int rc = pclose(pipe);
  if (rc != 0) {
    throw std::runtime_error("xacro CLI failed with code " + std::to_string(rc));
  }
  return out;
}

builtin_interfaces::msg::Duration seconds_to_duration(double seconds)
{
  const int sec = static_cast<int>(seconds);
  const int nanosec = static_cast<int>((seconds - sec) * 1e9);
  builtin_interfaces::msg::Duration d;
  d.sec = static_cast<int32_t>(sec);
  d.nanosec = static_cast<uint32_t>(nanosec);
  return d;
}
}  // namespace

TeleopIKNode::TeleopIKNode() : rclcpp::Node("teleop_ik_node")
{
}

std::unique_ptr<TeleopIKNode> TeleopIKNode::make_for_test(
    const std::string & urdf_xml, const std::string & ee_frame_name)
{
  auto node = std::unique_ptr<TeleopIKNode>(new TeleopIKNode());
  // urdfdom の XML パースが不安定なため, 一時ファイル経由で buildModel する.
  char tmpl[] = "/tmp/teleop_ik_urdfXXXXXX";
  const int fd = mkstemp(tmpl);
  if (fd < 0) {
    throw std::runtime_error("mkstemp failed");
  }
  const ssize_t n = write(fd, urdf_xml.data(), urdf_xml.size());
  if (n < 0 || static_cast<size_t>(n) != urdf_xml.size()) {
    close(fd);
    std::remove(tmpl);
    throw std::runtime_error("write to temp URDF failed");
  }
  close(fd);
  try {
    pinocchio::urdf::buildModel(tmpl, node->model_, false, false);
  } catch (...) {
    std::remove(tmpl);
    throw;
  }
  std::remove(tmpl);
  node->data_ = pinocchio::Data(node->model_);
  node->q_current_ = pinocchio::neutral(node->model_);

  for (size_t i = 0; i < 5; ++i) {
    const std::string name = std::to_string(i + 1);
    if (node->model_.existJointName(name)) {
      node->arm_joint_ids_[i] = node->model_.getJointId(name);
    } else {
      node->arm_joint_ids_[i] = static_cast<pinocchio::JointIndex>(-1);
    }
  }
  for (size_t i = 0; i < 3; ++i) {
    node->position_joint_ids_[i] = node->arm_joint_ids_[i];
  }
  node->wrist_joint_ids_[0] = node->arm_joint_ids_[3];
  node->wrist_joint_ids_[1] = node->arm_joint_ids_[4];

  if (!node->model_.existFrame(ee_frame_name)) {
    throw std::runtime_error("Frame '" + ee_frame_name + "' not found in URDF");
  }
  node->ee_frame_id_ = node->model_.getFrameId(ee_frame_name);
  return node;
}

Eigen::VectorXd TeleopIKNode::clamp_joints(const Eigen::VectorXd & q) const
{
  return q.cwiseMax(model_.lowerPositionLimit).cwiseMin(model_.upperPositionLimit);
}

std::pair<double, double> TeleopIKNode::apply_stick_deadzone(
    double x, double y, double deadzone) const
{
  const double mag = std::hypot(x, y);
  if (mag <= deadzone || mag < 1e-9 || deadzone >= 1.0) {
    return {0.0, 0.0};
  }
  const double scale = (mag - deadzone) / (mag * (1.0 - deadzone));
  return {x * scale, y * scale};
}

std::optional<double> TeleopIKNode::stamp_to_time(
    const builtin_interfaces::msg::Time & stamp) const
{
  const auto sec = static_cast<int64_t>(stamp.sec);
  const auto nsec = static_cast<int64_t>(stamp.nanosec);
  if (sec == 0 && nsec == 0) {
    return std::nullopt;
  }
  return static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
}

std::optional<Eigen::VectorXd> TeleopIKNode::solve_ik(
    const Eigen::Vector3d & target_position, const Eigen::VectorXd & q_seed)
{
  std::vector<pinocchio::Index> position_velocity_indexes;
  position_velocity_indexes.reserve(position_joint_ids_.size());
  for (const auto jid : position_joint_ids_) {
    if (jid == static_cast<pinocchio::JointIndex>(-1)) continue;
    position_velocity_indexes.push_back(model_.joints[jid].idx_v());
  }

  Eigen::VectorXd q = clamp_joints(q_seed);
  for (int i = 0; i < kIkMaxIterations; ++i) {
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    const Eigen::Vector3d err = target_position - data_.oMf[ee_frame_id_].translation();
    if (err.norm() < kIkTolerance) {
      return clamp_joints(q);
    }
    Eigen::MatrixXd J_full = Eigen::MatrixXd::Zero(6, model_.nv);
    pinocchio::computeFrameJacobian(
        model_, data_, q, ee_frame_id_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_full);
    Eigen::MatrixXd J(3, position_velocity_indexes.size());
    for (Eigen::Index c = 0; c < static_cast<Eigen::Index>(position_velocity_indexes.size()); ++c) {
      J.col(c) = J_full.topRows(3).col(position_velocity_indexes[c]);
    }
    const Eigen::Matrix3d JJt = J * J.transpose() + kIkDamping * Eigen::Matrix3d::Identity();
    const Eigen::VectorXd dq_pos = J.transpose() * JJt.ldlt().solve(err);
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(model_.nv);
    for (size_t k = 0; k < position_velocity_indexes.size(); ++k) {
      dq[position_velocity_indexes[k]] = dq_pos[k];
    }
    q = clamp_joints(pinocchio::integrate(model_, q, dq * kIkDt));
  }
  RCLCPP_WARN(get_logger(), "IK did not converge within max iterations");
  return std::nullopt;
}

void TeleopIKNode::on_active(bool active)
{
  if (active) {
    if (!active_) {
      pinocchio::forwardKinematics(model_, data_, q_current_);
      pinocchio::updateFramePlacements(model_, data_);
      arm_init_pos_ = data_.oMf[ee_frame_id_].translation();
      unity_anchor_set_ = false;
      q_solution_ = q_current_;
      wrist_init_pos_.setZero();
      integrated_stick_.setZero();
      last_msg_stamp_.reset();
      active_ = true;
    } else {
      integrated_stick_.setZero();
      last_msg_stamp_.reset();
    }
  } else if (active_) {
    active_ = false;
  }
}

void TeleopIKNode::on_joint_state(const std::string & name, double position)
{
  if (!model_.existJointName(name)) {
    return;
  }
  const auto jid = model_.getJointId(name);
  const auto idx_q = model_.joints[jid].idx_q();
  if (idx_q >= 0 && static_cast<Eigen::Index>(idx_q) < q_current_.size()) {
    q_current_[idx_q] = position;
  }
}

void TeleopIKNode::on_target_with_input(
    const geometry_msgs::msg::Pose & pose,
    float stick_x, float stick_y,
    const builtin_interfaces::msg::Time & stamp,
    double position_scale,
    double stick_velocity_scale,
    double stick_deadzone,
    double stick_max_delta_per_msg,
    double stick_fallback_dt,
    bool unity_conversion)
{
  if (!active_) {
    return;
  }

  Eigen::Vector3d ros_pos;
  if (unity_conversion) {
    ros_pos = teleop_ik::unity_position_to_ros(
        pose.position.x, pose.position.y, pose.position.z, position_scale);
  } else {
    ros_pos << pose.position.x * position_scale,
               pose.position.y * position_scale,
               pose.position.z * position_scale;
  }

  if (!unity_anchor_set_) {
    unity_anchor_pos_ = ros_pos;
    unity_anchor_set_ = true;
    last_msg_stamp_ = stamp_to_time(stamp);
    return;
  }

  const Eigen::Vector3d delta = ros_pos - unity_anchor_pos_;
  const Eigen::Vector3d target_pos = arm_init_pos_ + delta;

  const auto now = stamp_to_time(stamp);
  double delta_t;
  if (!last_msg_stamp_.has_value() || !now.has_value()) {
    delta_t = stick_fallback_dt;
  } else {
    delta_t = *now - *last_msg_stamp_;
    if (delta_t <= 0.0 || delta_t > 0.5) {
      delta_t = stick_fallback_dt;
    }
  }
  last_msg_stamp_ = now;

  const auto [vx, vy] = apply_stick_deadzone(stick_x, stick_y, stick_deadzone);
  const double cap_v = stick_max_delta_per_msg / std::max(stick_velocity_scale * delta_t, 1e-6);
  const double vx_c = std::clamp(vx, -cap_v, cap_v);
  const double vy_c = std::clamp(vy, -cap_v, cap_v);
  integrated_stick_.x() += vx_c * stick_velocity_scale * delta_t;
  integrated_stick_.y() += vy_c * stick_velocity_scale * delta_t;

  Eigen::VectorXd q_seed = q_solution_;
  if (wrist_joint_ids_[0] != static_cast<pinocchio::JointIndex>(-1)) {
    const auto idx_q_0 = model_.joints[wrist_joint_ids_[0]].idx_q();
    q_seed[idx_q_0] = wrist_init_pos_.y() + integrated_stick_.y();
  }
  if (wrist_joint_ids_[1] != static_cast<pinocchio::JointIndex>(-1)) {
    const auto idx_q_1 = model_.joints[wrist_joint_ids_[1]].idx_q();
    q_seed[idx_q_1] = wrist_init_pos_.x() + integrated_stick_.x();
  }
  q_seed = clamp_joints(q_seed);

  if (auto result = solve_ik(target_pos, q_seed); result.has_value()) {
    q_solution_ = *result;
  }
}

void TeleopIKNode::on_gripper(double value)
{
  (void)value;
  // publish 呼び出しは ROS 2 wiring (Task 14) で配線する.
}

trajectory_msgs::msg::JointTrajectory TeleopIKNode::make_arm_trajectory(
    const Eigen::VectorXd & q, double trajectory_time_from_start) const
{
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = {"1", "2", "3", "4", "5"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  for (const auto jid : arm_joint_ids_) {
    if (jid == static_cast<pinocchio::JointIndex>(-1)) continue;
    const auto idx_q = model_.joints[jid].idx_q();
    point.positions.push_back(q[idx_q]);
  }
  point.time_from_start = seconds_to_duration(trajectory_time_from_start);
  traj.points.push_back(point);
  return traj;
}

trajectory_msgs::msg::JointTrajectory TeleopIKNode::make_gripper_trajectory(
    double angle, double trajectory_time_from_start) const
{
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = {"6"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.push_back(angle);
  point.time_from_start = seconds_to_duration(trajectory_time_from_start);
  traj.points.push_back(point);
  return traj;
}

}  // namespace teleop_ik
