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

#include <fcntl.h>
#include <sys/wait.h>
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
constexpr double kIkDt = 0.2;

// xacro を shell 経由ではなく execvp で安全に起動し, stdout を読み取る.
// パスが `.xacro` で終わらない場合は, すでに展開済みの URDF ファイルと
// みなしてファイル内容を直接読み込む (テストでも使用).
std::string process_xacro(const std::string & path)
{
  // .xacro 以外は展開済み URDF として読む.
  if (path.size() < 6 ||
      path.compare(path.size() - 6, 6, ".xacro") != 0)
  {
    std::ifstream f(path);
    if (!f.good()) {
      throw std::runtime_error("Failed to open URDF file: " + path);
    }
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
  }

  int pipefd[2];
  if (pipe(pipefd) < 0) {
    throw std::runtime_error("pipe() failed");
  }
  const pid_t pid = fork();
  if (pid < 0) {
    close(pipefd[0]);
    close(pipefd[1]);
    throw std::runtime_error("fork() failed");
  }
  if (pid == 0) {
    close(pipefd[0]);
    dup2(pipefd[1], STDOUT_FILENO);
    close(pipefd[1]);
    execlp("xacro", "xacro", path.c_str(), static_cast<char *>(nullptr));
    _exit(127);
  }
  close(pipefd[1]);
  std::array<char, 4096> buf{};
  std::string out;
  while (true) {
    const ssize_t n = read(pipefd[0], buf.data(), buf.size());
    if (n < 0) {
      if (errno == EINTR) continue;
      close(pipefd[0]);
      waitpid(pid, nullptr, 0);
      throw std::runtime_error("read() from xacro pipe failed");
    }
    if (n == 0) break;
    out.append(buf.data(), static_cast<size_t>(n));
  }
  close(pipefd[0]);
  int status = 0;
  waitpid(pid, &status, 0);
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    const int code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    throw std::runtime_error(
        "xacro CLI failed with code " + std::to_string(code));
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
  init_ros_node();
}

TeleopIKNode::TeleopIKNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("teleop_ik_node", options)
{
  init_ros_node();
}

void TeleopIKNode::init_ros_node()
{
  // パラメータ宣言
  this->declare_parameter<std::string>("urdf_path", "");
  this->declare_parameter<std::string>("end_effector_frame", "gripper");
  this->declare_parameter<double>("position_scale", 1.0);
  this->declare_parameter<double>("ik_damping", 1e-6);
  this->declare_parameter<int>("ik_max_iterations", 100);
  this->declare_parameter<double>("ik_tolerance", 1e-4);
  this->declare_parameter<double>("trajectory_time_from_start", 0.1);
  this->declare_parameter<bool>("unity_conversion", true);
  this->declare_parameter<double>("stick_velocity_scale", 1.5);
  this->declare_parameter<double>("stick_deadzone", 0.1);
  this->declare_parameter<double>("stick_max_delta_per_msg", 0.2);
  this->declare_parameter<double>("stick_fallback_dt", 0.0111);

  // URDF 読み込み & モデル構築
  const auto urdf_path = this->get_parameter("urdf_path").as_string();
  if (urdf_path.empty()) {
    RCLCPP_FATAL(get_logger(), "Parameter 'urdf_path' is required");
    throw std::runtime_error("Parameter 'urdf_path' is required");
  }
  const std::string urdf_xml = process_xacro(urdf_path);
  pinocchio::urdf::buildModelFromXML(urdf_xml, model_, false, false);
  data_ = pinocchio::Data(model_);
  q_current_ = pinocchio::neutral(model_);

  const auto ee_frame = this->get_parameter("end_effector_frame").as_string();
  if (!model_.existFrame(ee_frame)) {
    RCLCPP_FATAL(get_logger(), "Frame '%s' not found in URDF", ee_frame.c_str());
    throw std::runtime_error("Frame '" + ee_frame + "' not found in URDF");
  }
  ee_frame_id_ = model_.getFrameId(ee_frame);

  for (size_t i = 0; i < 5; ++i) {
    const std::string name = std::to_string(i + 1);
    if (!model_.existJointName(name)) {
      RCLCPP_FATAL(get_logger(), "Joint '%s' not found in URDF", name.c_str());
      throw std::runtime_error("Joint '" + name + "' not found in URDF");
    }
    arm_joint_ids_[i] = model_.getJointId(name);
  }
  // IK ソルバが触れる関節: joint 1, 2, 3, 4 (4DOF, 3D 位置ターゲットで 1 自由度冗長).
  for (size_t i = 0; i < 4; ++i) {
    position_joint_ids_[i] = arm_joint_ids_[i];
  }
  // FK 制御の関節: joint 5 のみ (stick_x の速度積分).
  wrist_joint_ids_[0] = arm_joint_ids_[4];

  RCLCPP_INFO(
      get_logger(), "Pinocchio model loaded: %d DOF, EE frame='%s' (id=%zu)",
      model_.nq, ee_frame.c_str(), static_cast<size_t>(ee_frame_id_));

  // QoS
  rclcpp::QoS target_qos(rclcpp::KeepLast(10));
  target_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  target_qos.durability(rclcpp::DurabilityPolicy::Volatile);

  sub_active_ = this->create_subscription<std_msgs::msg::Bool>(
      "/teleop/active", 10,
      std::bind(&TeleopIKNode::on_active_msg, this, std::placeholders::_1));
  sub_target_ = this->create_subscription<teleop_ik::msg::TargetPoseWithInput>(
      "/teleop/target", target_qos,
      std::bind(&TeleopIKNode::on_target_msg, this, std::placeholders::_1));
  sub_gripper_ = this->create_subscription<std_msgs::msg::Float64>(
      "/teleop/gripper", 10,
      std::bind(&TeleopIKNode::on_gripper_msg, this, std::placeholders::_1));
  sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/follower/joint_states", 10,
      std::bind(&TeleopIKNode::on_joint_states_msg, this, std::placeholders::_1));

  pub_arm_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/follower/arm_controller/joint_trajectory", 10);
  pub_gripper_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/follower/gripper_controller/joint_trajectory", 10);
}

std::unique_ptr<TeleopIKNode> TeleopIKNode::make_for_test(
    const std::string & urdf_xml, const std::string & ee_frame_name)
{
  // テスト用: デフォルトコンストラクタの重い body (URDF 読み込み等) をスキップ.
  auto node = std::unique_ptr<TeleopIKNode>(new TeleopIKNode(true));
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
  // IK ソルバ対象: joint 1, 2, 3, 4. FK: joint 5.
  for (size_t i = 0; i < 4; ++i) {
    node->position_joint_ids_[i] = node->arm_joint_ids_[i];
  }
  node->wrist_joint_ids_[0] = node->arm_joint_ids_[4];

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
    const Eigen::Vector3d & target_position, const Eigen::VectorXd & q_seed,
    double damping, int max_iter, double tol)
{
  std::vector<pinocchio::Index> position_velocity_indexes;
  position_velocity_indexes.reserve(position_joint_ids_.size());
  for (const auto jid : position_joint_ids_) {
    if (jid == static_cast<pinocchio::JointIndex>(-1)) continue;
    position_velocity_indexes.push_back(model_.joints[jid].idx_v());
  }

  Eigen::VectorXd q = clamp_joints(q_seed);
  for (int i = 0; i < max_iter; ++i) {
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    const Eigen::Vector3d err = target_position - data_.oMf[ee_frame_id_].translation();
    if (err.norm() < tol) {
      return clamp_joints(q);
    }
    Eigen::MatrixXd J_full = Eigen::MatrixXd::Zero(6, model_.nv);
    pinocchio::computeFrameJacobian(
        model_, data_, q, ee_frame_id_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_full);
    Eigen::MatrixXd J(3, position_velocity_indexes.size());
    for (Eigen::Index c = 0; c < static_cast<Eigen::Index>(position_velocity_indexes.size()); ++c) {
      J.col(c) = J_full.topRows(3).col(position_velocity_indexes[c]);
    }
    const Eigen::Matrix3d JJt = J * J.transpose() + damping * Eigen::Matrix3d::Identity();
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
      // wrist 初期角 = q_current_ の joint 4/5 値. 関節 4 → stick_y
      // (wrist_init_pos.y()), 関節 5 → stick_x (wrist_init_pos.x())
      // という軸マッピングは on_target_with_input と同じ.
      // joint 4 は arm_joint_ids_[3], joint 5 は wrist_joint_ids_[0] (= arm_joint_ids_[4]).
      wrist_init_pos_.setZero();
      const auto jid_4 = arm_joint_ids_[3];
      if (jid_4 != static_cast<pinocchio::JointIndex>(-1)) {
        const auto idx_q_4 = model_.joints[jid_4].idx_q();
        if (idx_q_4 >= 0 && static_cast<Eigen::Index>(idx_q_4) < q_current_.size()) {
          wrist_init_pos_.y() = q_current_[idx_q_4];  // stick_y → joint 4
        }
      }
      if (wrist_joint_ids_[0] != static_cast<pinocchio::JointIndex>(-1)) {
        const auto idx_q_5 = model_.joints[wrist_joint_ids_[0]].idx_q();
        if (idx_q_5 >= 0 && static_cast<Eigen::Index>(idx_q_5) < q_current_.size()) {
          wrist_init_pos_.x() = q_current_[idx_q_5];  // stick_x → joint 5
        }
      }
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

bool TeleopIKNode::on_target_with_input(
    const geometry_msgs::msg::Pose & pose,
    float stick_x, float stick_y,
    const builtin_interfaces::msg::Time & stamp,
    double position_scale,
    double stick_velocity_scale,
    double stick_deadzone,
    double stick_max_delta_per_msg,
    double stick_fallback_dt,
    bool unity_conversion,
    double ik_damping, int ik_max_iterations, double ik_tolerance)
{
  if (!active_) {
    return false;
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
    // 初回メッセージは anchor 設定のみで, IK 計算も publish もしない.
    unity_anchor_pos_ = ros_pos;
    unity_anchor_set_ = true;
    last_msg_stamp_ = stamp_to_time(stamp);
    return false;
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
  // joint 4 は IK ソルバの冗長 DOF. stick_y 積分値を seed の bias として渡す.
  if (arm_joint_ids_[3] != static_cast<pinocchio::JointIndex>(-1)) {
    const auto idx_q_4 = model_.joints[arm_joint_ids_[3]].idx_q();
    q_seed[idx_q_4] = wrist_init_pos_.y() + integrated_stick_.y();
  }
  // joint 5 は FK (position_joint_ids_ に含めない) なので q_seed で
  // 上書きしない. q_solution_ からの前回値 (= 直近の stick 積分値) を保持.
  q_seed = clamp_joints(q_seed);

  if (auto result = solve_ik(target_pos, q_seed, ik_damping, ik_max_iterations, ik_tolerance);
      result.has_value()) {
    q_solution_ = *result;
    // joint 5 はソルバ外. stick_x 由来の FK 値を q_solution_ に注入して
    // make_arm_trajectory がそのまま publish できるようにする.
    // 注入値は joint limit に必ず clamp する (旧実装の clamp_joints 経由と等価).
    if (wrist_joint_ids_[0] != static_cast<pinocchio::JointIndex>(-1)) {
      const auto idx_q_5 = model_.joints[wrist_joint_ids_[0]].idx_q();
      const double raw_5 = wrist_init_pos_.x() + integrated_stick_.x();
      q_solution_[idx_q_5] = std::clamp(
          raw_5,
          model_.lowerPositionLimit[idx_q_5],
          model_.upperPositionLimit[idx_q_5]);
    }
    return true;
  }
  // IK 失敗: 古い q_solution_ を維持し, publish しない.
  return false;
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

void TeleopIKNode::on_active_msg(const std_msgs::msg::Bool::SharedPtr msg)
{
  on_active(msg->data);
}

void TeleopIKNode::on_target_msg(const teleop_ik::msg::TargetPoseWithInput::SharedPtr msg)
{
  const bool solved = on_target_with_input(
      msg->pose, msg->stick_x, msg->stick_y, msg->header.stamp,
      this->get_parameter("position_scale").as_double(),
      this->get_parameter("stick_velocity_scale").as_double(),
      this->get_parameter("stick_deadzone").as_double(),
      this->get_parameter("stick_max_delta_per_msg").as_double(),
      this->get_parameter("stick_fallback_dt").as_double(),
      this->get_parameter("unity_conversion").as_bool(),
      this->get_parameter("ik_damping").as_double(),
      this->get_parameter("ik_max_iterations").as_int(),
      this->get_parameter("ik_tolerance").as_double());

  // IK が解けて初めて publish. anchor 設定 / IK 失敗時は publish しない.
  if (!solved) {
    return;
  }
  const double t = this->get_parameter("trajectory_time_from_start").as_double();
  pub_arm_->publish(make_arm_trajectory(q_solution_, t));
}

void TeleopIKNode::on_gripper_msg(const std_msgs::msg::Float64::SharedPtr msg)
{
  on_gripper(msg->data);
  if (!active_) {
    return;
  }
  const double value = std::clamp(msg->data, 0.0, 1.0);
  const double angle = kGripperLower + value * (kGripperUpper - kGripperLower);
  const double t = this->get_parameter("trajectory_time_from_start").as_double();
  pub_gripper_->publish(make_gripper_trajectory(angle, t));
}

void TeleopIKNode::on_joint_states_msg(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (i < msg->position.size()) {
      on_joint_state(msg->name[i], msg->position[i]);
    }
  }
}

}  // namespace teleop_ik

// main は ik_node_main.cpp に分離 (テストで lib を link 可能にするため).
