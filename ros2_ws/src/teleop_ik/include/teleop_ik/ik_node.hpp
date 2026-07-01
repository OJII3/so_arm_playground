// teleop_ik/include/teleop_ik/ik_node.hpp
#ifndef TELEOP_IK__IK_NODE_HPP_
#define TELEOP_IK__IK_NODE_HPP_

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <teleop_ik/msg/reset_command.hpp>
#include <teleop_ik/msg/target_pose_with_input.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace teleop_ik
{

class TeleopIKNode : public rclcpp::Node
{
 public:
  // テスト用ファクトリ. URDF XML を直接渡して部分初期化する.
  // rclcpp::init/shutdown は呼び出し側で完了済み前提.
  static std::unique_ptr<TeleopIKNode> make_for_test(
      const std::string & urdf_xml, const std::string & ee_frame_name = "ee");

  TeleopIKNode();
  // NodeOptions 指定版: パラメータオーバーライド (urdf_path など) を反映.
  explicit TeleopIKNode(const rclcpp::NodeOptions & options);
  // テスト用: デフォルトコンストラクタをスキップして rclcpp::Node だけ構築.
  // bool 引数はオーバーロード曖昧さ回避のためのダミー.
  explicit TeleopIKNode(bool /*test_mode*/, const std::string & node_name = "teleop_ik_node")
      : rclcpp::Node(node_name) {}

 private:
  void on_reset(const teleop_ik::msg::ResetCommand & msg);

  // コンストラクタ初期化の本体. デフォルト / NodeOptions 両方から共有.
  void init_ros_node();

 public:

  // 純粋ヘルパー (テストから直接呼ぶ).
  Eigen::VectorXd clamp_joints(const Eigen::VectorXd & q) const;
  std::pair<double, double> apply_stick_deadzone(
      double x, double y, double deadzone) const;
  std::optional<double> stamp_to_time(const builtin_interfaces::msg::Time & stamp) const;
  std::optional<Eigen::VectorXd> solve_ik(
      const Eigen::Vector3d & target_position, const Eigen::VectorXd & q_seed,
      double damping, int max_iter, double tol);

  // セッション管理 + 統合 callback
  // on_target_with_input: 戻り値 true = IK 収束して publish すべき, false = anchor 設定 or IK 失敗
  bool on_target_with_input(
      const geometry_msgs::msg::Pose & pose,
      float stick_x, float stick_y,
      const builtin_interfaces::msg::Time & stamp,
      double position_scale,
      double stick_velocity_scale,
      double stick_deadzone,
      double stick_max_delta_per_msg,
      double stick_fallback_dt,
      bool unity_conversion,
      double ik_damping, int ik_max_iterations, double ik_tolerance);
  void on_active(bool active);
  void on_joint_state(const std::string & name, double position);
  void on_gripper(double value);

  trajectory_msgs::msg::JointTrajectory make_arm_trajectory(
      const Eigen::VectorXd & q, double trajectory_time_from_start) const;
  trajectory_msgs::msg::JointTrajectory make_gripper_trajectory(
      double angle, double trajectory_time_from_start) const;

  // ROS 2 callback ラッパ
  void on_active_msg(const std_msgs::msg::Bool::SharedPtr msg);
  void on_target_msg(const teleop_ik::msg::TargetPoseWithInput::SharedPtr msg);
  void on_gripper_msg(const std_msgs::msg::Float64::SharedPtr msg);
  void on_reset_msg(const teleop_ik::msg::ResetCommand::SharedPtr msg);
  void on_joint_states_msg(const sensor_msgs::msg::JointState::SharedPtr msg);

  // メンバ: テストから状態を組み立てるため public としている.
  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex ee_frame_id_ = 0;
  std::array<pinocchio::JointIndex, 5> arm_joint_ids_{};
  std::array<pinocchio::JointIndex, 4> position_joint_ids_{};
  std::array<pinocchio::JointIndex, 1> wrist_joint_ids_{};

  // セッション状態 (active 中のみ有効).
  bool active_ = false;
  Eigen::Vector3d arm_init_pos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d unity_anchor_pos_ = Eigen::Vector3d::Zero();
  bool unity_anchor_set_ = false;
  Eigen::Vector2d wrist_init_pos_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d integrated_stick_ = Eigen::Vector2d::Zero();
  std::optional<double> last_msg_stamp_;
  Eigen::VectorXd q_current_;
  Eigen::VectorXd q_solution_;

  // ROS 2 sub/pub
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_active_;
  rclcpp::Subscription<teleop_ik::msg::TargetPoseWithInput>::SharedPtr sub_target_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_gripper_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Subscription<teleop_ik::msg::ResetCommand>::SharedPtr sub_reset_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_gripper_;
};

}  // namespace teleop_ik

#endif  // TELEOP_IK__IK_NODE_HPP_
