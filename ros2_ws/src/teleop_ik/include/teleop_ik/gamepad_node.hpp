// teleop_ik/include/teleop_ik/gamepad_node.hpp
#ifndef TELEOP_IK__GAMEPAD_NODE_HPP_
#define TELEOP_IK__GAMEPAD_NODE_HPP_

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <teleop_ik/msg/target_pose_with_input.hpp>

namespace teleop_ik
{

class GamepadTeleopNode : public rclcpp::Node
{
 public:
  GamepadTeleopNode();

  // テスト用.
  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg);
  void timer_tick();

  // 状態 (テストから触る).
  bool active_ = false;
  bool prev_toggle_pressed_ = false;
  double target_x_ = 0.0;
  double target_y_ = 0.0;
  double target_z_ = 0.0;
  double gripper_value_ = 0.5;
  bool joy_received_ = false;
  std::optional<sensor_msgs::msg::Joy> latest_joy_;

  // パラメータを直接読めるように public 化.
  double linear_speed_ = 0.0;
  double vertical_speed_ = 0.0;
  double deadzone_ = 0.0;
  double dt_ = 0.0;
  int axis_x_ = 0;
  int axis_y_ = 0;
  int axis_z_ = 0;
  int btn_gripper_open_ = 0;
  int btn_gripper_close_ = 0;
  int btn_toggle_ = 0;

 private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;
  rclcpp::Publisher<teleop_ik::msg::TargetPoseWithInput>::SharedPtr pub_target_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_active_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_gripper_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace teleop_ik

#endif  // TELEOP_IK__GAMEPAD_NODE_HPP_
