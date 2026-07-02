// teleop_ik/src/gamepad_node.cpp
#include "teleop_ik/gamepad_node.hpp"

#include <algorithm>
#include <cmath>

namespace teleop_ik
{

namespace
{
double apply_deadzone(double v, double dz) { return std::abs(v) < dz ? 0.0 : v; }
double safe_axis(const sensor_msgs::msg::Joy & j, int idx) {
  return idx < static_cast<int>(j.axes.size()) ? j.axes[idx] : 0.0;
}
bool safe_button(const sensor_msgs::msg::Joy & j, int idx) {
  return idx < static_cast<int>(j.buttons.size()) ? j.buttons[idx] != 0 : false;
}
}  // namespace

GamepadTeleopNode::GamepadTeleopNode() : rclcpp::Node("gamepad_teleop_node")
{
  this->declare_parameter<double>("publish_rate", 50.0);
  this->declare_parameter<double>("linear_speed", 0.05);
  this->declare_parameter<double>("vertical_speed", 0.05);
  this->declare_parameter<double>("deadzone", 0.15);
  this->declare_parameter<int>("axis_x", 1);
  this->declare_parameter<int>("axis_y", 0);
  this->declare_parameter<int>("axis_z", 3);
  this->declare_parameter<int>("button_gripper_open", 5);
  this->declare_parameter<int>("button_gripper_close", 4);
  this->declare_parameter<int>("button_toggle_active", 0);

  linear_speed_ = this->get_parameter("linear_speed").as_double();
  vertical_speed_ = this->get_parameter("vertical_speed").as_double();
  deadzone_ = this->get_parameter("deadzone").as_double();
  axis_x_ = this->get_parameter("axis_x").as_int();
  axis_y_ = this->get_parameter("axis_y").as_int();
  axis_z_ = this->get_parameter("axis_z").as_int();
  btn_gripper_open_ = this->get_parameter("button_gripper_open").as_int();
  btn_gripper_close_ = this->get_parameter("button_gripper_close").as_int();
  btn_toggle_ = this->get_parameter("button_toggle_active").as_int();

  const double rate = this->get_parameter("publish_rate").as_double();
  dt_ = 1.0 / rate;

  sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&GamepadTeleopNode::on_joy, this, std::placeholders::_1));
  pub_target_ = this->create_publisher<teleop_ik::msg::TargetPoseWithInput>("/teleop/target", 10);
  pub_active_ = this->create_publisher<std_msgs::msg::Bool>("/teleop/active", 10);
  pub_gripper_ = this->create_publisher<std_msgs::msg::Float64>("/teleop/gripper", 10);

  timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt_), std::bind(&GamepadTeleopNode::timer_tick, this));
}

void GamepadTeleopNode::on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (!joy_received_) {
    joy_received_ = true;
  }
  latest_joy_ = *msg;
}

void GamepadTeleopNode::timer_tick()
{
  if (!latest_joy_.has_value()) {
    return;
  }
  const auto & joy = *latest_joy_;

  const bool toggle_pressed = safe_button(joy, btn_toggle_);
  if (toggle_pressed && !prev_toggle_pressed_) {
    active_ = !active_;
    std_msgs::msg::Bool msg;
    msg.data = active_;
    pub_active_->publish(msg);
    if (active_) {
      target_x_ = target_y_ = target_z_ = 0.0;
      gripper_value_ = 0.5;
    }
  }
  prev_toggle_pressed_ = toggle_pressed;

  if (!active_) {
    return;
  }

  const double vx = apply_deadzone(safe_axis(joy, axis_x_), deadzone_);
  const double vy = apply_deadzone(safe_axis(joy, axis_y_), deadzone_);
  const double vz = apply_deadzone(safe_axis(joy, axis_z_), deadzone_);
  target_x_ += vx * linear_speed_ * dt_;
  target_y_ += vy * linear_speed_ * dt_;
  target_z_ += vz * vertical_speed_ * dt_;

  constexpr double gripper_speed = 2.0;
  if (safe_button(joy, btn_gripper_open_)) {
    gripper_value_ = std::min(1.0, gripper_value_ + gripper_speed * dt_);
  }
  if (safe_button(joy, btn_gripper_close_)) {
    gripper_value_ = std::max(0.0, gripper_value_ - gripper_speed * dt_);
  }

  auto msg = teleop_ik::msg::TargetPoseWithInput();
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "world";
  msg.pose.position.x = target_x_;
  msg.pose.position.y = target_y_;
  msg.pose.position.z = target_z_;
  msg.pose.orientation.w = 1.0;
  msg.stick_x = 0.0f;
  msg.stick_y = 0.0f;
  // gamepad is always in IK mode (no trigger toggle available).
  msg.ik_active = true;
  pub_target_->publish(msg);

  std_msgs::msg::Float64 gmsg;
  gmsg.data = gripper_value_;
  pub_gripper_->publish(gmsg);
}

}  // namespace teleop_ik

// main は gamepad_node_main.cpp に分離 (テストで link 可能にするため).
