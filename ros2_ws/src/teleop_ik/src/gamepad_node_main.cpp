// teleop_ik/src/gamepad_node_main.cpp
// gamepad_node.cpp の main 関数部分. テストで lib を使いやすくするため分離.
#include "teleop_ik/gamepad_node.hpp"

#include <cstdio>
#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<teleop_ik::GamepadTeleopNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "gamepad_teleop_node: %s\n", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
