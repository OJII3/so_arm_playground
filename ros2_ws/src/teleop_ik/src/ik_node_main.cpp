// teleop_ik/src/ik_node_main.cpp
// ik_node.cpp の main 関数部分. テストで lib を使いやすくするため分離.
#include "teleop_ik/ik_node.hpp"

#include <cstdio>
#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<teleop_ik::TeleopIKNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "teleop_ik_node: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
