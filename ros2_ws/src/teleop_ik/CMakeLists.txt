cmake_minimum_required(VERSION 3.16)
project(teleop_ik NONE)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/TargetPoseWithInput.msg
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()
