cmake_minimum_required(VERSION 3.16)
project(teleop_ik)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/TargetPoseWithInput.msg
  DEPENDENCIES geometry_msgs std_msgs
)

# --- C++ core library ---
add_library(teleop_ik_core SHARED
  src/coordinate_utils.cpp
  src/ik_node.cpp
)
target_include_directories(teleop_ik_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(teleop_ik_core Eigen3)
target_link_libraries(teleop_ik_core
  pinocchio::pinocchio teleop_ik__rosidl_typesupport_cpp)
ament_target_dependencies(teleop_ik_core
  rclcpp geometry_msgs sensor_msgs trajectory_msgs builtin_interfaces)

# --- gamepad library (class only, no main) ---
add_library(teleop_ik_gamepad SHARED
  src/gamepad_node.cpp
)
target_include_directories(teleop_ik_gamepad PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(teleop_ik_gamepad
  rclcpp std_msgs sensor_msgs rosidl_typesupport_cpp)
target_link_libraries(teleop_ik_gamepad teleop_ik__rosidl_typesupport_cpp)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# --- ik_node executable ---
add_executable(teleop_ik_node src/ik_node.cpp)
target_include_directories(teleop_ik_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(teleop_ik_node
  rclcpp geometry_msgs std_msgs sensor_msgs trajectory_msgs builtin_interfaces)
target_link_libraries(teleop_ik_node
  teleop_ik_core pinocchio::pinocchio teleop_ik__rosidl_typesupport_cpp)
install(TARGETS teleop_ik_node RUNTIME DESTINATION lib/${PROJECT_NAME})

# --- gamepad_teleop_node executable ---
add_executable(gamepad_teleop_node src/gamepad_node_main.cpp)
target_link_libraries(gamepad_teleop_node teleop_ik_gamepad)
install(TARGETS gamepad_teleop_node RUNTIME DESTINATION lib/${PROJECT_NAME})

# Install libraries
install(TARGETS
  teleop_ik_core teleop_ik_gamepad
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 pinocchio rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_coordinate_utils_cpp test/test_coordinate_utils.cpp)
  target_link_libraries(test_coordinate_utils_cpp teleop_ik_core)
  ament_add_gtest(test_target_msg_cpp test/test_target_msg.cpp)
  ament_target_dependencies(test_target_msg_cpp rosidl_typesupport_cpp)
  target_link_libraries(test_target_msg_cpp teleop_ik__rosidl_typesupport_cpp)
  ament_add_gtest(test_ik_node_helpers_cpp test/test_ik_node_helpers.cpp)
  target_link_libraries(test_ik_node_helpers_cpp teleop_ik_core)
  ament_target_dependencies(test_ik_node_helpers_cpp
    rclcpp geometry_msgs trajectory_msgs builtin_interfaces pinocchio)
  set(_urdf_xacro "${CMAKE_CURRENT_SOURCE_DIR}/test/so101.urdf")
  set_tests_properties(test_ik_node_helpers_cpp PROPERTIES
    ENVIRONMENT "TELEOP_IK_TEST_URDF_PATH=${_urdf_xacro}"
    SKIP_RETURN_CODE 77)
  ament_add_gtest(test_gamepad_node_cpp test/test_gamepad_node.cpp)
  ament_target_dependencies(test_gamepad_node_cpp rclcpp sensor_msgs rosidl_typesupport_cpp)
  target_include_directories(test_gamepad_node_cpp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_gamepad_node_cpp
    teleop_ik_gamepad teleop_ik__rosidl_typesupport_cpp)
  ament_add_gtest(test_qos_cpp test/test_qos.cpp)
  ament_target_dependencies(test_qos_cpp rclcpp)
  target_link_libraries(test_qos_cpp teleop_ik_gamepad)
  ament_add_gtest(test_packaging_cpp test/test_packaging.cpp)
  ament_target_dependencies(test_packaging_cpp ament_index_cpp)
endif()

ament_package()
