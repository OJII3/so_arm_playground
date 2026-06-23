cmake_minimum_required(VERSION 3.16)
project(teleop_ik)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Python3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Compute relative Python site-packages path
string(REGEX REPLACE "^.*/lib/" "lib/" python_site_packages "${Python3_SITELIB}")

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/TargetPoseWithInput.msg
  DEPENDENCIES geometry_msgs std_msgs
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install source Python package alongside the rosidl-generated one
install(DIRECTORY
  ${PROJECT_NAME}
  DESTINATION ${python_site_packages}
)

function(install_python_console_script script_name script_module)
  set(script_path "${CMAKE_CURRENT_BINARY_DIR}/${script_name}")
  file(WRITE "${script_path}"
"#!/usr/bin/env python3
from ${script_module} import main
if __name__ == '__main__':
    main()
")
  file(CHMOD "${script_path}" PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE)
  install(PROGRAMS "${script_path}" DESTINATION lib/${PROJECT_NAME})
endfunction()

install_python_console_script(ik_node teleop_ik.ik_node)
install_python_console_script(gamepad_node teleop_ik.gamepad_node)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_target_msg
    test/test_target_msg.py
    APPEND_ENV "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${python_site_packages}"
  )
  ament_add_pytest_test(test_coordinate_utils
    test/test_coordinate_utils.py
    APPEND_ENV "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${python_site_packages}"
  )
  ament_add_pytest_test(test_ik_node
    test/test_ik_node.py
    APPEND_ENV "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${python_site_packages}"
  )
  ament_add_pytest_test(test_packaging
    test/test_packaging.py
    APPEND_ENV "TELEOP_IK_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}"
  )
endif()

# --- C++ implementation (added incrementally) ---
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

add_library(teleop_ik_core SHARED
  src/coordinate_utils.cpp
  src/ik_node.cpp
)
target_include_directories(teleop_ik_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(teleop_ik_core Eigen3)
target_link_libraries(teleop_ik_core pinocchio::pinocchio teleop_ik__rosidl_typesupport_cpp)
ament_target_dependencies(teleop_ik_core
  rclcpp geometry_msgs sensor_msgs trajectory_msgs builtin_interfaces)

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
  # 実 SO-101 URDF (xacro 展開済) を TELEOP_IK_TEST_URDF_PATH に注入.
  # CMake のビルド時 xacro を呼べないため, 生成済みファイルを使う前提.
  # ファイルがなければテストは SKIP される.
  set(_urdf_xacro "${CMAKE_CURRENT_SOURCE_DIR}/test/so101.urdf")
  set_tests_properties(test_ik_node_helpers_cpp PROPERTIES
    ENVIRONMENT "TELEOP_IK_TEST_URDF_PATH=${_urdf_xacro}"
    SKIP_RETURN_CODE 77)
endif()

install(TARGETS teleop_ik_core
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# --- ik_node executable (Task 14) ---
add_executable(teleop_ik_node
  src/ik_node.cpp
)
target_include_directories(teleop_ik_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(teleop_ik_node
  rclcpp geometry_msgs std_msgs sensor_msgs trajectory_msgs builtin_interfaces)
target_link_libraries(teleop_ik_node teleop_ik_core pinocchio::pinocchio teleop_ik__rosidl_typesupport_cpp)
install(TARGETS teleop_ik_node RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 pinocchio rclcpp)

ament_package()
