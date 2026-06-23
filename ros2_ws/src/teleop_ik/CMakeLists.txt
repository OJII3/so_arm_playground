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

add_library(teleop_ik_core SHARED
  src/coordinate_utils.cpp
)
target_include_directories(teleop_ik_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(teleop_ik_core Eigen3)
target_link_libraries(teleop_ik_core pinocchio::pinocchio)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_coordinate_utils_cpp test/test_coordinate_utils.cpp)
  target_link_libraries(test_coordinate_utils_cpp teleop_ik_core)
endif()

install(TARGETS teleop_ik_core
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 pinocchio)

ament_package()
