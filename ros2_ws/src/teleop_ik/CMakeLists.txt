cmake_minimum_required(VERSION 3.16)
project(teleop_ik)

find_package(Python3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
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

set(node_scripts
  ik_node teleop_ik.ik_node:main
  gamepad_node teleop_ik.gamepad_node:main
)
foreach(script_entry ${node_scripts})
  list(POP_FRONT script_entry script_name script_module)
  set(script_path "${CMAKE_CURRENT_BINARY_DIR}/${script_name}")
  file(WRITE "${script_path}"
"#!/usr/bin/env python3
from ${script_module} import main
if __name__ == '__main__':
    main()
")
  install(PROGRAMS "${script_path}" DESTINATION lib/${PROJECT_NAME})
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_target_msg
    test/test_target_msg.py
    APPEND_ENV "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${python_site_packages}"
  )
endif()

ament_package()
