cmake_minimum_required(VERSION 3.8)
project(interplanetar_rover)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# ── Custom messages ────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmCommand.msg"
  DEPENDENCIES std_msgs
)

# ── Python nodes ──────────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  interplanetar_rover/gui_node.py
  interplanetar_rover/wheel_bridge_node.py
  interplanetar_rover/arm_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
