cmake_minimum_required(VERSION 3.5)
project(panda_keyboard_control)

find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces (srv/action)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveCommand.srv"
  "action/PickPlace.action"
)

# Export dependency so generated code is available
ament_export_dependencies(rosidl_default_runtime)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install executable scripts (your node)
install(PROGRAMS
  panda_keyboard_control/moveit_service_action_node.py
  panda_keyboard_control/keyboard_control.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
