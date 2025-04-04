cmake_minimum_required(VERSION 3.8)
project(terrain_mapping_drone_control)

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DroneCommand.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Install models directory
install(
  DIRECTORY
    models
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()