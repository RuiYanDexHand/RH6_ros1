cmake_minimum_required(VERSION 3.0.2)
project(rh6_msg)

# Find catkin and message generation dependencies (aligned with ROS2 deps where applicable)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
)

# Declare ROS messages
add_message_files(
  FILES
  Rh6Msg.msg
)

# Generate message code
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

# Export package settings
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs
)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)