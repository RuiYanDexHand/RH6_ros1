cmake_minimum_required(VERSION 3.0.2)
project(rh6_cmd)

# Find catkin and message generation dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

# Message definitions
add_message_files(
  FILES
  Rh6Cmd.msg
)

# Service definitions
add_service_files(
  FILES
  Rh6fk.srv
  Rh6ik.srv
)

# Generate message and service code
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Export package settings
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)