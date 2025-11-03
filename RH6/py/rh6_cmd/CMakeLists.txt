cmake_minimum_required(VERSION 3.0.2)
project(rh6_cmd)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  Rh6Cmd.msg
)

add_service_files(
  FILES
  Rh6fk.srv
  Rh6ik.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)