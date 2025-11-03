cmake_minimum_required(VERSION 3.0.2)
project(rh6_ctrl_py)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  rh6_msg
  rh6_cmd
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs rh6_msg rh6_cmd
)

catkin_install_python(PROGRAMS
  scripts/rh6_ctrl_node.py
  scripts/rh6_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)