cmake_minimum_required(VERSION 3.0.2)
project(rh6_kin_py)

find_package(catkin REQUIRED COMPONENTS rospy)

# Ensure Python package is exported in devel space
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy
)

# Install Python modules when doing `catkin_make install`
install(DIRECTORY rh6_kin_py/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)