cmake_minimum_required(VERSION 3.0.2)
project(rh6_ctrl)

# Use proper language standard settings instead of applying C++ flags to C files
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  geometry_msgs
  sensor_msgs
  rh6_cmd
  rh6_msg
)

# Eigen3
find_package(Eigen3 REQUIRED)

# Pinocchio
find_package(pinocchio REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp roslib geometry_msgs sensor_msgs rh6_cmd rh6_msg
)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# rh6_kinematics node
add_executable(rh6_kinematics_node src/rh6_kinematics.cpp)

# ROS1 rh6_ctrl node (with CAN and ryhandlib)
add_executable(rh6_ctrl_node
  src/rh6_ctrl.cpp
  src/can_socket.c
  src/ryhandlib_port.c
)

# ROS1 rh6_test node (publisher demo)
add_executable(rh6_test_node src/rh6_test.cpp)

# Ensure Boost MPL supports enough types for Pinocchio's joint variant
# See: Boost MPL list default (20). Pinocchio may require >20.
target_compile_definitions(rh6_kinematics_node PRIVATE BOOST_MPL_LIMIT_LIST_SIZE=50 BOOST_VARIANT_LIMIT_TYPES=50 BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS)
target_compile_definitions(rh6_ctrl_node PRIVATE        BOOST_MPL_LIMIT_LIST_SIZE=50 BOOST_VARIANT_LIMIT_TYPES=50 BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS)
target_compile_definitions(rh6_test_node PRIVATE        BOOST_MPL_LIMIT_LIST_SIZE=50 BOOST_VARIANT_LIMIT_TYPES=50 BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS)

# Link against Pinocchio (support both imported target and plain library names)
if(TARGET pinocchio::pinocchio)
  target_link_libraries(rh6_kinematics_node
    ${catkin_LIBRARIES}
    pinocchio::pinocchio
  )
  target_link_libraries(rh6_ctrl_node
    ${catkin_LIBRARIES}
    pinocchio::pinocchio
  )
  target_link_libraries(rh6_test_node
    ${catkin_LIBRARIES}
    pinocchio::pinocchio
  )
else()
  target_link_libraries(rh6_kinematics_node
    ${catkin_LIBRARIES}
    pinocchio
  )
  target_link_libraries(rh6_ctrl_node
    ${catkin_LIBRARIES}
    pinocchio
  )
  target_link_libraries(rh6_test_node
    ${catkin_LIBRARIES}
    pinocchio
  )
endif()

# Ryhand library (prefer 64-bit first)
find_library(RYHAND_LIB
  NAMES Ryhand64 Ryhand
  PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib
  NO_DEFAULT_PATH
)

if(NOT RYHAND_LIB)
  message(FATAL_ERROR "Ryhand library not found in rh6_ctrl/lib")
endif()
message(STATUS "Using Ryhand library: ${RYHAND_LIB}")

# Link Ryhand only for rh6_ctrl_node
target_link_libraries(rh6_ctrl_node ${RYHAND_LIB})

# POSIX threads and rt (for linux realtime priority)
if(UNIX)
  find_package(Threads REQUIRED)
  target_link_libraries(rh6_ctrl_node Threads::Threads rt)
endif()

add_dependencies(rh6_kinematics_node ${catkin_EXPORTED_TARGETS})
add_dependencies(rh6_ctrl_node ${catkin_EXPORTED_TARGETS})
add_dependencies(rh6_test_node ${catkin_EXPORTED_TARGETS})

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/dual_sine_cmd.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files (including Python launchers)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.py" PATTERN "*.launch"
)

# Install C++ node executables
install(TARGETS rh6_kinematics_node rh6_ctrl_node rh6_test_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)