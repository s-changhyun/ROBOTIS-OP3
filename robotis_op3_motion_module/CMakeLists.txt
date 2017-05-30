cmake_minimum_required(VERSION 2.8.3)
project(robotis_op3_motion_module)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cmake_modules
  eigen_conversions
  dynamixel_sdk
  robotis_device
  robotis_math
  robotis_controller_msgs
  robotis_framework_common
  scilab_optimization
  robotis_op3_kinematics_dynamics
  #robotis_op3_online_walking
  robotis_op3_motion_module_msgs
)

find_package(Eigen REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robotis_op3_motion_module
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

link_directories(
  #/usr/lib/scilab
)

add_library(robotis_op3_motion_module
  src/motion_module.cpp
)

add_dependencies(robotis_op3_motion_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(robotis_op3_motion_module
  yaml-cpp
  #/usr/lib/scilab/libscilab.so
  ${catkin_LIBRARIES}
)
