cmake_minimum_required(VERSION 2.8.3)
project(robotis_op3_online_walking)

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
  #scilab_optimization
  robotis_op3_kinematics_dynamics
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
  LIBRARIES robotis_op3_online_walking
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


add_library(robotis_op3_online_walking
  src/op3_online_walking.cpp
  src/online_walking.cpp
) 

add_dependencies(robotis_op3_online_walking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(robotis_op3_online_walking
  yaml-cpp
  #/usr/lib/scilab/libscilab.so
  ${catkin_LIBRARIES}
)
