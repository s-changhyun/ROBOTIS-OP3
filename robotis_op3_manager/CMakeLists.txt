cmake_minimum_required(VERSION 2.8.3)
project(robotis_op3_manager)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamixel_sdk
  robotis_framework_common
  robotis_device
  robotis_controller
  robotis_controller_msgs
  robotis_math
  cmake_modules
  robotis_op3_kinematics_dynamics
  scilab_optimization
  robotis_op3_online_walking
  robotis_op3_motion_module
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
catkin_package()

################################################################################
# Build
################################################################################
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(robotis_op3_manager src/robotis_op3_manager.cpp)

add_dependencies(robotis_op3_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(robotis_op3_manager
  ${catkin_LIBRARIES}
)
