cmake_minimum_required(VERSION 2.8.3)
project(robotis_op3_motion_module_msgs)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
  FILES
  WholebodyKinematicsPose.msg
  WholebodyJointPose.msg
  BalanceParam.msg
  JointFeedBackGain.msg
  PoseXYZRPY.msg
  PoseZRPY.msg
  RobotPose.msg
  StepTimeData.msg
  StepPositionData.msg
  StepData.msg
  WalkingJointStatesStamped.msg
  Step2D.msg
  Step2DArray.msg
  FootStepCommand.msg
  JointControllerGain.msg
  FrictionGain.msg
)

add_service_files(
  FILES
  IsRunning.srv
  SetBalanceParam.srv
  GetReferenceStepData.srv
  AddStepDataArray.srv
  StartWalking.srv
  RemoveExistingStepData.srv
  GetWholebodyKinematicsPose.srv
  GetJointControllerGain.srv
  GetFrictionGain.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs geometry_msgs
)

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
