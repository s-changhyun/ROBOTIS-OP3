/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/robotis_op3_joint_controller_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robotis_op3_joint_controller_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv)
{}

QNode::~QNode()
{
  if(ros::isStarted())
  {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init()
{
  ros::init(init_argc,init_argv,"robotis_op3_joint_controller_gui");

  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;

  // Add your ros communications here.
  set_mode_msg_pub_ = n.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);
  set_joint_controller_gain_pub_ = n.advertise<robotis_op3_motion_module_msgs::JointControllerGain>("/robotis/motion/set_joint_control_gain_msg", 0);
  save_joint_controller_gain_pub_ = n.advertise<std_msgs::String>("/robotis/motion/save_joint_control_gain_msg", 0);
  set_joint_value_pub_ = n.advertise<robotis_op3_motion_module_msgs::WholebodyJointPose>("/robotis/motion/wholebody_joint_pose_msg", 0);
  set_external_force_pub_ = n.advertise<geometry_msgs::WrenchStamped>("/robotis/motion/external_force_msg", 0);

  send_joint_torque_offset_pub_ = n.advertise<sensor_msgs::JointState>("/robotis/motion/joint_torque_offset_msg", 0);
  set_joint_torque_offset_pub_ = n.advertise<std_msgs::String>("/robotis/motion/set_joint_torque_offset", 0);

  set_friction_gain_pub_ = n.advertise<robotis_op3_motion_module_msgs::FrictionGain>("/robotis/motion/set_friction_gain_msg", 0);
  save_friction_gain_pub_ = n.advertise<std_msgs::String>("/robotis/motion/save_friction_gain_msg", 0);

  send_gazebo_robot_init_pub_ = n.advertise<std_msgs::Float32>("/robotis_op3/harness/velocity", 0);
  send_gazebo_robot_detach_pub_ = n.advertise<std_msgs::Bool>("/robotis_op3/harness/detach", 0);

  send_motion_control_mode_pub_ = n.advertise<std_msgs::String>("/robotis/motion/ctrl_mode", 0);

  send_floating_base_pub_ = n.advertise<std_msgs::Bool>("/robotis/motion/floating_base_msg_", 0);

  load_joint_controller_gain_client_ = n.serviceClient<robotis_op3_motion_module_msgs::GetJointControllerGain>("/robotis/motion/get_joint_control_gain", 0);
  load_friction_gain_client_ = n.serviceClient<robotis_op3_motion_module_msgs::GetFrictionGain>("/robotis/motion/get_friction_gain", 0);

  start();
  return true;
}

void QNode::setMotionModule(std_msgs::String msg)
{
  set_mode_msg_pub_.publish(msg);
}

void QNode::sendGazeboRobotInit(std_msgs::Float32 msg)
{
  send_gazebo_robot_init_pub_.publish(msg);
}

void QNode::sendGaeboRobotDetach(std_msgs::Bool msg)
{
  send_gazebo_robot_detach_pub_.publish(msg);

  send_floating_base_pub_.publish(msg);
}

void QNode::sendMotionControlMode(std_msgs::String msg)
{
  send_motion_control_mode_pub_.publish(msg);
}

void QNode::sendJointControllerGain(robotis_op3_motion_module_msgs::JointControllerGain msg)
{
  set_joint_controller_gain_pub_.publish(msg);
}

void QNode::sendJointTorqueOffset(sensor_msgs::JointState msg)
{
  send_joint_torque_offset_pub_.publish(msg);
}

void QNode::setJointTorqueOffset(std_msgs::String msg)
{
  set_joint_torque_offset_pub_.publish(msg);
}

void QNode::saveJointControllerGain(std_msgs::String msg)
{
  save_joint_controller_gain_pub_.publish(msg);
}

void QNode::loadJointControllerGain()
{
  robotis_op3_motion_module_msgs::GetJointControllerGain srv;

  if (load_joint_controller_gain_client_.call(srv))
  {
    robotis_op3_motion_module_msgs::JointControllerGain msg;

    msg.joint_name = srv.response.joint_control_gain.joint_name;
    msg.ff_torque_gain = srv.response.joint_control_gain.ff_torque_gain;
    msg.fb_p_torque_gain = srv.response.joint_control_gain.fb_p_torque_gain;
    msg.ff_joint_vel_gain = srv.response.joint_control_gain.ff_joint_vel_gain;
    msg.fb_p_joint_pos_gain = srv.response.joint_control_gain.fb_p_joint_pos_gain;
    msg.fb_d_joint_pos_gain = srv.response.joint_control_gain.fb_d_joint_pos_gain;
    msg.fb_i_joint_pos_gain = srv.response.joint_control_gain.fb_i_joint_pos_gain;

    Q_EMIT updateJointControllerGain(msg);
  }
}

void QNode::sendJointValue(robotis_op3_motion_module_msgs::WholebodyJointPose msg)
{
  set_joint_value_pub_.publish(msg);
}

void QNode::sendExternalForce(geometry_msgs::WrenchStamped msg)
{
  set_external_force_pub_.publish(msg);
}

void QNode::sendFrictionGain(robotis_op3_motion_module_msgs::FrictionGain msg)
{
  set_friction_gain_pub_.publish(msg);
}

void QNode::loadFrictionGain()
{
  robotis_op3_motion_module_msgs::GetFrictionGain srv;

  if (load_friction_gain_client_.call(srv))
  {
    robotis_op3_motion_module_msgs::FrictionGain msg;

    msg.joint_name = srv.response.friction_gain.joint_name;
    msg.friction_static_gain = srv.response.friction_gain.friction_static_gain;
    msg.friction_dyanmic_gain = srv.response.friction_gain.friction_dyanmic_gain;
    msg.friction_viscous_gain = srv.response.friction_gain.friction_viscous_gain;

    Q_EMIT updateFrictionGain(msg);
  }
}

void QNode::saveFrictionGain(std_msgs::String msg)
{
  save_friction_gain_pub_.publish(msg);
}

void QNode::run()
{
  ros::Rate loop_rate(125);
  while ( ros::ok() )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::log( const LogLevel &level, const std::string &msg, std::string sender)
{
  logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;

  std::stringstream _sender;
  _sender << "[" << sender << "] ";

  switch ( level ) {
  case(Debug) : {
    ROS_DEBUG_STREAM(msg);
    logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << _sender.str() << msg;
    break;
  }
  case(Info) : {
    ROS_INFO_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << _sender.str() << msg;
    break;
  }
  case(Warn) : {
    ROS_WARN_STREAM(msg);
    logging_model_msg << "[WARN] [" << ros::Time::now() << "]: " << _sender.str() <<msg;
    break;
  }
  case(Error) : {
    ROS_ERROR_STREAM(msg);
    logging_model_msg << "<ERROR> [" << ros::Time::now() << "]: " << _sender.str() <<msg;
    break;
  }
  case(Fatal) : {
    ROS_FATAL_STREAM(msg);
    logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << _sender.str() <<msg;
    break;
  }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace robotis_op3_joint_controller_gui
