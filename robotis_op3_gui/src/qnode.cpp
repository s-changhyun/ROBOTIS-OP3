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
#include "../include/robotis_op3_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robotis_op3_gui {

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

void QNode::sendSetModeMsg()
{
  std_msgs::String str_msg;

  str_msg.data = "motion_module";
  set_mode_msg_pub_.publish(str_msg);
}

void QNode::parseWholebodyJointPose(std_msgs::String msg)
{
  std::string file_path;
  file_path = ros::package::getPath("robotis_op3_motion_module") + "/config/" + msg.data + ".yaml";

  if (msg.data == "init_pose")
  {
    std_msgs::String msg;
    msg.data = "reset";
    sendWholebodyPelvisPoseReset(msg);
  }

  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile( file_path.c_str() );
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return ;
  }

  robotis_op3_motion_module_msgs::WholebodyJointPose wholebody_joint_pose_msg;
  wholebody_joint_pose_msg.mov_time = doc["mov_time"].as< double >();

  YAML::Node tar_pose_node = doc["tar_pose"];
  for(YAML::iterator it = tar_pose_node.begin() ; it != tar_pose_node.end() ; ++it)
  {
    std::string joint_name;
    double value;

    joint_name = it->first.as<std::string>();
    value = it->second.as<double>();

    wholebody_joint_pose_msg.joint_state.name.push_back(joint_name);
    wholebody_joint_pose_msg.joint_state.position.push_back(value * DEGREE2RADIAN);
  }

  sendWholebodyJointPose(wholebody_joint_pose_msg);
}

void QNode::getWholebodyKinematicsPose(std_msgs::String msg)
{
  robotis_op3_motion_module_msgs::GetWholebodyKinematicsPose srv;
  srv.request.group_name = msg.data;

  if (get_wholebody_kinematics_pose_client_.call(srv))
  {
    geometry_msgs::Pose msg;
    msg = srv.response.group_pose;

    Q_EMIT updateWholebodyKinematicsPose(msg);
  }
}

void QNode::sendWholebodyPelvisPoseReset(std_msgs::String msg)
{
  wholebody_pelvis_pose_reset_msg_pub_.publish(msg);
}

void QNode::sendWholebodyJointPose(robotis_op3_motion_module_msgs::WholebodyJointPose msg)
{
  wholebody_joint_pose_msg_pub_.publish(msg);
}


void QNode::sendWholebodyKinematicsPose(robotis_op3_motion_module_msgs::WholebodyKinematicsPose msg)
{
  wholebody_kinematics_pose_msg_pub_.publish(msg);
}

void QNode::sendFootStepCommand(robotis_op3_motion_module_msgs::FootStepCommand msg)
{
  foot_step_command_pub_.publish(msg);
}

bool QNode::init()
{
  ros::init(init_argc,init_argv,"robotis_op3_gui");

  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;

  // Add your ros communications here.
  set_mode_msg_pub_ = n.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);

  wholebody_pelvis_pose_reset_msg_pub_ = n.advertise<std_msgs::String>("/robotis/motion/wholeboy_pelvis_pose_reset_msg", 0);
  wholebody_joint_pose_msg_pub_ = n.advertise<robotis_op3_motion_module_msgs::WholebodyJointPose>("/robotis/motion/wholebody_joint_pose_msg", 0);
  wholebody_kinematics_pose_msg_pub_ = n.advertise<robotis_op3_motion_module_msgs::WholebodyKinematicsPose>("/robotis/motion/wholebody_kinematics_pose_msg", 0);

  foot_step_command_pub_ = n.advertise<robotis_op3_motion_module_msgs::FootStepCommand>("/robotis/robotis_op3_foot_step_generator/foot_step_command", 0);

  get_wholebody_kinematics_pose_client_ =
      n.serviceClient<robotis_op3_motion_module_msgs::GetWholebodyKinematicsPose>("/robotis/motion/get_wholebody_kinematics_pose", 0);

  start();
  return true;
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

}  // namespace robotis_op3_gui
