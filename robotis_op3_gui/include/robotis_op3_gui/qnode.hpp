/**
 * @file /include/robotis_op3_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robotis_op3_gui_QNODE_HPP_
#define robotis_op3_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>

#include <ros/callback_queue.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <boost/thread.hpp>

#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include "robotis_math/robotis_math.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include <robotis_op3_motion_module_msgs/WholebodyJointPose.h>
#include <robotis_op3_motion_module_msgs/WholebodyKinematicsPose.h>
#include <robotis_op3_motion_module_msgs/FootStepCommand.h>

#include <robotis_op3_motion_module_msgs/GetWholebodyKinematicsPose.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robotis_op3_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
  Q_OBJECT
public:
  QNode(int argc, char** argv );
  virtual ~QNode();
  bool init();
  void run();

  /*********************
  ** Logging
  **********************/
  enum LogLevel
  {
      Debug,
      Info,
      Warn,
      Error,
      Fatal
  };

  QStringListModel* loggingModel() { return &logging_model; }
  void log( const LogLevel &level, const std::string &msg, std::string sender);

  void sendSetModeMsg();
  void parseWholebodyJointPose(std_msgs::String msg);
  void getWholebodyKinematicsPose(std_msgs::String msg);
  void sendWholebodyPelvisPoseReset(std_msgs::String msg);
  void sendWholebodyJointPose(robotis_op3_motion_module_msgs::WholebodyJointPose msg);
  void sendWholebodyKinematicsPose(robotis_op3_motion_module_msgs::WholebodyKinematicsPose msg);
  void sendFootStepCommand(robotis_op3_motion_module_msgs::FootStepCommand msg);

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();

  void updateWholebodyKinematicsPose(geometry_msgs::Pose);

private:
  int init_argc;
  char** init_argv;
  QStringListModel logging_model;

  ros::Publisher set_mode_msg_pub_;
  ros::Publisher wholebody_pelvis_pose_reset_msg_pub_;
  ros::Publisher wholebody_joint_pose_msg_pub_;
  ros::Publisher wholebody_kinematics_pose_msg_pub_;
  ros::Publisher foot_step_command_pub_;

  ros::ServiceClient get_wholebody_kinematics_pose_client_;
};

}  // namespace robotis_op3_gui

#endif /* robotis_op3_gui_QNODE_HPP_ */
