/**
 * @file /include/robotis_op3_joint_controller_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robotis_op3_joint_controller_gui_QNODE_HPP_
#define robotis_op3_joint_controller_gui_QNODE_HPP_

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
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>

#include "robotis_math/robotis_math.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "robotis_op3_motion_module_msgs/WholebodyJointPose.h"
#include "robotis_op3_motion_module_msgs/JointControllerGain.h"
#include "robotis_op3_motion_module_msgs/FrictionGain.h"

#include "robotis_op3_motion_module_msgs/GetJointControllerGain.h"
#include "robotis_op3_motion_module_msgs/GetFrictionGain.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robotis_op3_joint_controller_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread
{
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

  void setMotionModule(std_msgs::String msg);
  void sendGazeboRobotInit(std_msgs::Float32 msg);
  void sendGaeboRobotDetach(std_msgs::Bool msg);
  void sendMotionControlMode(std_msgs::String msg);

  void sendJointTorqueOffset(sensor_msgs::JointState msg);
  void setJointTorqueOffset(std_msgs::String msg);

  void sendJointControllerGain(robotis_op3_motion_module_msgs::JointControllerGain msg);
  void saveJointControllerGain(std_msgs::String msg);
  void loadJointControllerGain();

  void sendJointValue(robotis_op3_motion_module_msgs::WholebodyJointPose msg);
  void sendExternalForce(geometry_msgs::WrenchStamped msg);

  void sendFrictionGain(robotis_op3_motion_module_msgs::FrictionGain msg);
  void loadFrictionGain();
  void saveFrictionGain(std_msgs::String msg);

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();

  void updateJointControllerGain(robotis_op3_motion_module_msgs::JointControllerGain);
  void updateFrictionGain(robotis_op3_motion_module_msgs::FrictionGain);

private:
  int init_argc;
  char** init_argv;
  QStringListModel logging_model;

  ros::Publisher set_mode_msg_pub_;
  ros::Publisher send_gazebo_robot_init_pub_;
  ros::Publisher send_gazebo_robot_detach_pub_;
  ros::Publisher send_motion_control_mode_pub_;

  ros::Publisher send_floating_base_pub_;

  ros::Publisher set_joint_controller_gain_pub_;
  ros::Publisher save_joint_controller_gain_pub_;
  ros::Publisher set_joint_value_pub_;
  ros::Publisher set_external_force_pub_;

  ros::Publisher send_joint_torque_offset_pub_;
  ros::Publisher set_joint_torque_offset_pub_;

  ros::Publisher set_friction_gain_pub_;
  ros::Publisher save_friction_gain_pub_;

  ros::ServiceClient load_joint_controller_gain_client_;
  ros::ServiceClient load_friction_gain_client_;
};

}  // namespace robotis_op3_joint_controller_gui

#endif /* robotis_op3_joint_controller_gui_QNODE_HPP_ */
