/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#ifndef robotis_op3_MOTION_MODULE_H_
#define robotis_op3_MOTION_MODULE_H_

#include <map>
#include <fstream>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

//#include "gazebo/gazebo.hh"
//#include "gazebo/common/common.hh"
//#include "gazebo/physics/physics.hh"

#include "robotis_math/robotis_math.h"
#include "robotis_framework_common/motion_module.h"

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "robotis_op3_kinematics_dynamics/kinematics_dynamics.h"

//#include "robotis_op3_online_walking/online_walking.h"

#include "robotis_op3_motion_module_msgs/WholebodyJointPose.h"
#include "robotis_op3_motion_module_msgs/WholebodyKinematicsPose.h"
#include "robotis_op3_motion_module_msgs/JointControllerGain.h"
#include "robotis_op3_motion_module_msgs/FrictionGain.h"

#include "robotis_op3_motion_module_msgs/GetWholebodyKinematicsPose.h"
#include "robotis_op3_motion_module_msgs/GetJointControllerGain.h"
#include "robotis_op3_motion_module_msgs/GetFrictionGain.h"

namespace robotis_op3
{

#define MAX_JOINT_NUM   (12)
#define TASK_DEMENSION  (6)

class MotionModule
  : public robotis_framework::MotionModule,
    public robotis_framework::Singleton<MotionModule>
{
private:
  double control_cycle_sec_;
  boost::thread queue_thread_;

  std::string gain_path_;
  std::string torque_offset_path_;
  std::string friction_path_;

  /* sample subscriber & publisher */
  ros::Publisher status_msg_pub_;
  ros::Publisher set_ctrl_module_pub_;
  ros::Publisher goal_joint_state_pub_;
  ros::Publisher friction_pub_;

  ros::ServiceServer load_joint_controller_gain_server_;
  ros::ServiceServer get_wholebody_kinematics_pose_server_;
  ros::ServiceServer load_friction_gain_server_;

  boost::thread* tra_gene_tread_;

  std::map<std::string, int> joint_name_to_id_;

  /* joint information */
  Eigen::VectorXd present_joint_position_;
  Eigen::VectorXd present_joint_velocity_;
  Eigen::VectorXd present_joint_effort_;

  Eigen::VectorXd goal_joint_position_;
  Eigen::VectorXd goal_joint_velocity_;
  Eigen::VectorXd goal_joint_acceleration_;

  Eigen::VectorXd torque_offset_;
  Eigen::VectorXd torque_offset_sign_;

  /* friction gain */
  Eigen::VectorXd friction_static_gain_;
  Eigen::VectorXd friction_dynamic_gain_;
  Eigen::VectorXd friction_viscous_gain_;

  Eigen::VectorXd goal_joint_effort_;

  Eigen::VectorXd joint_position_integral_;

  /* joint gain */
  Eigen::VectorXd ff_torque_gain_;
  Eigen::VectorXd fb_p_torque_gain_;

  Eigen::VectorXd ff_joint_vel_gain_;
  Eigen::VectorXd fb_p_joint_pos_gain_;
  Eigen::VectorXd fb_i_joint_pos_gain_;
  Eigen::VectorXd fb_d_joint_pos_gain_;

  /* movement */
  bool is_moving_;
  double mov_time_;
  int all_time_steps_;
  int cnt_;

  /* msgs */
  robotis_op3_motion_module_msgs::WholebodyJointPose wholebody_goal_joint_msg_;
  robotis_op3_motion_module_msgs::WholebodyKinematicsPose wholebody_goal_kinematics_msg_;

  robotis_op3_motion_module_msgs::JointControllerGain joint_controller_gain_msg_;
  robotis_op3_motion_module_msgs::FrictionGain friction_gain_msg_;

  /* target joint values */
  Eigen::VectorXd target_joint_position_;

  int via_num_;
  Eigen::MatrixXd via_time_;
  Eigen::MatrixXd target_joint_via_position_;
  Eigen::MatrixXd target_joint_via_velocity_;
  Eigen::MatrixXd target_joint_via_acceleration_;

  /* goal trajectory */
  Eigen::MatrixXd goal_joint_position_tra_;
  Eigen::MatrixXd goal_joint_velocity_tra_;
  Eigen::MatrixXd goal_joint_acceleration_tra_;

  Eigen::MatrixXd goal_task_tra_;
  Eigen::MatrixXd goal_pelvis_tra_;
  Eigen::MatrixXd goal_l_foot_tra_;
  Eigen::MatrixXd goal_r_foot_tra_;
  Eigen::MatrixXd goal_l_arm_tra_;
  Eigen::MatrixXd goal_r_arm_tra_;

  /* wholebody inverse kinematics */
  bool wb_ik_solving_;
  bool wb_pelvis_planning_;
  bool wb_l_arm_planning_;
  bool wb_r_arm_planning_;

  Eigen::MatrixXd ik_weight_;
  Eigen::MatrixXd ik_target_position_, ik_target_rotation_;
  Eigen::Quaterniond ik_start_quaternion_, ik_goal_quaternion_;

  Eigen::MatrixXd wb_pelvis_target_position_, wb_pelvis_target_rotation_;
  Eigen::Quaterniond wb_pelvis_start_quaternion_, wb_pelvis_goal_quaternion_;

  Eigen::MatrixXd wb_l_arm_target_position_, wb_l_arm_target_rotation_;
  Eigen::Quaterniond wb_l_arm_start_quaternion_, wb_l_arm_goal_quaternion_;
  Eigen::MatrixXd wb_r_arm_target_position_, wb_r_arm_target_rotation_;
  Eigen::Quaterniond wb_r_arm_start_quaternion_, wb_r_arm_goal_quaternion_;

  Eigen::MatrixXd wb_l_foot_target_position_, wb_l_foot_target_rotation_;
  Eigen::Quaterniond wb_l_foot_start_quaternion_, wb_l_foot_goal_quaternion_;
  Eigen::MatrixXd wb_r_foot_target_position_, wb_r_foot_target_rotation_;
  Eigen::Quaterniond wb_r_foot_start_quaternion_, wb_r_foot_goal_quaternion_;

  Eigen::MatrixXd wb_l_foot_default_position_, wb_r_foot_default_position_;
  Eigen::Quaterniond wb_l_foot_default_quaternion_, wb_r_foot_default_quaternion_;

  /* etc */
  bool display_joint_angle_;
  bool solve_floating_base_;

  /* center of mass */
  double total_mass_;
  Eigen::Vector3d center_of_mass_;

  // kinematics dynamics parameters
  robotis_op3::KinematicsDynamics *robotis_;

//  bool walking_ctrl_mode_;
  bool joint_ctrl_mode_;

//  std::map<std::string, JointPose>::iterator op3_walking_joint_it_[12];
  Eigen::Matrix4d mat_pelvis_to_chest_;
  robotis_framework::Pose3D pose3d_g_to_chest_;
  boost::mutex    process_mutex_;

  void queueThread();

  void parseInverseKinematicsWeightData(const std::string &path);
  void parseJointControllerGain(const std::string &path);

  void gazeboInitializationCallback(const std_msgs::String::ConstPtr& msg);

  void saveJointControllerGainCallback(const std_msgs::String::ConstPtr& msg);
  void setJointControllerGainCallback(const robotis_op3_motion_module_msgs::JointControllerGain::ConstPtr& msg);
  void setExternalForceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
//  void setJointValueCallback(const sensor_msgs::JointState::ConstPtr& msg);

  bool loadJointControllerGainCallback(robotis_op3_motion_module_msgs::GetJointControllerGain::Request &req,
                                       robotis_op3_motion_module_msgs::GetJointControllerGain::Response &res);
  bool loadFrictionGainCallback(robotis_op3_motion_module_msgs::GetFrictionGain::Request &req,
                                robotis_op3_motion_module_msgs::GetFrictionGain::Response &res);

  bool getWholebodyKinematicsPoseCallback(robotis_op3_motion_module_msgs::GetWholebodyKinematicsPose::Request &req,
                                          robotis_op3_motion_module_msgs::GetWholebodyKinematicsPose::Response &res);

  void setWholebodyPelvisPoseResetMsgCallback(const std_msgs::String::ConstPtr& msg);
  void setWholebodyJointPoseMsgCallback(const robotis_op3_motion_module_msgs::WholebodyJointPose::ConstPtr& msg);
  void setWholebodyKinematicsPoseMsgCallback(const robotis_op3_motion_module_msgs::WholebodyKinematicsPose::ConstPtr& msg);

  void setFloatingBaseMsgCallback(const std_msgs::Bool::ConstPtr& msg);
  void setCtrlModeMsgCallback(const std_msgs::String::ConstPtr& msg);

  void applyJointTorqueOffsetCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void setJointTorqueOffsetCallback(const std_msgs::String::ConstPtr& msg);

  void saveFrictionGainCallback(const std_msgs::String::ConstPtr& msg);
  void setFrictionGainCallback(const robotis_op3_motion_module_msgs::FrictionGain::ConstPtr& msg);

  void planWholebodyJointPose();
  void planWholebodyKinematicsPose();
  void planDefaultLegTrajectory();

  void setPelvisPose(int cnt);
  void setLeftFootPose(int cnt);
  void setRightFootPose(int cnt);
  void setLeftArmPose(int cnt);
  void setRightArmPose(int cnt);

  void setStartTrajectory();
  void setEndTrajectory();

  void solveWholebodyInverseKinematics();

  void updateJointControllerGain();
  Eigen::VectorXd calcJointController();

  void updateFrictionGain();
  Eigen::VectorXd calcFriction();

public:
  MotionModule();
  virtual ~MotionModule();

  /* ROS Topic Callback Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
               std::map<std::string, double> sensors);

  void stop();
  bool isRunning();
  void publishStatusMsg(unsigned int type, std::string msg);
};

}

#endif /* robotis_op3_MOTION_MODULE_H_ */
