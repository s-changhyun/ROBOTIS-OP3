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

#include <stdio.h>
#include "robotis_op3_motion_module/motion_module.h"

using namespace robotis_op3;

MotionModule::MotionModule()
  : control_cycle_sec_(0.004),
    is_moving_(false),
    wb_ik_solving_(false),
    wb_pelvis_planning_(false),
    wb_l_arm_planning_(false),
    wb_r_arm_planning_(false),
    display_joint_angle_(false),
    solve_floating_base_(false),
    cnt_(0)
{
  enable_       = false;
  module_name_  = "motion_module";
  control_mode_ = robotis_framework::PositionControl;

  //  result_["r_sho_pitch"] = new robotis_framework::DynamixelState();
  //  result_["l_sho_pitch"] = new robotis_framework::DynamixelState();
  //  result_["r_sho_roll"]  = new robotis_framework::DynamixelState();
  //  result_["l_sho_roll"]  = new robotis_framework::DynamixelState();
  //  result_["r_el"]        = new robotis_framework::DynamixelState();
  //  result_["l_el"]        = new robotis_framework::DynamixelState();
  result_["r_hip_yaw"]   = new robotis_framework::DynamixelState();
  result_["l_hip_yaw"]   = new robotis_framework::DynamixelState();
  result_["r_hip_roll"]  = new robotis_framework::DynamixelState();
  result_["l_hip_roll"]  = new robotis_framework::DynamixelState();
  result_["r_hip_pitch"] = new robotis_framework::DynamixelState();
  result_["l_hip_pitch"] = new robotis_framework::DynamixelState();
  result_["r_knee"]      = new robotis_framework::DynamixelState();
  result_["l_knee"]      = new robotis_framework::DynamixelState();
  result_["r_ank_pitch"] = new robotis_framework::DynamixelState();
  result_["l_ank_pitch"] = new robotis_framework::DynamixelState();
  result_["r_ank_roll"]  = new robotis_framework::DynamixelState();
  result_["l_ank_roll"]  = new robotis_framework::DynamixelState();
  //  result_["head_pan"]    = new robotis_framework::DynamixelState();
  //  result_["head_tilt"]   = new robotis_framework::DynamixelState();

  //  joint_name_to_id_["r_sho_pitch"] = 1;
  //  joint_name_to_id_["l_sho_pitch"] = 2;
  //  joint_name_to_id_["r_sho_roll"]  = 3;
  //  joint_name_to_id_["l_sho_roll"]  = 4;
  //  joint_name_to_id_["r_el"]        = 5;
  //  joint_name_to_id_["l_el"]        = 6;
  joint_name_to_id_["r_hip_yaw"]   = 1;
  joint_name_to_id_["l_hip_yaw"]   = 2;
  joint_name_to_id_["r_hip_roll"]  = 3;
  joint_name_to_id_["l_hip_roll"]  = 4;
  joint_name_to_id_["r_hip_pitch"] = 5;
  joint_name_to_id_["l_hip_pitch"] = 6;
  joint_name_to_id_["r_knee"]      = 7;
  joint_name_to_id_["l_knee"]      = 8;
  joint_name_to_id_["r_ank_pitch"] = 9;
  joint_name_to_id_["l_ank_pitch"] = 10;
  joint_name_to_id_["r_ank_roll"]  = 11;
  joint_name_to_id_["l_ank_roll"]  = 12;
  //  joint_name_to_id_["head_pan"]    = 19;
  //  joint_name_to_id_["head_tilt"]   = 20;

  gain_path_ = ros::package::getPath("robotis_op3_motion_module") + "/config/joint_gain.yaml";
  torque_offset_path_ = ros::package::getPath("robotis_op3_motion_module") + "/config/torque_offset.yaml";
  friction_path_ = ros::package::getPath("robotis_op3_motion_module") + "/config/friction_gain.yaml";

  /* joint information */
  present_joint_position_   = Eigen::VectorXd::Zero(MAX_JOINT_NUM + 1);
  present_joint_velocity_   = Eigen::VectorXd::Zero(MAX_JOINT_NUM + 1);
  present_joint_effort_     = Eigen::VectorXd::Zero(MAX_JOINT_NUM + 1);

  goal_joint_position_      = Eigen::VectorXd::Zero(MAX_JOINT_NUM + 1);
  goal_joint_velocity_      = Eigen::VectorXd::Zero(MAX_JOINT_NUM + 1);
  goal_joint_acceleration_  = Eigen::VectorXd::Zero(MAX_JOINT_NUM + 1);

  goal_joint_effort_        = Eigen::VectorXd::Zero(MAX_JOINT_NUM + 1);
  joint_position_integral_  = Eigen::VectorXd::Zero(MAX_JOINT_NUM + 1);

  /* torque offset (friction dynamic */
  torque_offset_      = Eigen::VectorXd::Zero(MAX_JOINT_NUM + 1);
  torque_offset_sign_ = Eigen::VectorXd::Zero(MAX_JOINT_NUM + 1);
  torque_offset_sign_.fill(0.0);

  /* friction */
  friction_static_gain_ = Eigen::VectorXd::Zero(MAX_JOINT_NUM + 1);
  friction_dynamic_gain_ = Eigen::VectorXd::Zero(MAX_JOINT_NUM + 1);;
  friction_viscous_gain_ = Eigen::VectorXd::Zero(MAX_JOINT_NUM + 1);;

  /* joint gain */
  ff_torque_gain_    = Eigen::VectorXd::Zero(MAX_JOINT_NUM + 1);
  fb_p_torque_gain_  = Eigen::VectorXd::Zero(MAX_JOINT_NUM + 1);

  ff_joint_vel_gain_    = Eigen::VectorXd::Zero(MAX_JOINT_NUM + 1);
  fb_p_joint_pos_gain_  = Eigen::VectorXd::Zero(MAX_JOINT_NUM + 1);
  fb_i_joint_pos_gain_  = Eigen::VectorXd::Zero(MAX_JOINT_NUM + 1);
  fb_d_joint_pos_gain_  = Eigen::VectorXd::Zero(MAX_JOINT_NUM + 1);

  /* target joint values */
  via_num_ = 1;
  via_time_ = Eigen::MatrixXd::Zero(via_num_, 1);

  target_joint_position_          = Eigen::VectorXd::Zero(MAX_JOINT_ID + 1);
  target_joint_via_position_      = Eigen::MatrixXd::Zero(via_num_,MAX_JOINT_ID + 1);
  target_joint_via_velocity_      = Eigen::MatrixXd::Zero(via_num_, MAX_JOINT_ID + 1);
  target_joint_via_acceleration_  = Eigen::MatrixXd::Zero(via_num_, MAX_JOINT_ID + 1);

  /* wholebody inverse kinematics */
//  ik_weight_ = Eigen::MatrixXd::Zero(ALL_JOINT_ID+1,1);

//  wb_pelvis_target_position_ = Eigen::MatrixXd::Zero(3,1);
//  wb_l_foot_target_position_ = Eigen::MatrixXd::Zero(3,1);
//  wb_r_foot_target_position_ = Eigen::MatrixXd::Zero(3,1);
//  wb_l_arm_target_position_ = Eigen::MatrixXd::Zero(3,1);
//  wb_r_arm_target_position_ = Eigen::MatrixXd::Zero(3,1);

//  wb_l_foot_default_position_ = Eigen::MatrixXd::Zero(3,1);
//  wb_l_foot_default_position_.coeffRef(0,0) = -0.005;
//  wb_l_foot_default_position_.coeffRef(1,0) = 0.037;

//  wb_r_foot_default_position_ = Eigen::MatrixXd::Zero(3,1);
//  wb_r_foot_default_position_.coeffRef(0,0) = -0.005;
//  wb_r_foot_default_position_.coeffRef(1,0) = -0.037;

//  wb_l_foot_default_quaternion_ = robotis_framework::convertRPYToQuaternion(0.0, 0.0, 0.0);
//  wb_r_foot_default_quaternion_ = robotis_framework::convertRPYToQuaternion(0.0, 0.0, 0.0);

  /* robot kinetmatics & dynamics */
  robotis_ = new robotis_op3::KinematicsDynamics(robotis_op3::WholeBody);

  total_mass_ = robotis_->calcTotalMass(0);
  ROS_INFO("total mass: %f", total_mass_);

  walking_ctrl_mode_ = false;
  joint_ctrl_mode_   = true;
}

MotionModule::~MotionModule()
{
  queue_thread_.join();
}

void MotionModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_sec_ = control_cycle_msec * 0.001;
  ROS_INFO("control cycle : %f", control_cycle_sec_);
  queue_thread_ = boost::thread(boost::bind(&MotionModule::queueThread, this));
  OnlineWalkingModule::getInstance()->initialize(control_cycle_msec, robot);
  walking_ctrl_mode_ = false;
  joint_ctrl_mode_   = true;

  op3_walking_joint_it_[ 0] = OnlineWalkingModule::getInstance()->joint_name_to_pose_.find("r_hip_yaw"  );
  op3_walking_joint_it_[ 1] = OnlineWalkingModule::getInstance()->joint_name_to_pose_.find("r_hip_roll" );
  op3_walking_joint_it_[ 2] = OnlineWalkingModule::getInstance()->joint_name_to_pose_.find("r_hip_pitch");
  op3_walking_joint_it_[ 3] = OnlineWalkingModule::getInstance()->joint_name_to_pose_.find("r_knee"     );
  op3_walking_joint_it_[ 4] = OnlineWalkingModule::getInstance()->joint_name_to_pose_.find("r_ank_pitch");
  op3_walking_joint_it_[ 5] = OnlineWalkingModule::getInstance()->joint_name_to_pose_.find("r_ank_roll" );
  op3_walking_joint_it_[ 6] = OnlineWalkingModule::getInstance()->joint_name_to_pose_.find("l_hip_yaw"  );
  op3_walking_joint_it_[ 7] = OnlineWalkingModule::getInstance()->joint_name_to_pose_.find("l_hip_roll" );
  op3_walking_joint_it_[ 8] = OnlineWalkingModule::getInstance()->joint_name_to_pose_.find("l_hip_pitch");
  op3_walking_joint_it_[ 9] = OnlineWalkingModule::getInstance()->joint_name_to_pose_.find("l_knee"     );
  op3_walking_joint_it_[10] = OnlineWalkingModule::getInstance()->joint_name_to_pose_.find("l_ank_pitch");
  op3_walking_joint_it_[11] = OnlineWalkingModule::getInstance()->joint_name_to_pose_.find("l_ank_roll" );

  OnlineWalkingModule::getInstance()->setModuleEnable(true);


  mat_pelvis_to_chest_ = robotis_framework::getTransformationXYZRPY(0.005, 0, 0.0907, 0, 0, 0);
  pose3d_g_to_chest_ = robotis_framework::getPose3DfromTransformMatrix(OnlineWalkingModule::getInstance()->desired_matrix_g_to_pelvis_ * mat_pelvis_to_chest_);
}

void MotionModule::queueThread()
{
  ros::NodeHandle    ros_node;
  ros::CallbackQueue callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publish topics */
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  set_ctrl_module_pub_ = ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);
  goal_joint_state_pub_ = ros_node.advertise<sensor_msgs::JointState>("/robotis/motion/goal_joint_states", 1);

  friction_pub_ = ros_node.advertise<sensor_msgs::JointState>("/robotis/motion/friction", 1);

  load_joint_controller_gain_server_ = ros_node.advertiseService("/robotis/motion/get_joint_control_gain",
                                                                 &MotionModule::loadJointControllerGainCallback, this);
  load_friction_gain_server_ = ros_node.advertiseService("/robotis/motion/get_friction_gain",
                                                         &MotionModule::loadFrictionGainCallback, this);

  //  get_wholebody_kinematics_pose_server_ = ros_node.advertiseService("/robotis/motion/get_wholebody_kinematics_pose",
  //                                                                    &MotionModule::getWholebodyKinematicsPoseCallback, this);

  /* subscribe topics */
//  ros::Subscriber gazebo_init_sub = ros_node.subscribe("/robotis/motion/gazebo_init_msg", 5,
//                                                       &MotionModule::gazeboInitializationCallback, this);
  ros::Subscriber save_joint_controller_gain_sub = ros_node.subscribe("/robotis/motion/save_joint_control_gain_msg", 5,
                                                                      &MotionModule::saveJointControllerGainCallback, this);
  ros::Subscriber set_joint_controller_gain_sub = ros_node.subscribe("/robotis/motion/set_joint_control_gain_msg", 5,
                                                                     &MotionModule::setJointControllerGainCallback, this);
  ros::Subscriber set_external_force_sub = ros_node.subscribe("/robotis/motion/external_force_msg",5,
                                                              &MotionModule::setExternalForceCallback, this);
  //  ros::Subscriber set_joint_value_sub = ros_node.subscribe("/robotis/motion/set_joint_value_msg", 5,
  //                                                           &MotionModule::setJointValueCallback, this);


  //  ros::Subscriber wholeboy_pelvis_pose_reset_msg_sub = ros_node.subscribe("/robotis/motion/wholeboy_pelvis_pose_reset_msg", 5,
  //                                                                          &MotionModule::setWholebodyPelvisPoseResetMsgCallback, this);
  ros::Subscriber wholebody_joint_pose_msg_sub = ros_node.subscribe("/robotis/motion/wholebody_joint_pose_msg", 5,
                                                                    &MotionModule::setWholebodyJointPoseMsgCallback, this);
  //  ros::Subscriber wholebody_kinematics_pose_msg_sub = ros_node.subscribe("/robotis/motion/wholebody_kinematics_pose_msg", 5,
  //                                                                         &MotionModule::setWholebodyKinematicsPoseMsgCallback, this);

  //  std::string ik_weight_path = ros::package::getPath("robotis_op3_motion_module") + "/config/ik_weight.yaml";
  //  parseInverseKinematicsWeightData(ik_weight_path);

  ros::Subscriber set_floating_base_sub = ros_node.subscribe("/robotis/motion/floating_base_msg_", 5,
                                                             &MotionModule::setFloatingBaseMsgCallback, this);

  ros::Subscriber ctrl_mode_msg_sub = ros_node.subscribe("/robotis/motion/ctrl_mode", 5,
                                                         &MotionModule::setCtrlModeMsgCallback, this);

  ros::Subscriber send_joint_torque_offset_sub = ros_node.subscribe("/robotis/motion/joint_torque_offset_msg", 5,
                                                                    &MotionModule::applyJointTorqueOffsetCallback, this);
  ros::Subscriber set_joint_torque_offset_sub = ros_node.subscribe("/robotis/motion/set_joint_torque_offset", 5,
                                                                   &MotionModule::setJointTorqueOffsetCallback, this);

  ros::Subscriber save_friction_gain_sub = ros_node.subscribe("/robotis/motion/save_friction_gain_msg", 5,
                                                              &MotionModule::saveFrictionGainCallback, this);
  ros::Subscriber set_friction_gain_sub = ros_node.subscribe("/robotis/motion/set_friction_gain_msg", 5,
                                                             &MotionModule::setFrictionGainCallback, this);


  ros::WallDuration duration(control_cycle_sec_);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

//void MotionModule::gazeboInitializationCallback(const std_msgs::String::ConstPtr& msg)
//{
//  if (msg->data == "gazebo_init")
//  {
//    ROS_INFO("1");

//  }
//}

//void MotionModule::parseInverseKinematicsWeightData(const std::string &path)
//{
//  YAML::Node doc;
//  try
//  {
//    // load yaml
//    doc = YAML::LoadFile( path.c_str() );
//  }
//  catch(const std::exception& e)
//  {
//    ROS_ERROR("Fail to load yaml file.");
//    return;
//  }

//  YAML::Node inverse_kinematics_weight_node = doc["weight_value"];
//  for(YAML::iterator it = inverse_kinematics_weight_node.begin(); it != inverse_kinematics_weight_node.end() ; ++it)
//  {
//    int id = it->first.as<int>();
//    double value = it->second.as<double>();

//    ik_weight_.coeffRef(id,0) = value;
//  }
//}

//bool MotionModule::getWholebodyKinematicsPoseCallback(robotis_op3_motion_module_msgs::GetWholebodyKinematicsPose::Request &req,
//                                                      robotis_op3_motion_module_msgs::GetWholebodyKinematicsPose::Response &res)
//{
//  if (enable_==false)
//  {
//    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please Set Position Control Module");
//    return false;
//  }

//  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Get Present Kinematics Pose");

//  int joint_id;

//  if (req.group_name == "pelvis")
//    joint_id = ID_CHEST;
//  else if (req.group_name == "left_arm")
//    joint_id = ID_L_ARM_END;
//  else if (req.group_name == "right_arm")
//    joint_id = ID_R_ARM_END;
//  else if (req.group_name == "left_foot")
//    joint_id = ID_L_LEG_END;
//  else if (req.group_name == "right_foot")
//    joint_id = ID_R_LEG_END;
//  else
//    return false;

//  res.group_pose.position.x = robotis_->link_data_[joint_id]->position_.coeff(0,0);
//  res.group_pose.position.y = robotis_->link_data_[joint_id]->position_.coeff(1,0);
//  res.group_pose.position.z = robotis_->link_data_[joint_id]->position_.coeff(2,0);

//  Eigen::Quaterniond quaternion =
//      robotis_framework::convertRotationToQuaternion(robotis_->link_data_[joint_id]->orientation_);

//  res.group_pose.orientation.x = quaternion.x();
//  res.group_pose.orientation.y = quaternion.y();
//  res.group_pose.orientation.z = quaternion.z();
//  res.group_pose.orientation.w = quaternion.w();

//  return true;
//}

//void MotionModule::setWholebodyPelvisPoseResetMsgCallback(const std_msgs::String::ConstPtr& msg)
//{
//  if (msg->data == "reset")
//  {
//    ROS_INFO("Reset Pelvis Pose");
//    robotis_->link_data_[ID_PELVIS_X]->relative_position_.coeffRef(0,0) = 0.0;
//    robotis_->link_data_[ID_PELVIS_Y]->relative_position_.coeffRef(1,0) = 0.0;
//    robotis_->link_data_[ID_PELVIS_Z]->relative_position_.coeffRef(2,0) = 0.3417 - 0.02;

//    robotis_->link_data_[ID_PELVIS_ROLL]->joint_angle_ = 0.0;
//    robotis_->link_data_[ID_PELVIS_PITCH]->joint_angle_ = 0.0;
//    robotis_->link_data_[ID_PELVIS_YAW]->joint_angle_ = 0.0;
//  }
//}

void MotionModule::setWholebodyJointPoseMsgCallback(const robotis_op3_motion_module_msgs::WholebodyJointPose::ConstPtr& msg)
{
  if(enable_ == false)
    return;

  wholebody_goal_joint_msg_ = *msg;

  if (is_moving_ == false)
  {
    tra_gene_tread_ = new boost::thread(boost::bind(&MotionModule::planWholebodyJointPose, this));
    delete tra_gene_tread_;
  }
  else
    ROS_INFO("previous task is alive");

  return;
}

//void MotionModule::setWholebodyKinematicsPoseMsgCallback(const robotis_op3_motion_module_msgs::WholebodyKinematicsPose::ConstPtr& msg)
//{
//  if(enable_ == false)
//    return;

//  ROS_INFO("Set WholebodyKinematicsPose");

//  wholebody_goal_kinematics_msg_ = *msg;

//  if (is_moving_ == false)
//  {
//    tra_gene_tread_ = new boost::thread(boost::bind(&MotionModule::planWholebodyKinematicsPose, this));
//    delete tra_gene_tread_;
//  }
//  else
//    ROS_INFO("previous task is alive");

//  return;
//}

void MotionModule::planWholebodyJointPose()
{
  mov_time_ = wholebody_goal_joint_msg_.mov_time;
  if (mov_time_ == 0.0)
    mov_time_ = 3.0;

  for (int id = 1; id<=MAX_JOINT_ID; id++)
    target_joint_position_(id) = goal_joint_position_(id);

  for (int it = 0; it <wholebody_goal_joint_msg_.joint_state.name.size(); it++)
  {
    std::string joint_name = wholebody_goal_joint_msg_.joint_state.name[it];
    double joint_value = wholebody_goal_joint_msg_.joint_state.position[it];

    target_joint_position_(joint_name_to_id_[joint_name]) = joint_value;
  }

  all_time_steps_ = int(mov_time_ / control_cycle_sec_) + 1;
  goal_joint_position_tra_.resize(all_time_steps_, MAX_JOINT_ID + 1);
  goal_joint_velocity_tra_.resize(all_time_steps_, MAX_JOINT_ID + 1);
  goal_joint_acceleration_tra_.resize(all_time_steps_, MAX_JOINT_ID + 1);

  for (int id=1; id<=MAX_JOINT_ID; id++)
  {
    double ini_value = goal_joint_position_(id);
    double tar_value = target_joint_position_(id);

    Eigen::MatrixXd tra;

    tra = robotis_framework::calcMinimumJerkTraPlus(ini_value, 0.0, 0.0,
                                                    tar_value, 0.0, 0.0,
                                                    control_cycle_sec_, mov_time_);

    goal_joint_position_tra_.block(0, id, all_time_steps_, 1)     = tra.block(0,0,all_time_steps_,1);
    goal_joint_velocity_tra_.block(0, id, all_time_steps_, 1)     = tra.block(0,1,all_time_steps_,1);
    goal_joint_acceleration_tra_.block(0, id, all_time_steps_, 1) = tra.block(0,2,all_time_steps_,1);


    torque_offset_sign_(id) = robotis_framework::sign(tar_value - ini_value);
  }

  PRINT_MAT(torque_offset_sign_);

  is_moving_ = true;
  cnt_ = 0;

  robotis_op3_motion_module_msgs::WholebodyJointPose empty_msg;
  wholebody_goal_joint_msg_ = empty_msg;
}

//void MotionModule::planWholebodyKinematicsPose()
//{
//  mov_time_ = wholebody_goal_kinematics_msg_.mov_time;
//  int all_time_steps = int(floor((mov_time_/control_cycle_sec_) + 1 ));
//  mov_time_ = double (all_time_steps - 1) * control_cycle_sec_;

//  all_time_steps_ = int(mov_time_/control_cycle_sec_) + 1;
//  goal_pelvis_tra_.resize(all_time_steps_, 3);
//  goal_l_foot_tra_.resize(all_time_steps_, 3);
//  goal_r_foot_tra_.resize(all_time_steps_, 3);
//  goal_l_arm_tra_.resize(all_time_steps_, 3);
//  goal_r_arm_tra_.resize(all_time_steps_, 3);

//  if (wholebody_goal_kinematics_msg_.name == "pelvis")
//  {
//    Eigen::MatrixXd pelvis_tar_value = Eigen::MatrixXd::Zero(3,1);
//    pelvis_tar_value.coeffRef(0,0) = wholebody_goal_kinematics_msg_.pelvis_pose.position.x;
//    pelvis_tar_value.coeffRef(1,0) = wholebody_goal_kinematics_msg_.pelvis_pose.position.y;
//    pelvis_tar_value.coeffRef(2,0) = wholebody_goal_kinematics_msg_.pelvis_pose.position.z;

//    for (int dim=0; dim<3; dim++)
//    {
//      double ini_value = robotis_->link_data_[ID_CHEST]->position_.coeff(dim,0);
//      double tar_value = pelvis_tar_value.coeff(dim,0);

//      Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
//                                                                  tar_value, 0.0, 0.0,
//                                                                  control_cycle_sec_, mov_time_);

//      goal_pelvis_tra_.block(0, dim, all_time_steps_, 1) = tra;
//    }

//    Eigen::Quaterniond pelvis_target_quaternion(wholebody_goal_kinematics_msg_.pelvis_pose.orientation.w,
//                                                wholebody_goal_kinematics_msg_.pelvis_pose.orientation.x,
//                                                wholebody_goal_kinematics_msg_.pelvis_pose.orientation.y,
//                                                wholebody_goal_kinematics_msg_.pelvis_pose.orientation.z);

//    wb_pelvis_goal_quaternion_ = pelvis_target_quaternion;

//    planDefaultLegTrajectory();
//    wb_pelvis_planning_ = true;
//  }
//  else if (wholebody_goal_kinematics_msg_.name == "left_arm")
//  {
//    Eigen::MatrixXd l_arm_tar_value = Eigen::MatrixXd::Zero(3,1);
//    l_arm_tar_value.coeffRef(0,0) = wholebody_goal_kinematics_msg_.l_arm_pose.position.x;
//    l_arm_tar_value.coeffRef(1,0) = wholebody_goal_kinematics_msg_.l_arm_pose.position.y;
//    l_arm_tar_value.coeffRef(2,0) = wholebody_goal_kinematics_msg_.l_arm_pose.position.z;

//    for (int dim=0; dim<3; dim++)
//    {
//      double ini_value = robotis_->link_data_[ID_L_ARM_END]->position_.coeff(dim,0);
//      double tar_value = l_arm_tar_value.coeff(dim,0);

//      Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
//                                                                  tar_value, 0.0, 0.0,
//                                                                  control_cycle_sec_, mov_time_);

//      goal_l_arm_tra_.block(0, dim, all_time_steps_, 1) = tra;
//    }

//    /* target quaternion */
//    Eigen::Quaterniond l_arm_target_quaternion(wholebody_goal_kinematics_msg_.l_arm_pose.orientation.w,
//                                               wholebody_goal_kinematics_msg_.l_arm_pose.orientation.x,
//                                               wholebody_goal_kinematics_msg_.l_arm_pose.orientation.y,
//                                               wholebody_goal_kinematics_msg_.l_arm_pose.orientation.z);

//    wb_l_arm_goal_quaternion_ = l_arm_target_quaternion;

//    planDefaultLegTrajectory();
//    wb_l_arm_planning_ = true;
//  }

//  cnt_ = 0;
//  wb_ik_solving_ = true;
//  is_moving_ = true;
//}

//void MotionModule::planDefaultLegTrajectory()
//{
//  for (int dim=0; dim<3; dim++)
//  {
//    double ini_value = wb_l_foot_default_position_.coeff(dim,0);
//    double tar_value = wb_l_foot_default_position_.coeff(dim,0);

//    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
//                                                                tar_value, 0.0, 0.0,
//                                                                control_cycle_sec_, mov_time_);

//    goal_l_foot_tra_.block(0, dim, all_time_steps_, 1) = tra;
//  }

//  for (int dim=0; dim<3; dim++)
//  {
//    double ini_value = wb_r_foot_default_position_.coeff(dim,0);
//    double tar_value = wb_r_foot_default_position_.coeff(dim,0);

//    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0,
//                                                                tar_value, 0.0, 0.0,
//                                                                control_cycle_sec_, mov_time_);

//    goal_r_foot_tra_.block(0, dim, all_time_steps_, 1) = tra;
//  }

//  wb_l_foot_goal_quaternion_ = wb_l_foot_default_quaternion_;
//  wb_r_foot_goal_quaternion_ = wb_r_foot_default_quaternion_;
//}

//void MotionModule::setPelvisPose(int cnt)
//{
//  for ( int dim = 0; dim < 3; dim++ )
//    wb_pelvis_target_position_.coeffRef(dim, 0) = goal_pelvis_tra_.coeff(cnt, dim);

//  double time_step = ( double ) cnt / ( double ) all_time_steps_;
//  Eigen::Quaterniond quaternion = wb_pelvis_start_quaternion_.slerp(time_step, wb_pelvis_goal_quaternion_);

//  wb_pelvis_target_rotation_ = robotis_framework::convertQuaternionToRotation(quaternion);
//}

//void MotionModule::setLeftFootPose(int cnt)
//{
//  for (int dim=0; dim<3; dim++)
//    wb_l_foot_target_position_.coeffRef(dim, 0) = goal_l_foot_tra_.coeff(cnt, dim);

//  double time_step = ( double ) cnt / ( double ) all_time_steps_;
//  Eigen::Quaterniond quaternion = wb_l_foot_start_quaternion_.slerp(time_step, wb_l_foot_goal_quaternion_);

//  wb_l_foot_target_rotation_ = robotis_framework::convertQuaternionToRotation(quaternion);
//}

//void MotionModule::setRightFootPose(int cnt)
//{
//  for (int dim=0; dim<3; dim++)
//    wb_r_foot_target_position_.coeffRef(dim, 0) = goal_r_foot_tra_.coeff(cnt, dim);

//  double time_step = ( double ) cnt / ( double ) all_time_steps_;
//  Eigen::Quaterniond quaternion = wb_r_foot_start_quaternion_.slerp(time_step, wb_r_foot_goal_quaternion_);

//  wb_r_foot_target_rotation_ = robotis_framework::convertQuaternionToRotation(quaternion);
//}

//void MotionModule::setLeftArmPose(int cnt)
//{
//  for (int dim=0; dim<3; dim++)
//    wb_l_arm_target_position_.coeffRef(dim, 0) = goal_l_arm_tra_.coeff(cnt, dim);

//  double time_step = ( double ) cnt / ( double ) all_time_steps_;
//  Eigen::Quaterniond quaternion = wb_l_arm_start_quaternion_.slerp(time_step, wb_l_arm_goal_quaternion_);

//  wb_l_arm_target_rotation_ = robotis_framework::convertQuaternionToRotation(quaternion);
//}

//void MotionModule::setRightArmPose(int cnt)
//{
//  for (int dim=0; dim<3; dim++)
//    wb_r_arm_target_position_.coeffRef(dim, 0) = goal_r_arm_tra_.coeff(cnt, dim);

//  double time_step = ( double ) cnt / ( double ) all_time_steps_;
//  Eigen::Quaterniond quaternion = wb_r_arm_start_quaternion_.slerp(time_step, wb_r_arm_goal_quaternion_);

//  wb_r_arm_target_rotation_ = robotis_framework::convertQuaternionToRotation(quaternion);
//}

//void MotionModule::solveWholebodyInverseKinematics()
//{
//  int max_iter = 70;
//  double ik_tol = 1e-3;

//  if (wb_pelvis_planning_ == true)
//  {
//    robotis_->link_data_[ID_PELVIS_X]->relative_position_.coeffRef(0,0) = wb_pelvis_target_position_.coeff(0,0);
//    robotis_->link_data_[ID_PELVIS_Y]->relative_position_.coeffRef(1,0) = wb_pelvis_target_position_.coeff(1,0);
//    robotis_->link_data_[ID_PELVIS_Z]->relative_position_.coeffRef(2,0) = wb_pelvis_target_position_.coeff(2,0);

//    Eigen::MatrixXd wb_pelvis_target_rpy = robotis_framework::convertRotationToRPY(wb_pelvis_target_rotation_);

//    robotis_->link_data_[ID_PELVIS_ROLL]->joint_angle_  = wb_pelvis_target_rpy.coeff(0,0);
//    robotis_->link_data_[ID_PELVIS_PITCH]->joint_angle_ = wb_pelvis_target_rpy.coeff(1,0);
//    robotis_->link_data_[ID_PELVIS_YAW]->joint_angle_   = wb_pelvis_target_rpy.coeff(2,0);
//  }

//  bool l_arm_ik_success = true;
//  bool r_arm_ik_success = true;
//  if (wb_l_arm_planning_ == true)
//    l_arm_ik_success =
//        robotis_->calcInverseKinematicsForWholebody(ID_BASE, ID_L_ARM_END, wb_l_arm_target_position_, wb_l_arm_target_rotation_, max_iter, ik_tol, ik_weight_);
//  else if (wb_r_arm_planning_ == true)
//    r_arm_ik_success =
//        robotis_->calcInverseKinematicsForWholebody(ID_BASE, ID_R_ARM_END, wb_r_arm_target_position_, wb_r_arm_target_rotation_, max_iter, ik_tol, ik_weight_);

//  bool l_foot_ik_success =
//      robotis_->calcInverseKinematics(ID_CHEST, ID_L_LEG_END, wb_l_foot_target_position_, wb_l_foot_target_rotation_, max_iter, ik_tol);
//  bool r_foot_ik_success =
//      robotis_->calcInverseKinematics(ID_CHEST, ID_R_LEG_END, wb_r_foot_target_position_, wb_r_foot_target_rotation_, max_iter, ik_tol);

////  if (l_foot_ik_success == true && r_foot_ik_success == true &&
//      l_arm_ik_success == true && r_arm_ik_success == true)
//  {
//    for (int id=1; id<=MAX_JOINT_ID; id++)
//      goal_joint_position_(id) = robotis_->link_data_[id]->joint_angle_;
//  }
//  else
//  {
//    ROS_INFO("----- ik failed -----");
//    ROS_INFO("[end] send trajectory");

//    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "IK Failed");

//    is_moving_ = false;
//    wb_ik_solving_ = false;

//    wb_pelvis_planning_ = false;
//    wb_l_arm_planning_ = false;
//    wb_r_arm_planning_ = false;
//    cnt_ = 0;

//    wb_pelvis_target_position_ = robotis_->link_data_[ID_CHEST]->position_;
//    wb_pelvis_target_rotation_ = robotis_->link_data_[ID_CHEST]->orientation_;

//    wb_l_foot_target_position_ = wb_l_foot_default_position_;
//    wb_l_foot_target_rotation_ = robotis_framework::convertQuaternionToRotation(wb_l_foot_default_quaternion_);
//    wb_r_foot_target_position_ = wb_r_foot_default_position_;
//    wb_r_foot_target_rotation_ = robotis_framework::convertQuaternionToRotation(wb_r_foot_default_quaternion_);
//  }
//}

void MotionModule::setStartTrajectory()
{
  ROS_INFO("[start] send trajectory");
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");

  //  if (wb_ik_solving_ == true)
  //  {
  //    Eigen::MatrixXd wb_pelvis_start_rotation = robotis_->link_data_[ID_CHEST]->orientation_;
  //    wb_pelvis_start_quaternion_ = robotis_framework::convertRotationToQuaternion(wb_pelvis_start_rotation);

  //    Eigen::MatrixXd wb_l_foot_start_rotation = robotis_->link_data_[ID_L_LEG_END]->orientation_;
  //    wb_l_foot_start_quaternion_ = robotis_framework::convertRotationToQuaternion(wb_l_foot_start_rotation);
  //    Eigen::MatrixXd wb_r_foot_start_rotation = robotis_->link_data_[ID_R_LEG_END]->orientation_;
  //    wb_r_foot_start_quaternion_ = robotis_framework::convertRotationToQuaternion(wb_r_foot_start_rotation);

  //    Eigen::MatrixXd wb_l_arm_start_rotation = robotis_->link_data_[ID_L_ARM_END]->orientation_;
  //    wb_l_arm_start_quaternion_ = robotis_framework::convertRotationToQuaternion(wb_l_arm_start_rotation);
  //    Eigen::MatrixXd wb_r_arm_start_rotation = robotis_->link_data_[ID_R_ARM_END]->orientation_;
  //    wb_r_arm_start_quaternion_ = robotis_framework::convertRotationToQuaternion(wb_r_arm_start_rotation);
  //  }
}

void MotionModule::setEndTrajectory()
{
  if (is_moving_ == true)
  {
    if (cnt_ >= all_time_steps_)
    {
      ROS_INFO("[end] send trajectory");
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory");

      is_moving_ = false;
      wb_ik_solving_ = false;

      wb_pelvis_planning_ = false;
      wb_l_arm_planning_ = false;
      wb_r_arm_planning_ = false;
      cnt_ = 0;

      if (display_joint_angle_ == true)
      {
        for (int id=1; id<=12; id++)
          ROS_INFO("%s : %f",
                   robotis_->link_data_[id]->name_.c_str(),
                   robotis_->link_data_[id]->joint_angle_ * RADIAN2DEGREE);
      }

      ROS_INFO("-----");

      for (int id=1; id<=MAX_JOINT_ID; id++)
        ROS_INFO("torque[%d] : %f", id, goal_joint_effort_(id));

      PRINT_MAT(robotis_->link_data_[ID_PELVIS_ROLL]->joint_torque_);
      PRINT_MAT(robotis_->link_data_[ID_PELVIS_PITCH]->joint_torque_);
      PRINT_MAT(robotis_->link_data_[ID_PELVIS_YAW]->joint_torque_);

//      PRINT_MAT(robotis_->link_data_[15]->position_);

//      PRINT_MAT(robotis_->link_data_[1]->position_);
//      PRINT_MAT(robotis_->link_data_[2]->position_);

//      PRINT_MAT(robotis_->link_data_[3]->position_);
//      PRINT_MAT(robotis_->link_data_[4]->position_);

//      PRINT_MAT(robotis_->link_data_[5]->position_);
//      PRINT_MAT(robotis_->link_data_[6]->position_);

//      PRINT_MAT(robotis_->link_data_[7]->position_);
//      PRINT_MAT(robotis_->link_data_[8]->position_);

//      PRINT_MAT(robotis_->link_data_[9]->position_);
//      PRINT_MAT(robotis_->link_data_[10]->position_);

//      PRINT_MAT(robotis_->link_data_[11]->position_);
//      PRINT_MAT(robotis_->link_data_[12]->position_);

//      PRINT_MAT(robotis_->link_data_[13]->position_);
//      PRINT_MAT(robotis_->link_data_[14]->position_);

      PRINT_MAT(robotis_->link_data_[ID_L_LEG_END]->position_);
      PRINT_MAT(robotis_->link_data_[ID_R_LEG_END]->position_);

      Eigen::Vector3d mass_center = robotis_->calcMassCenter(0);
      center_of_mass_ = robotis_->calcCenterOfMass(mass_center);
      PRINT_MAT(center_of_mass_);

      ROS_INFO("-----");

      torque_offset_sign_.fill(0.0);

    }
  }
}

void MotionModule::parseJointControllerGain(const std::string &path)
{
  //  Eigen::MatrixXd pid_gain = Eigen::MatrixXd::Zero(MAX_JOINT_NUM,3);

  //  YAML::Node doc;
  //  try
  //  {
  //    // load yaml
  //    doc = YAML::LoadFile( path.c_str() );
  //  }
  //  catch(const std::exception& e)
  //  {
  //    ROS_ERROR("Fail to load yaml file.");
  //    return pid_gain;
  //  }

  //  for(std::map<std::string, int>::iterator it = joint_name_to_id_.begin(); it != joint_name_to_id_.end(); it++)
  //  {
  //    YAML::Node joint_node = doc[it->first];

  //    pid_gain.coeffRef(joint_name_to_id_[it->first]-1,0) = joint_node["p_gain"].as<double>();
  //    pid_gain.coeffRef(joint_name_to_id_[it->first]-1,1) = joint_node["i_gain"].as<double>();
  //    pid_gain.coeffRef(joint_name_to_id_[it->first]-1,2) = joint_node["d_gain"].as<double>();
  //  }

  //  return pid_gain;
}

bool MotionModule::loadJointControllerGainCallback(robotis_op3_motion_module_msgs::GetJointControllerGain::Request &req,
                                                   robotis_op3_motion_module_msgs::GetJointControllerGain::Response &res)
{
  if (enable_==false)
    return false;

  process_mutex_.lock();
  Eigen::VectorXd ff_torque_gain = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);
  Eigen::VectorXd fb_p_torque_gain = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);
  Eigen::VectorXd ff_joint_vel_gain = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);
  Eigen::VectorXd fb_p_joint_pos_gain = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);
  Eigen::VectorXd fb_d_joint_pos_gain = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);
  Eigen::VectorXd fb_i_joint_pos_gain = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);

  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile( gain_path_.c_str() );
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return false;
  }

  for(std::map<std::string, int>::iterator it = joint_name_to_id_.begin(); it != joint_name_to_id_.end(); it++)
  {
    YAML::Node joint_node = doc[it->first];

    ff_torque_gain(joint_name_to_id_[it->first]) = joint_node["ff_torque"].as<double>();
    fb_p_torque_gain(joint_name_to_id_[it->first]) = joint_node["fb_p_torque"].as<double>();
    ff_joint_vel_gain(joint_name_to_id_[it->first]) = joint_node["ff_joint_velocity"].as<double>();
    fb_p_joint_pos_gain(joint_name_to_id_[it->first]) = joint_node["fb_p_joint_position"].as<double>();
    fb_d_joint_pos_gain(joint_name_to_id_[it->first]) = joint_node["fb_d_joint_position"].as<double>();
    fb_i_joint_pos_gain(joint_name_to_id_[it->first]) = joint_node["fb_i_joint_position"].as<double>();
  }

  for(std::map<std::string, int>::iterator it = joint_name_to_id_.begin(); it != joint_name_to_id_.end(); it++)
  {
    res.joint_control_gain.joint_name.push_back(it->first);
    res.joint_control_gain.ff_torque_gain.push_back(ff_torque_gain(joint_name_to_id_[it->first]));
    res.joint_control_gain.fb_p_torque_gain.push_back(fb_p_torque_gain(joint_name_to_id_[it->first]));
    res.joint_control_gain.ff_joint_vel_gain.push_back(ff_joint_vel_gain(joint_name_to_id_[it->first]));
    res.joint_control_gain.fb_p_joint_pos_gain.push_back(fb_p_joint_pos_gain(joint_name_to_id_[it->first]));
    res.joint_control_gain.fb_d_joint_pos_gain.push_back(fb_d_joint_pos_gain(joint_name_to_id_[it->first]));
    res.joint_control_gain.fb_i_joint_pos_gain.push_back(fb_i_joint_pos_gain(joint_name_to_id_[it->first]));
  }
  process_mutex_.unlock();
  return true;
}

bool MotionModule::loadFrictionGainCallback(robotis_op3_motion_module_msgs::GetFrictionGain::Request &req,
                                            robotis_op3_motion_module_msgs::GetFrictionGain::Response &res)
{
  if (enable_==false)
    return false;

  process_mutex_.lock();
  Eigen::VectorXd friction_static_gain = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);
  Eigen::VectorXd friction_dynamic_gain = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);
  Eigen::VectorXd friction_viscous_gain = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);

  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile( friction_path_.c_str() );
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return false;
  }

  for(std::map<std::string, int>::iterator it = joint_name_to_id_.begin(); it != joint_name_to_id_.end(); it++)
  {
    YAML::Node joint_node = doc[it->first];

    friction_static_gain(joint_name_to_id_[it->first]) = joint_node["friction_static"].as<double>();
    friction_dynamic_gain(joint_name_to_id_[it->first]) = joint_node["friction_dynamic"].as<double>();
    friction_viscous_gain(joint_name_to_id_[it->first]) = joint_node["friction_viscous"].as<double>();
  }

  for(std::map<std::string, int>::iterator it = joint_name_to_id_.begin(); it != joint_name_to_id_.end(); it++)
  {
    res.friction_gain.joint_name.push_back(it->first);
    res.friction_gain.friction_static_gain.push_back(friction_static_gain(joint_name_to_id_[it->first]));
    res.friction_gain.friction_dyanmic_gain.push_back(friction_dynamic_gain(joint_name_to_id_[it->first]));
    res.friction_gain.friction_viscous_gain.push_back(friction_viscous_gain(joint_name_to_id_[it->first]));
  }
  process_mutex_.unlock();
  return true;
}

void MotionModule::saveJointControllerGainCallback(const std_msgs::String::ConstPtr& msg)
{
  if (enable_==false)
    return;

  if (msg->data == "save_gain")
  {
    YAML::Emitter out;
    out << YAML::BeginMap;

    std::map<std::string, double> gain_value;

    for(std::map<std::string, int>::iterator it = joint_name_to_id_.begin(); it != joint_name_to_id_.end(); it++)
    {
      gain_value["ff_torque"] = ff_torque_gain_(joint_name_to_id_[it->first]);
      gain_value["fb_p_torque"] = fb_p_torque_gain_(joint_name_to_id_[it->first]);
      gain_value["ff_joint_velocity"] = ff_joint_vel_gain_(joint_name_to_id_[it->first]);
      gain_value["fb_p_joint_position"] = fb_p_joint_pos_gain_(joint_name_to_id_[it->first]);
      gain_value["fb_d_joint_position"] = fb_d_joint_pos_gain_(joint_name_to_id_[it->first]);
      gain_value["fb_i_joint_position"] = fb_i_joint_pos_gain_(joint_name_to_id_[it->first]);

      out << YAML::Key << it->first << YAML::Value << gain_value;
    }

    out << YAML::EndMap;

    // output to file
    std::ofstream fout(gain_path_.c_str());
    fout << out.c_str();
  }
}

void MotionModule::setJointControllerGainCallback(const robotis_op3_motion_module_msgs::JointControllerGain::ConstPtr& msg)
{
  joint_controller_gain_msg_ = *msg;

  updateJointControllerGain();
}

void MotionModule::setCtrlModeMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  if(msg->data == "walking")
  {
    walking_ctrl_mode_ = true;
    joint_ctrl_mode_   = false;
  }
  else if(msg->data == "joint_ctrl")
  {
    walking_ctrl_mode_ = false;
    joint_ctrl_mode_   = true;
  }
  else
  {
    walking_ctrl_mode_ = false;
    joint_ctrl_mode_   = false;
    ROS_ERROR("Invalid Ctrl Mode");
  }
}

void MotionModule::setExternalForceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  if (msg->header.frame_id == "foot")
  {
    Eigen::Vector3d mass_center = robotis_->calcMassCenter(0);
    center_of_mass_ = robotis_->calcCenterOfMass(mass_center);

    double force_z = 0.5 * total_mass_ * 9.81;
    double torque_y = 0.5 * (center_of_mass_.coeff(0,0) - robotis_->link_data_[ID_R_LEG_END]->position_.coeff(0)) * total_mass_ * 9.81;
    PRINT_MAT(center_of_mass_);
    PRINT_MAT(robotis_->link_data_[ID_R_LEG_END]->position_);

//    Eigen::Vector3d l_foot = robotis_->link_data_[ID_L_LEG_END]->position_ - center_of_mass_;
//    Eigen::Vector3d r_foot = robotis_->link_data_[ID_R_LEG_END]->position_ - center_of_mass_;

//    Eigen::MatrixXd l_external_force_matrix = Eigen::MatrixXd::Identity(6,6);
////    l_external_force_matrix.block<3,3>(3,0) = robotis_framework::calcHatto(l_foot);
//    Eigen::MatrixXd r_external_force_matrix = Eigen::MatrixXd::Identity(6,6);
////    r_external_force_matrix.block<3,3>(3,0) = robotis_framework::calcHatto(r_foot);

//    Eigen::MatrixXd force = Eigen::MatrixXd::Zero(6,1);
//    force.coeffRef(0,0) = 0.0;
//    force.coeffRef(1,0) = 0.0;
//    force.coeffRef(2,0) = 0.5 * total_mass_ * 9.8;
//    force.coeffRef(3,0) = 0.0;
//    force.coeffRef(4,0) = torque_y;
//    force.coeffRef(5,0) = 0.0;

//    Eigen::VectorXd l_external_force = l_external_force_matrix * force;
//    Eigen::VectorXd r_external_force = r_external_force_matrix * force;

//    for (int it=0; it<6; it++)
//    {
//      robotis_->link_data_[ID_L_LEG_END]->external_force_.coeffRef(it,0) = l_external_force.coeff(it,0);
//      robotis_->link_data_[ID_R_LEG_END]->external_force_.coeffRef(it,0) = r_external_force.coeff(it,0);
//    }

//    PRINT_MAT(l_external_force);
//    PRINT_MAT(r_external_force);

    robotis_->link_data_[ID_L_ANKLE_ROLL]->external_force_.coeffRef(0,0) = msg->wrench.force.x;
    robotis_->link_data_[ID_L_ANKLE_ROLL]->external_force_.coeffRef(1,0) = msg->wrench.force.y;
    robotis_->link_data_[ID_L_ANKLE_ROLL]->external_force_.coeffRef(2,0) = force_z;
    robotis_->link_data_[ID_L_ANKLE_ROLL]->external_force_.coeffRef(3,0) = msg->wrench.torque.x;
    robotis_->link_data_[ID_L_ANKLE_ROLL]->external_force_.coeffRef(4,0) = -torque_y;
    robotis_->link_data_[ID_L_ANKLE_ROLL]->external_force_.coeffRef(5,0) = msg->wrench.torque.z;

    robotis_->link_data_[ID_R_ANKLE_ROLL]->external_force_.coeffRef(0,0) = msg->wrench.force.x;
    robotis_->link_data_[ID_R_ANKLE_ROLL]->external_force_.coeffRef(1,0) = msg->wrench.force.y;
    robotis_->link_data_[ID_R_ANKLE_ROLL]->external_force_.coeffRef(2,0) = force_z;
    robotis_->link_data_[ID_R_ANKLE_ROLL]->external_force_.coeffRef(3,0) = msg->wrench.torque.x;
    robotis_->link_data_[ID_R_ANKLE_ROLL]->external_force_.coeffRef(4,0) = -torque_y;
    robotis_->link_data_[ID_R_ANKLE_ROLL]->external_force_.coeffRef(5,0) = msg->wrench.torque.z;

//    robotis_->link_data_[ID_L_LEG_END]->external_force_.coeffRef(0,0) = msg->wrench.force.x;
//    robotis_->link_data_[ID_L_LEG_END]->external_force_.coeffRef(1,0) = msg->wrench.force.y;
//    robotis_->link_data_[ID_L_LEG_END]->external_force_.coeffRef(2,0) = 0.5 * total_mass_ * 9.81;
//    robotis_->link_data_[ID_L_LEG_END]->external_force_.coeffRef(3,0) = msg->wrench.torque.x;
//    robotis_->link_data_[ID_L_LEG_END]->external_force_.coeffRef(4,0) = msg->wrench.torque.y;
//    robotis_->link_data_[ID_L_LEG_END]->external_force_.coeffRef(5,0) = msg->wrench.torque.z;

//    robotis_->link_data_[ID_R_LEG_END]->external_force_.coeffRef(0,0) = msg->wrench.force.x;
//    robotis_->link_data_[ID_R_LEG_END]->external_force_.coeffRef(1,0) = msg->wrench.force.y;
//    robotis_->link_data_[ID_R_LEG_END]->external_force_.coeffRef(2,0) = 0.5 * total_mass_ * 9.81;
//    robotis_->link_data_[ID_R_LEG_END]->external_force_.coeffRef(3,0) = msg->wrench.torque.x;
//    robotis_->link_data_[ID_R_LEG_END]->external_force_.coeffRef(4,0) = msg->wrench.torque.y;
//    robotis_->link_data_[ID_R_LEG_END]->external_force_.coeffRef(5,0) = msg->wrench.torque.z;

    ROS_INFO("force z : %f", force_z);
    ROS_INFO("torque y : %f", torque_y);

//    if (msg->wrench.force.z == 0.0)
//    {
//      robotis_->link_data_[ID_L_LEG_END]->external_force_.coeffRef(2,0) = msg->wrench.force.z;
//      robotis_->link_data_[ID_R_LEG_END]->external_force_.coeffRef(2,0) = msg->wrench.force.z;
//    }

  }
}

void MotionModule::setFloatingBaseMsgCallback(const std_msgs::Bool::ConstPtr& msg)
{
  solve_floating_base_ = true;

  ROS_INFO("solve_floating_base_ is true");
}

//void MotionModule::setJointValueCallback(const sensor_msgs::JointState::ConstPtr& msg)
//{

//}

void MotionModule::applyJointTorqueOffsetCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  for (int idx = 0; idx < msg->name.size(); idx++)
  {
    std::string joint_name = msg->name[idx];

    torque_offset_[joint_name_to_id_[joint_name]] = msg->effort[idx];
  }

  PRINT_MAT(torque_offset_);
}

void MotionModule::setJointTorqueOffsetCallback(const std_msgs::String::ConstPtr& msg)
{
  if (enable_==false)
    return;

  process_mutex_.lock();

  if (msg->data == "load_joint_torque_offset")
  {
    YAML::Node doc;
    try
    {
      // load yaml
      doc = YAML::LoadFile( torque_offset_path_.c_str() );
    }
    catch(const std::exception& e)
    {
      ROS_ERROR("Fail to load yaml file.");
      return;
    }

    YAML::Node torque_offset_node = doc["torque_offset"];
    for(YAML::iterator it = torque_offset_node.begin() ; it != torque_offset_node.end() ; ++it)
    {
      std::string joint_name;
      double value;

      joint_name = it->first.as<std::string>();
      value = it->second.as<double>();

      torque_offset_(joint_name_to_id_[joint_name]) = value;
    }

    PRINT_MAT(torque_offset_);
  }
  else if (msg->data == "save_joint_torque_offset")
  {
    YAML::Emitter out;
    out << YAML::BeginMap;

    std::map<std::string, double> offset_value;

    for(std::map<std::string, int>::iterator it = joint_name_to_id_.begin(); it != joint_name_to_id_.end(); it++)
    {
      std::string joint_name = it->first;

      offset_value[joint_name] = torque_offset_(joint_name_to_id_[it->first]);
    }

    out << YAML::Key << "torque_offset" << YAML::Value << offset_value;
    out << YAML::EndMap;

    // output to file
    std::ofstream fout(torque_offset_path_.c_str());
    fout << out.c_str();
  }

  process_mutex_.unlock();
}

void MotionModule::saveFrictionGainCallback(const std_msgs::String::ConstPtr& msg)
{
  if (enable_==false)
    return;

  ROS_INFO("1");

  if (msg->data == "save_gain")
  {
    YAML::Emitter out;
    out << YAML::BeginMap;

    std::map<std::string, double> gain_value;

    for(std::map<std::string, int>::iterator it = joint_name_to_id_.begin(); it != joint_name_to_id_.end(); it++)
    {
      gain_value["friction_static"] = friction_static_gain_(joint_name_to_id_[it->first]);
      gain_value["friction_dynamic"] = friction_dynamic_gain_(joint_name_to_id_[it->first]);
      gain_value["friction_viscous"] = friction_viscous_gain_(joint_name_to_id_[it->first]);

      out << YAML::Key << it->first << YAML::Value << gain_value;
    }

    out << YAML::EndMap;

    // output to file
    std::ofstream fout(friction_path_.c_str());
    fout << out.c_str();
  }
}

void MotionModule::setFrictionGainCallback(const robotis_op3_motion_module_msgs::FrictionGain::ConstPtr& msg)
{
  friction_gain_msg_ = *msg;

  updateFrictionGain();
}

void MotionModule::updateJointControllerGain()
{
  for (int it=0; it<joint_controller_gain_msg_.joint_name.size(); it++)
  {
    int id = joint_name_to_id_[joint_controller_gain_msg_.joint_name[it]];

    ff_torque_gain_(id) = joint_controller_gain_msg_.ff_torque_gain[it];
    fb_p_torque_gain_(id) = joint_controller_gain_msg_.fb_p_torque_gain[it];
    ff_joint_vel_gain_(id) = joint_controller_gain_msg_.ff_joint_vel_gain[it];
    fb_p_joint_pos_gain_(id) = joint_controller_gain_msg_.fb_p_joint_pos_gain[it];
    fb_i_joint_pos_gain_(id) = joint_controller_gain_msg_.fb_i_joint_pos_gain[it];
    fb_d_joint_pos_gain_(id) = joint_controller_gain_msg_.fb_d_joint_pos_gain[it];
  }

//  PRINT_MAT(ff_torque_gain_);
//  PRINT_MAT(fb_p_torque_gain_);
//  PRINT_MAT(ff_joint_vel_gain_);
//  PRINT_MAT(fb_p_joint_pos_gain_);
//  PRINT_MAT(fb_d_joint_pos_gain_);
//  PRINT_MAT(fb_i_joint_pos_gain_);
}

void MotionModule::updateFrictionGain()
{
  for (int it=0; it<friction_gain_msg_.joint_name.size(); it++)
  {
    int id = joint_name_to_id_[friction_gain_msg_.joint_name[it]];

    friction_static_gain_(id) = friction_gain_msg_.friction_static_gain[it];
    friction_dynamic_gain_(id) = friction_gain_msg_.friction_dyanmic_gain[it];
    friction_viscous_gain_(id) = friction_gain_msg_.friction_viscous_gain[it];
  }
}

Eigen::VectorXd MotionModule::calcJointController()
{
  Eigen::VectorXd joint_controller_output = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);

  Eigen::VectorXd joint_torque_output = Eigen::MatrixXd::Zero(MAX_JOINT_ID+1);
  Eigen::VectorXd joint_position_output = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);

  Eigen::VectorXd joint_error = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);

  for (int id=1; id<=MAX_JOINT_ID; id++)
  {
    joint_error(id) = goal_joint_position_(id) - present_joint_position_(id);
    joint_position_integral_(id) = joint_position_integral_(id) + (joint_error(id) * control_cycle_sec_);

    joint_torque_output(id) =
        ff_torque_gain_(id) * goal_joint_effort_(id) +
        fb_p_torque_gain_(id) * (goal_joint_effort_(id) - present_joint_effort_(id));

    joint_position_output(id) =
        ff_joint_vel_gain_(id) * present_joint_velocity_(id) +
        fb_p_joint_pos_gain_(id) * (goal_joint_position_(id) - present_joint_position_(id)) +
        fb_d_joint_pos_gain_(id) * (goal_joint_velocity_(id) - present_joint_velocity_(id)) +
        fb_i_joint_pos_gain_(id) * joint_position_integral_(id);

    joint_controller_output(id) = joint_torque_output(id) + joint_position_output(id);

    joint_controller_output(id) += torque_offset_sign_(id) * torque_offset_(id);
  }

  return joint_controller_output;
}

Eigen::VectorXd MotionModule::calcFriction()
{
  Eigen::VectorXd friction_output = Eigen::VectorXd::Zero(MAX_JOINT_ID+1);

  double fs, fd, fv;

  for (int id=1; id<=MAX_JOINT_ID; id++)
  {
    if (cnt_ >= 1 && cnt_ <= 10)
    {
      double diff = target_joint_position_(id) - present_joint_position_(id);

      if (diff > 1e-4)
        fs = 1.0;
      else if ( diff < -1e-4)
        fs = -1.0;
    }
    else
      fs = 0.0;

    if (fabs(present_joint_velocity_(id)) < 1e-3)
      fd = 0.0;
    else
      fd = robotis_framework::sign(present_joint_velocity_(id));

    fv = present_joint_velocity_(id);

    friction_output(id) =
        friction_static_gain_(id) * fs +
        friction_dynamic_gain_(id) * fd +
        friction_viscous_gain_(id) * fv;
  }

  return friction_output;
}

void MotionModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                           std::map<std::string, double> sensors)
{
  ros::Time begin = ros::Time::now();
  ros::Duration time_duration;

  if (enable_==false)
  {
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Please Set Motion Module");
    return;
  }

  process_mutex_.lock();

  /*----- Get Joint State -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;

    robotis_framework::Dynamixel *dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      dxl = dxl_it->second;
    else
      continue;

    present_joint_position_(joint_name_to_id_[joint_name]) = dxl->dxl_state_->present_position_;
    present_joint_velocity_(joint_name_to_id_[joint_name]) = dxl->dxl_state_->present_velocity_;
    present_joint_effort_(joint_name_to_id_[joint_name]) = dxl->dxl_state_->present_torque_;
  }

  if (joint_ctrl_mode_ == true)
  {
    if (is_moving_ == true)
    {
      if (cnt_ == 0)
      {
        setStartTrajectory();
      }

      for (int id=1; id<=MAX_JOINT_ID; id++)
      {
        goal_joint_position_(id)      = goal_joint_position_tra_(cnt_, id);
        goal_joint_velocity_(id)      = goal_joint_velocity_tra_(cnt_, id);
        goal_joint_acceleration_(id)  = goal_joint_acceleration_tra_(cnt_, id);
      }
      cnt_++;
    }

    for (int id=1; id<=MAX_JOINT_ID; id++)
    {
      robotis_->link_data_[id]->joint_angle_        = goal_joint_position_(id);
      robotis_->link_data_[id]->joint_velocity_     = goal_joint_velocity_(id);
      robotis_->link_data_[id]->joint_acceleration_ = goal_joint_acceleration_(id);
    }

    setEndTrajectory();
  }
  else if(walking_ctrl_mode_ == true)
  {
    OnlineWalkingModule *op3_online_walking = OnlineWalkingModule::getInstance();
    op3_online_walking->process(dxls, sensors);

    mat_pelvis_to_chest_ = robotis_framework::getTransformationXYZRPY(0, 0, 0.0285, 0, 0, 0);
    pose3d_g_to_chest_ = robotis_framework::getPose3DfromTransformMatrix(op3_online_walking->desired_matrix_g_to_pelvis_ * mat_pelvis_to_chest_);

    robotis_->link_data_[ID_PELVIS_X]->relative_position_.coeffRef(0,0)     = pose3d_g_to_chest_.x;
    robotis_->link_data_[ID_PELVIS_Y]->relative_position_.coeffRef(0,0)     = pose3d_g_to_chest_.y;
    robotis_->link_data_[ID_PELVIS_Z]->relative_position_.coeffRef(0,0)     = pose3d_g_to_chest_.z;
    robotis_->link_data_[ID_PELVIS_ROLL]->relative_position_.coeffRef(0,0)  = pose3d_g_to_chest_.roll;
    robotis_->link_data_[ID_PELVIS_PITCH]->relative_position_.coeffRef(0,0) = pose3d_g_to_chest_.pitch;
    robotis_->link_data_[ID_PELVIS_YAW]->relative_position_.coeffRef(0,0)   = pose3d_g_to_chest_.yaw;

    //r leg
    robotis_->link_data_[joint_name_to_id_["r_hip_yaw"]]->joint_angle_          = op3_walking_joint_it_[ 0]->second.position_;
    robotis_->link_data_[joint_name_to_id_["r_hip_yaw"]]->joint_velocity_       = op3_walking_joint_it_[ 0]->second.velocity_;
    robotis_->link_data_[joint_name_to_id_["r_hip_yaw"]]->joint_acceleration_   = op3_walking_joint_it_[ 0]->second.acceleration_;

    robotis_->link_data_[joint_name_to_id_["r_hip_roll"]]->joint_angle_         = op3_walking_joint_it_[ 1]->second.position_;
    robotis_->link_data_[joint_name_to_id_["r_hip_roll"]]->joint_velocity_      = op3_walking_joint_it_[ 1]->second.velocity_;
    robotis_->link_data_[joint_name_to_id_["r_hip_roll"]]->joint_acceleration_  = op3_walking_joint_it_[ 1]->second.acceleration_;

    robotis_->link_data_[joint_name_to_id_["r_hip_pitch"]]->joint_angle_        = op3_walking_joint_it_[ 2]->second.position_;
    robotis_->link_data_[joint_name_to_id_["r_hip_pitch"]]->joint_velocity_     = op3_walking_joint_it_[ 2]->second.velocity_;
    robotis_->link_data_[joint_name_to_id_["r_hip_pitch"]]->joint_acceleration_ = op3_walking_joint_it_[ 2]->second.acceleration_;

    robotis_->link_data_[joint_name_to_id_["r_knee"]]->joint_angle_             = op3_walking_joint_it_[ 3]->second.position_;
    robotis_->link_data_[joint_name_to_id_["r_knee"]]->joint_velocity_          = op3_walking_joint_it_[ 3]->second.velocity_;
    robotis_->link_data_[joint_name_to_id_["r_knee"]]->joint_acceleration_      = op3_walking_joint_it_[ 3]->second.acceleration_;

    robotis_->link_data_[joint_name_to_id_["r_ank_pitch"]]->joint_angle_        = op3_walking_joint_it_[ 4]->second.position_;
    robotis_->link_data_[joint_name_to_id_["r_ank_pitch"]]->joint_velocity_     = op3_walking_joint_it_[ 4]->second.velocity_;
    robotis_->link_data_[joint_name_to_id_["r_ank_pitch"]]->joint_acceleration_ = op3_walking_joint_it_[ 4]->second.acceleration_;

    robotis_->link_data_[joint_name_to_id_["r_ank_roll"]]->joint_angle_         = op3_walking_joint_it_[ 5]->second.position_;
    robotis_->link_data_[joint_name_to_id_["r_ank_roll"]]->joint_velocity_      = op3_walking_joint_it_[ 5]->second.velocity_;
    robotis_->link_data_[joint_name_to_id_["r_ank_roll"]]->joint_acceleration_  = op3_walking_joint_it_[ 5]->second.acceleration_;

    //l leg
    robotis_->link_data_[joint_name_to_id_["l_hip_yaw"]]->joint_angle_          = op3_walking_joint_it_[ 6]->second.position_;
    robotis_->link_data_[joint_name_to_id_["l_hip_yaw"]]->joint_velocity_       = op3_walking_joint_it_[ 6]->second.velocity_;
    robotis_->link_data_[joint_name_to_id_["l_hip_yaw"]]->joint_acceleration_   = op3_walking_joint_it_[ 6]->second.acceleration_;

    robotis_->link_data_[joint_name_to_id_["l_hip_roll"]]->joint_angle_         = op3_walking_joint_it_[ 7]->second.position_;
    robotis_->link_data_[joint_name_to_id_["l_hip_roll"]]->joint_velocity_      = op3_walking_joint_it_[ 7]->second.velocity_;
    robotis_->link_data_[joint_name_to_id_["l_hip_roll"]]->joint_acceleration_  = op3_walking_joint_it_[ 7]->second.acceleration_;

    robotis_->link_data_[joint_name_to_id_["l_hip_pitch"]]->joint_angle_        = op3_walking_joint_it_[ 8]->second.position_;
    robotis_->link_data_[joint_name_to_id_["l_hip_pitch"]]->joint_velocity_     = op3_walking_joint_it_[ 8]->second.velocity_;
    robotis_->link_data_[joint_name_to_id_["l_hip_pitch"]]->joint_acceleration_ = op3_walking_joint_it_[ 8]->second.acceleration_;

    robotis_->link_data_[joint_name_to_id_["l_knee"]]->joint_angle_             = op3_walking_joint_it_[ 9]->second.position_;
    robotis_->link_data_[joint_name_to_id_["l_knee"]]->joint_velocity_          = op3_walking_joint_it_[ 9]->second.velocity_;
    robotis_->link_data_[joint_name_to_id_["l_knee"]]->joint_acceleration_      = op3_walking_joint_it_[ 9]->second.acceleration_;

    robotis_->link_data_[joint_name_to_id_["l_ank_pitch"]]->joint_angle_        = op3_walking_joint_it_[10]->second.position_;
    robotis_->link_data_[joint_name_to_id_["l_ank_pitch"]]->joint_velocity_     = op3_walking_joint_it_[10]->second.velocity_;
    robotis_->link_data_[joint_name_to_id_["l_ank_pitch"]]->joint_acceleration_ = op3_walking_joint_it_[10]->second.acceleration_;

    robotis_->link_data_[joint_name_to_id_["l_ank_roll"]]->joint_angle_         = op3_walking_joint_it_[11]->second.position_;
    robotis_->link_data_[joint_name_to_id_["l_ank_roll"]]->joint_velocity_      = op3_walking_joint_it_[11]->second.velocity_;
    robotis_->link_data_[joint_name_to_id_["l_ank_roll"]]->joint_acceleration_  = op3_walking_joint_it_[11]->second.acceleration_;

    for (int id=1; id<=MAX_JOINT_ID; id++)
    {
      goal_joint_position_(id) = robotis_->link_data_[id]->joint_angle_;
      goal_joint_velocity_(id) = robotis_->link_data_[id]->joint_velocity_;
      goal_joint_acceleration_(id) = robotis_->link_data_[id]->joint_acceleration_ ;
    }

//    for(unsigned int idx = 0; idx < 12; idx++)
//      std::cout << op3_walking_joint_it_[idx]->second.position_ << " " << op3_walking_joint_it_[idx]->second.velocity_ << " "<< op3_walking_joint_it_[idx]->second.acceleration_ << " ";
//
//    std::cout << std::endl;
  }


  robotis_->calcForwardAllKinematics(0); // forward kinematics
  Eigen::MatrixXd force_torque = robotis_->calcInverseDynamics(0); // inverse dynamics

  // Joint Controller
  for (int id=1; id<=MAX_JOINT_ID; id++)
    goal_joint_effort_(id) = robotis_->link_data_[id]->joint_torque_;


  Eigen::VectorXd joint_controller_output = calcJointController();
  Eigen::VectorXd friction_output = calcFriction();

  Eigen::VectorXd joint_controller_output_with_friction = joint_controller_output + friction_output;

  /*----- Set Goal Joint State -----*/
  sensor_msgs::JointState friction_msg;
  friction_msg.header.stamp = ros::Time::now();

  sensor_msgs::JointState goal_joint_state_msg;
  goal_joint_state_msg.header.stamp = ros::Time::now();

  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_position_ = goal_joint_position_(joint_name_to_id_[joint_name]);
    //    result_[joint_name]->goal_torque_ = joint_controller_output_with_friction(joint_name_to_id_[joint_name]);

    goal_joint_state_msg.name.push_back(joint_name);
    goal_joint_state_msg.position.push_back(goal_joint_position_(joint_name_to_id_[joint_name]));
    goal_joint_state_msg.velocity.push_back(goal_joint_velocity_(joint_name_to_id_[joint_name]));
    goal_joint_state_msg.effort.push_back(joint_controller_output_with_friction(joint_name_to_id_[joint_name]));
//    goal_joint_state_msg.effort.push_back(goal_joint_effort_(joint_name_to_id_[joint_name]));

    friction_msg.name.push_back(joint_name);
    friction_msg.effort.push_back(friction_output(joint_name_to_id_[joint_name]));
  }

  friction_pub_.publish(friction_msg);

  goal_joint_state_pub_.publish(goal_joint_state_msg);
  process_mutex_.unlock();


  time_duration = ros::Time::now() - begin;
  double loop_time = time_duration.toSec(); //time_duration.sec + time_duration.nsec * 1e-9;

  if (loop_time > 0.004)
    ROS_WARN("Calculation Time : %f", loop_time );
}


void MotionModule::stop()
{
  return;
}

bool MotionModule::isRunning()
{
  return is_moving_;
}

void MotionModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "MotionModule";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}
