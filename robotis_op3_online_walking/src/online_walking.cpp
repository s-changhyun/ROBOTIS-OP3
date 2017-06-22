/*
 * online_walking.cpp
 *
 *  Created on: 2016. 11. 29.
 *      Author: JaySong
 */

#include <stdio.h>
#include <eigen_conversions/eigen_msg.h>

#include "robotis_op3_online_walking/online_walking.h"

using namespace robotis_op3;

class WalkingStatusMSG
{
public:
  static const std::string FAILED_TO_ADD_STEP_DATA_MSG;
  static const std::string BALANCE_PARAM_SETTING_START_MSG;
  static const std::string BALANCE_PARAM_SETTING_FINISH_MSG;
  static const std::string WALKING_MODULE_IS_ENABLED_MSG;
  static const std::string WALKING_MODULE_IS_DISABLED_MSG;
  static const std::string BALANCE_HAS_BEEN_TURNED_OFF;
  static const std::string WALKING_START_MSG;
  static const std::string WALKING_FINISH_MSG;
};

const std::string WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG = "Failed_to_add_Step_Data";
const std::string WalkingStatusMSG::BALANCE_PARAM_SETTING_START_MSG = "Balance_Param_Setting_Started";
const std::string WalkingStatusMSG::BALANCE_PARAM_SETTING_FINISH_MSG = "Balance_Param_Setting_Finished";
const std::string WalkingStatusMSG::WALKING_MODULE_IS_ENABLED_MSG = "Walking_Module_is_enabled";
const std::string WalkingStatusMSG::WALKING_MODULE_IS_DISABLED_MSG = "Walking_Module_is_disabled";
const std::string WalkingStatusMSG::BALANCE_HAS_BEEN_TURNED_OFF = "Balance_has_been_turned_off";
const std::string WalkingStatusMSG::WALKING_START_MSG = "Walking_Started";
const std::string WalkingStatusMSG::WALKING_FINISH_MSG = "Walking_Finished";

JointPose::JointPose()
{
  position_ = 0;
  velocity_ = 0;
  acceleration_ = 0;
}

JointPose::~JointPose()
{ }


OnlineWalkingModule::OnlineWalkingModule()
: control_cycle_msec_(4)
{
  gazebo_          = false;
  enable_          = false;
  module_name_     = "walking_module";
  control_mode_    = robotis_framework::PositionControl;
  result_["r_hip_yaw"  ] = new robotis_framework::DynamixelState();
  result_["r_hip_roll" ] = new robotis_framework::DynamixelState();
  result_["r_hip_pitch"] = new robotis_framework::DynamixelState();
  result_["r_knee"     ] = new robotis_framework::DynamixelState();
  result_["r_ank_pitch"] = new robotis_framework::DynamixelState();
  result_["r_ank_roll" ] = new robotis_framework::DynamixelState();

  result_["l_hip_yaw"  ] = new robotis_framework::DynamixelState();
  result_["l_hip_roll" ] = new robotis_framework::DynamixelState();
  result_["l_hip_pitch"] = new robotis_framework::DynamixelState();
  result_["l_knee"     ] = new robotis_framework::DynamixelState();
  result_["l_ank_pitch"] = new robotis_framework::DynamixelState();
  result_["l_ank_roll" ] = new robotis_framework::DynamixelState();

  joint_name_to_pose_["r_hip_yaw"  ];
  joint_name_to_pose_["r_hip_roll" ];
  joint_name_to_pose_["r_hip_pitch"];
  joint_name_to_pose_["r_knee"     ];
  joint_name_to_pose_["r_ank_pitch"];
  joint_name_to_pose_["r_ank_roll" ];

  joint_name_to_pose_["l_hip_yaw"  ];
  joint_name_to_pose_["l_hip_roll" ];
  joint_name_to_pose_["l_hip_pitch"];
  joint_name_to_pose_["l_knee"     ];
  joint_name_to_pose_["l_ank_pitch"];
  joint_name_to_pose_["l_ank_roll" ];

  joint_name_to_index_["r_hip_yaw"  ] = 0;
  joint_name_to_index_["r_hip_roll" ] = 1;
  joint_name_to_index_["r_hip_pitch"] = 2;
  joint_name_to_index_["r_knee"     ] = 3;
  joint_name_to_index_["r_ank_pitch"] = 4;
  joint_name_to_index_["r_ank_roll" ] = 5;

  joint_name_to_index_["l_hip_yaw"  ] = 6;
  joint_name_to_index_["l_hip_roll" ] = 7;
  joint_name_to_index_["l_hip_pitch"] = 8;
  joint_name_to_index_["l_knee"     ] = 9;
  joint_name_to_index_["l_ank_pitch"] = 10;
  joint_name_to_index_["l_ank_roll" ] = 11;

  previous_running_    = present_running    = false;

  gyro_roll_ = gyro_pitch_ = 0;
  orientation_roll_ = orientation_pitch_ = 0;

  r_foot_fx_N_  = r_foot_fy_N_  = r_foot_fz_N_  = 0;
  r_foot_Tx_Nm_ = r_foot_Ty_Nm_ = r_foot_Tz_Nm_ = 0;
  l_foot_fx_N_  = l_foot_fy_N_  = l_foot_fz_N_  = 0;
  l_foot_Tx_Nm_ = l_foot_Ty_Nm_ = l_foot_Tz_Nm_ = 0;

  desired_matrix_g_to_pelvis_   = Eigen::MatrixXd::Identity(4,4);
  desired_matrix_g_to_rfoot_    = Eigen::MatrixXd::Identity(4,4);
  desired_matrix_g_to_lfoot_    = Eigen::MatrixXd::Identity(4,4);

  balance_update_with_loop_ = false;
  balance_update_duration_ = 2.0;
  balance_update_sys_time_ = 2.0;
  balance_update_polynomial_coeff_.resize(6, 1);

  double tf = balance_update_duration_;
  Eigen::MatrixXd A(6,6), B(6, 1);
  A <<  0.0,              0.0,            0.0,        0.0,      0.0,    1.0,
        0.0,              0.0,            0.0,        0.0,      1.0,    0.0,
        0.0,              0.0,            0.0,        2.0,      0.0,    0.0,
        tf*tf*tf*tf*tf,   tf*tf*tf*tf,    tf*tf*tf,   tf*tf,    tf,     1.0,
        5.0*tf*tf*tf*tf,  4.0*tf*tf*tf,   3.0*tf*tf,  2.0*tf,   1.0,    0.0,
        20.0*tf*tf*tf,    12.0*tf*tf,     6.0*tf,     2.0,      0.0,    0.0;

  B << 0, 0, 0, 2.0, 0, 0;

  balance_update_polynomial_coeff_ = A.inverse() * B;

  joint_feedback_update_with_loop_ = false;
  joint_feedback_update_duration_ = 2.0;
  joint_feedback_update_sys_time_ = 2.0;
  joint_feedback_update_polynomial_coeff_ = balance_update_polynomial_coeff_;

  rot_x_pi_3d_.resize(3,3);
  rot_x_pi_3d_ << 1,  0,  0,
                  0, -1,  0,
                  0,  0, -1;
  rot_z_pi_3d_.resize(3,3);
  rot_z_pi_3d_ << -1,  0, 0,
                   0, -1, 0,
                   0,  0, 1;
}

OnlineWalkingModule::~OnlineWalkingModule()
{
  queue_thread_.join();
}

void OnlineWalkingModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  queue_thread_ = boost::thread(boost::bind(&OnlineWalkingModule::queueThread, this));
  control_cycle_msec_ = control_cycle_msec;

  Robotisop3OnlineWalking *online_walking = Robotisop3OnlineWalking::getInstance();
  online_walking->setInitialPose(0, -0.035, -0.2211555, 0, 0, 0,
                                 0,  0.035, -0.2211555, 0, 0, 0,
                                 0,      0,          0, 0, 0, 0);
  online_walking->setPreviewRefHeight(0.184686); // center of mass, z position

  online_walking->hip_roll_feedforward_angle_rad_ = 0.0*M_PI/180.0;
  //online_walking->balance_ctrl_.setCOBManualAdjustment(-10.0*0.001, 0, 0);

  online_walking->initialize(control_cycle_msec);

  process_mutex_.lock();
  desired_matrix_g_to_pelvis_ = online_walking->mat_g_to_pelvis_;
  desired_matrix_g_to_rfoot_  = online_walking->mat_g_to_rfoot_;
  desired_matrix_g_to_lfoot_  = online_walking->mat_g_to_lfoot_;
  process_mutex_.unlock();


  online_walking->start();
  online_walking->process();

  result_["r_hip_yaw"  ]->goal_position_ = online_walking->out_angle_rad_[0];
  result_["r_hip_roll" ]->goal_position_ = online_walking->out_angle_rad_[1];
  result_["r_hip_pitch"]->goal_position_ = online_walking->out_angle_rad_[2];
  result_["r_knee"     ]->goal_position_ = online_walking->out_angle_rad_[3];
  result_["r_ank_pitch"]->goal_position_ = online_walking->out_angle_rad_[4];
  result_["r_ank_roll" ]->goal_position_ = online_walking->out_angle_rad_[5];

  result_["l_hip_yaw"  ]->goal_position_ = online_walking->out_angle_rad_[6];
  result_["l_hip_roll" ]->goal_position_ = online_walking->out_angle_rad_[7];
  result_["l_hip_pitch"]->goal_position_ = online_walking->out_angle_rad_[8];
  result_["l_knee"     ]->goal_position_ = online_walking->out_angle_rad_[9];
  result_["l_ank_pitch"]->goal_position_ = online_walking->out_angle_rad_[10];
  result_["l_ank_roll" ]->goal_position_ = online_walking->out_angle_rad_[11];

  for(std::map<std::string, robotis_framework::DynamixelState*>::iterator it = result_.begin();
      it != result_.end();
      it++)
  {
    joint_name_to_pose_[it->first].position_     = it->second->goal_position_;
    joint_name_to_pose_[it->first].velocity_     = 0;
    joint_name_to_pose_[it->first].acceleration_ = 0;
  }

  desired_matrix_g_to_pelvis_ = online_walking->mat_g_to_pelvis_;
  desired_matrix_g_to_rfoot_  = online_walking->mat_g_to_rfoot_;
  desired_matrix_g_to_lfoot_  = online_walking->mat_g_to_lfoot_;

  r_foot_target_ft_.resize(6,1);
  l_foot_target_ft_.resize(6,1);
  r_foot_target_ft_.fill(0);
  l_foot_target_ft_.fill(0);

  r_foot_target_ft_.coeffRef(0,0) = online_walking->r_target_fx_N_;
  r_foot_target_ft_.coeffRef(1,0) = online_walking->r_target_fy_N_;
  r_foot_target_ft_.coeffRef(2,0) = online_walking->r_target_fz_N_;
  l_foot_target_ft_.coeffRef(0,0) = online_walking->l_target_fx_N_;
  l_foot_target_ft_.coeffRef(1,0) = online_walking->l_target_fy_N_;
  l_foot_target_ft_.coeffRef(2,0) = online_walking->l_target_fz_N_;

  previous_running_ = isRunning();

  online_walking->hip_roll_feedforward_angle_rad_ = 0.0;
//#ifdef _USE_PD_BALANCE_
//  online_walking->balance_ctrl_.foot_roll_gyro_ctrl_.p_gain_ = 0;
//  online_walking->balance_ctrl_.foot_roll_gyro_ctrl_.d_gain_ = 0;
//  online_walking->balance_ctrl_.foot_pitch_gyro_ctrl_.p_gain_ = 0;
//  online_walking->balance_ctrl_.foot_pitch_gyro_ctrl_.d_gain_ = 0;
//
//  online_walking->balance_ctrl_.foot_roll_angle_ctrl_.p_gain_ = 0;
//  online_walking->balance_ctrl_.foot_roll_angle_ctrl_.d_gain_ = 0;
//  online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.p_gain_ = 0;
//  online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.d_gain_ = 0;
//
//  online_walking->balance_ctrl_.right_foot_force_x_ctrl_.p_gain_ = 0;
//  online_walking->balance_ctrl_.right_foot_force_x_ctrl_.d_gain_ = 0;
//  online_walking->balance_ctrl_.right_foot_force_y_ctrl_.p_gain_ = 0;
//  online_walking->balance_ctrl_.right_foot_force_y_ctrl_.d_gain_ = 0;
//  online_walking->balance_ctrl_.right_foot_force_z_ctrl_.p_gain_ = 0;
//  online_walking->balance_ctrl_.right_foot_force_z_ctrl_.d_gain_ = 0;
//  online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.p_gain_ = 0;
//  online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.d_gain_ = 0;
//  online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.p_gain_ = 0;
//  online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.d_gain_ = 0;
//
//#else
//  online_walking->balance_ctrl_.setGyroBalanceGainRatio(0.0);
//
//  online_walking->balance_ctrl_.foot_roll_angle_ctrl_.gain_  = 0.0;
//  online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.gain_ = 0.0;
//
//  online_walking->balance_ctrl_.right_foot_force_x_ctrl_.gain_ = 0.0;
//  online_walking->balance_ctrl_.right_foot_force_y_ctrl_.gain_ = 0.0;
//  online_walking->balance_ctrl_.left_foot_force_x_ctrl_.gain_  = 0.0;
//  online_walking->balance_ctrl_.left_foot_force_y_ctrl_.gain_  = 0.0;
//
//  online_walking->balance_ctrl_.foot_force_z_diff_ctrl_.gain_ = 0.0;
//  online_walking->balance_ctrl_.right_foot_force_z_ctrl_.gain_ = 0.0;
//  online_walking->balance_ctrl_.left_foot_force_z_ctrl_.gain_ = 0.0;
//
//  online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.gain_  = 0.0;
//  online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.gain_ = 0.0;
//  online_walking->balance_ctrl_.left_foot_torque_roll_ctrl_.gain_   = 0.0;
//  online_walking->balance_ctrl_.left_foot_torque_pitch_ctrl_.gain_  = 0.0;
//#endif
}

void OnlineWalkingModule::queueThread()
{
  ros::NodeHandle     ros_node;
  ros::CallbackQueue  callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  /* publish topics */
  robot_pose_pub_ = ros_node.advertise<robotis_op3_motion_module_msgs::RobotPose>("/robotis/op3/walking/robot_pose", 1);
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("robotis/op3/status", 1);
  //pelvis_base_msg_pub_ = ros_node.advertise<geometry_msgs::PoseStamped>("/robotis/op3/pelvis_pose_base_walking", 1);
  //movement_done_pub_ = ros_node.advertise<std_msgs::String>("/robotis/op3/movement_done", 1);

  //walking_joint_states_pub_ = ros_node.advertise<robotis_op3_motion_module_msgs::WalkingJointStatesStamped>("/robotis/walking/walking_joint_states", 1);


  /* ROS Service Callback Functions */
  ros::ServiceServer get_ref_step_data_server  = ros_node.advertiseService("/robotis/op3/walking/get_reference_step_data",   &OnlineWalkingModule::getReferenceStepDataServiceCallback,   this);
  ros::ServiceServer add_step_data_array_sever = ros_node.advertiseService("/robotis/op3/walking/add_step_data",             &OnlineWalkingModule::addStepDataServiceCallback,            this);
  ros::ServiceServer walking_start_server      = ros_node.advertiseService("/robotis/op3/walking/walking_start",             &OnlineWalkingModule::startWalkingServiceCallback,           this);
  ros::ServiceServer is_running_server         = ros_node.advertiseService("/robotis/op3/walking/is_running",                &OnlineWalkingModule::IsRunningServiceCallback,              this);
  //ros::ServiceServer set_balance_param_server  = ros_node.advertiseService("/robotis/op3/walking/set_balance_param",         &OnlineWalkingModule::setBalanceParamServiceCallback,        this);
  ros::ServiceServer remove_existing_step_data = ros_node.advertiseService("/robotis/op3/walking/remove_existing_step_data", &OnlineWalkingModule::removeExistingStepDataServiceCallback, this);

  /* sensor topic subscribe */
  //ros::Subscriber imu_data_sub      = ros_node.subscribe("/robotis/sensor/imu/imu",                  3, &OnlineWalkingModule::imuDataOutputCallback,        this);
  //ros::Subscriber feedback_gain_sub = ros_node.subscribe("/robotis/op3/walking/joint_feedback_gain", 1, &OnlineWalkingModule::setJointFeedBackGainCallback, this);

  ros::WallDuration duration(control_cycle_msec_ / 1000.0);
  if(ros::param::get("gazebo", gazebo_) == false)
    gazebo_ = false;

  while(ros_node.ok())
    callback_queue.callAvailable(duration);

}

void OnlineWalkingModule::publishRobotPose(void)
{
  robot_pose_msg_.global_to_pelvis.position.x = desired_matrix_g_to_pelvis_.coeff(0, 3);
  robot_pose_msg_.global_to_pelvis.position.y = desired_matrix_g_to_pelvis_.coeff(1, 3);
  robot_pose_msg_.global_to_pelvis.position.z = desired_matrix_g_to_pelvis_.coeff(2, 3);
  Eigen::Quaterniond quaterniond_g_to_cob(desired_matrix_g_to_pelvis_.block<3, 3>(0, 0));


  robot_pose_msg_.global_to_right_foot.position.x = desired_matrix_g_to_rfoot_.coeff(0, 3);
  robot_pose_msg_.global_to_right_foot.position.y = desired_matrix_g_to_rfoot_.coeff(1, 3);
  robot_pose_msg_.global_to_right_foot.position.z = desired_matrix_g_to_rfoot_.coeff(2, 3);
  Eigen::Quaterniond quaterniond_g_to_rf(desired_matrix_g_to_rfoot_.block<3, 3>(0, 0));

  robot_pose_msg_.global_to_left_foot.position.x = desired_matrix_g_to_lfoot_.coeff(0, 3);
  robot_pose_msg_.global_to_left_foot.position.y = desired_matrix_g_to_lfoot_.coeff(1, 3);
  robot_pose_msg_.global_to_left_foot.position.z = desired_matrix_g_to_lfoot_.coeff(2, 3);
  Eigen::Quaterniond quaterniond_g_to_lf(desired_matrix_g_to_lfoot_.block<3, 3>(0, 0));

  tf::quaternionEigenToMsg(quaterniond_g_to_cob, robot_pose_msg_.global_to_pelvis.orientation);
  tf::quaternionEigenToMsg(quaterniond_g_to_rf,  robot_pose_msg_.global_to_right_foot.orientation);
  tf::quaternionEigenToMsg(quaterniond_g_to_lf,  robot_pose_msg_.global_to_left_foot.orientation);

  robot_pose_pub_.publish(robot_pose_msg_);

//  geometry_msgs::PoseStamped pose_msg;
//  pose_msg.header.stamp = ros::Time::now();
//
//  process_mutex_.lock();
//
//  Eigen::MatrixXd g_to_pelvis = desired_matrix_g_to_cob_ * robotis_framework::getTransformationXYZRPY(0, 0, 0.093, 0, 0, 0);
//  Eigen::Quaterniond pelvis_rotation = robotis_framework::convertRotationToQuaternion(desired_matrix_g_to_cob_);
//
//  pose_msg.pose.position.x = g_to_pelvis.coeff(0, 3);
//  pose_msg.pose.position.y = g_to_pelvis.coeff(1, 3);
//  pose_msg.pose.position.z = g_to_pelvis.coeff(2, 3) - 0.093;
//  tf::quaternionEigenToMsg(pelvis_rotation, pose_msg.pose.orientation);
//
//  process_mutex_.unlock();
//
//  pelvis_base_msg_pub_.publish(pose_msg);
}

void OnlineWalkingModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.type = type;
  status_msg.module_name = "Walking";
  status_msg.status_msg = msg;

  status_msg_pub_.publish(status_msg);
}

int OnlineWalkingModule::convertStepDataMsgToStepData(robotis_op3_motion_module_msgs::StepData& src, robotis_framework::StepData& des)
{
  int copy_result = robotis_op3_motion_module_msgs::AddStepDataArray::Response::NO_ERROR;
  des.time_data.   walking_state        = src.time_data.walking_state;
  des.time_data.abs_step_time           = src.time_data.abs_step_time;
  des.time_data.dsp_ratio               = src.time_data.dsp_ratio;

  des.position_data.moving_foot         = src.position_data.moving_foot;
  des.position_data.shoulder_swing_gain = 0;
  des.position_data.elbow_swing_gain    = 0;
  des.position_data.foot_z_swap         = src.position_data.foot_z_swap;
  des.position_data.waist_pitch_angle   = 0;
  des.position_data.waist_yaw_angle     = src.position_data.torso_yaw_angle_rad;
  des.position_data.body_z_swap         = src.position_data.body_z_swap;

  des.position_data.body_pose.z          = src.position_data.body_pose.z;
  des.position_data.body_pose.roll       = src.position_data.body_pose.roll;
  des.position_data.body_pose.pitch      = src.position_data.body_pose.pitch;
  des.position_data.body_pose.yaw        = src.position_data.body_pose.yaw;
  des.position_data.right_foot_pose.x     = src.position_data.right_foot_pose.x;
  des.position_data.right_foot_pose.y     = src.position_data.right_foot_pose.y;
  des.position_data.right_foot_pose.z     = src.position_data.right_foot_pose.z;
  des.position_data.right_foot_pose.roll  = src.position_data.right_foot_pose.roll;
  des.position_data.right_foot_pose.pitch = src.position_data.right_foot_pose.pitch;
  des.position_data.right_foot_pose.yaw   = src.position_data.right_foot_pose.yaw;
  des.position_data.left_foot_pose.x      = src.position_data.left_foot_pose.x;
  des.position_data.left_foot_pose.y      = src.position_data.left_foot_pose.y;
  des.position_data.left_foot_pose.z      = src.position_data.left_foot_pose.z;
  des.position_data.left_foot_pose.roll   = src.position_data.left_foot_pose.roll;
  des.position_data.left_foot_pose.pitch  = src.position_data.left_foot_pose.pitch;
  des.position_data.left_foot_pose.yaw    = src.position_data.left_foot_pose.yaw;

  des.time_data.start_time_delay_ratio_x         = src.time_data.start_time_delay_ratio_x;
  des.time_data.start_time_delay_ratio_y         = src.time_data.start_time_delay_ratio_y;
  des.time_data.start_time_delay_ratio_z         = src.time_data.start_time_delay_ratio_z;
  des.time_data.start_time_delay_ratio_roll      = src.time_data.start_time_delay_ratio_roll;
  des.time_data.start_time_delay_ratio_pitch     = src.time_data.start_time_delay_ratio_pitch;
  des.time_data.start_time_delay_ratio_yaw       = src.time_data.start_time_delay_ratio_yaw;

  des.time_data.finish_time_advance_ratio_x     = src.time_data.finish_time_advance_ratio_x;
  des.time_data.finish_time_advance_ratio_y     = src.time_data.finish_time_advance_ratio_y;
  des.time_data.finish_time_advance_ratio_z     = src.time_data.finish_time_advance_ratio_z;
  des.time_data.finish_time_advance_ratio_roll  = src.time_data.finish_time_advance_ratio_roll;
  des.time_data.finish_time_advance_ratio_pitch = src.time_data.finish_time_advance_ratio_pitch;
  des.time_data.finish_time_advance_ratio_yaw   = src.time_data.finish_time_advance_ratio_yaw;

  if((src.time_data.walking_state != robotis_op3_motion_module_msgs::StepTimeData::IN_WALKING_STARTING)
      && (src.time_data.walking_state != robotis_op3_motion_module_msgs::StepTimeData::IN_WALKING)
      && (src.time_data.walking_state != robotis_op3_motion_module_msgs::StepTimeData::IN_WALKING_ENDING) )
    copy_result |= robotis_op3_motion_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;

  if((src.time_data.start_time_delay_ratio_x     < 0)
      || (src.time_data.start_time_delay_ratio_y     < 0)
      || (src.time_data.start_time_delay_ratio_z     < 0)
      || (src.time_data.start_time_delay_ratio_roll  < 0)
      || (src.time_data.start_time_delay_ratio_pitch < 0)
      || (src.time_data.start_time_delay_ratio_yaw   < 0) )
    copy_result |= robotis_op3_motion_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;

  if((src.time_data.finish_time_advance_ratio_x     < 0)
      || (src.time_data.finish_time_advance_ratio_y     < 0)
      || (src.time_data.finish_time_advance_ratio_z     < 0)
      || (src.time_data.finish_time_advance_ratio_roll  < 0)
      || (src.time_data.finish_time_advance_ratio_pitch < 0)
      || (src.time_data.finish_time_advance_ratio_yaw   < 0) )
    copy_result |= robotis_op3_motion_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;

  if(((src.time_data.start_time_delay_ratio_x + src.time_data.finish_time_advance_ratio_x) > 1.0)
      || ((src.time_data.start_time_delay_ratio_y      + src.time_data.finish_time_advance_ratio_y     ) > 1.0)
      || ((src.time_data.start_time_delay_ratio_z      + src.time_data.finish_time_advance_ratio_z     ) > 1.0)
      || ((src.time_data.start_time_delay_ratio_roll   + src.time_data.finish_time_advance_ratio_roll  ) > 1.0)
      || ((src.time_data.start_time_delay_ratio_pitch  + src.time_data.finish_time_advance_ratio_pitch ) > 1.0)
      || ((src.time_data.start_time_delay_ratio_yaw    + src.time_data.finish_time_advance_ratio_yaw   ) > 1.0) )
    copy_result |= robotis_op3_motion_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;

  if((src.position_data.moving_foot != robotis_op3_motion_module_msgs::StepPositionData::STANDING)
      && (src.position_data.moving_foot != robotis_op3_motion_module_msgs::StepPositionData::RIGHT_FOOT_SWING)
      && (src.position_data.moving_foot != robotis_op3_motion_module_msgs::StepPositionData::LEFT_FOOT_SWING))
    copy_result |= robotis_op3_motion_module_msgs::AddStepDataArray::Response::PROBLEM_IN_POSITION_DATA;

  if(src.position_data.foot_z_swap < 0)
    copy_result |= robotis_op3_motion_module_msgs::AddStepDataArray::Response::PROBLEM_IN_POSITION_DATA;

  return copy_result;
}

int OnlineWalkingModule::convertStepDataToStepDataMsg(robotis_framework::StepData& src, robotis_op3_motion_module_msgs::StepData& des)
{
  des.time_data.walking_state   = src.time_data.walking_state;
  des.time_data.abs_step_time   = src.time_data.abs_step_time;
  des.time_data.dsp_ratio       = src.time_data.dsp_ratio;

  des.time_data.start_time_delay_ratio_x     = des.time_data.finish_time_advance_ratio_x     = 0;
  des.time_data.start_time_delay_ratio_y     = des.time_data.finish_time_advance_ratio_y     = 0;
  des.time_data.start_time_delay_ratio_z     = des.time_data.finish_time_advance_ratio_z     = 0;
  des.time_data.start_time_delay_ratio_roll  = des.time_data.finish_time_advance_ratio_roll  = 0;
  des.time_data.start_time_delay_ratio_pitch = des.time_data.finish_time_advance_ratio_pitch = 0;
  des.time_data.start_time_delay_ratio_yaw   = des.time_data.finish_time_advance_ratio_yaw   = 0;

  des.position_data.moving_foot         = src.position_data.moving_foot;
  des.position_data.foot_z_swap         = src.position_data.foot_z_swap;
  des.position_data.torso_yaw_angle_rad = src.position_data.waist_yaw_angle;
  des.position_data.body_z_swap         = src.position_data.body_z_swap;

  des.position_data.body_pose.z           = src.position_data.body_pose.z;
  des.position_data.body_pose.roll        = src.position_data.body_pose.roll;
  des.position_data.body_pose.pitch       = src.position_data.body_pose.pitch;
  des.position_data.body_pose.yaw         = src.position_data.body_pose.yaw;
  des.position_data.right_foot_pose.x     = src.position_data.right_foot_pose.x;
  des.position_data.right_foot_pose.y     = src.position_data.right_foot_pose.y;
  des.position_data.right_foot_pose.z     = src.position_data.right_foot_pose.z;
  des.position_data.right_foot_pose.roll  = src.position_data.right_foot_pose.roll;
  des.position_data.right_foot_pose.pitch = src.position_data.right_foot_pose.pitch;
  des.position_data.right_foot_pose.yaw   = src.position_data.right_foot_pose.yaw;
  des.position_data.left_foot_pose.x      = src.position_data.left_foot_pose.x;
  des.position_data.left_foot_pose.y      = src.position_data.left_foot_pose.y;
  des.position_data.left_foot_pose.z      = src.position_data.left_foot_pose.z;
  des.position_data.left_foot_pose.roll   = src.position_data.left_foot_pose.roll;
  des.position_data.left_foot_pose.pitch  = src.position_data.left_foot_pose.pitch;
  des.position_data.left_foot_pose.yaw    = src.position_data.left_foot_pose.yaw;

  return 0;
}

bool OnlineWalkingModule::getReferenceStepDataServiceCallback(robotis_op3_motion_module_msgs::GetReferenceStepData::Request &req,
                                                              robotis_op3_motion_module_msgs::GetReferenceStepData::Response &res)
{
  Robotisop3OnlineWalking *online_walking = Robotisop3OnlineWalking::getInstance();

  robotis_framework::StepData refStepData;

  online_walking->getReferenceStepDatafotAddition(&refStepData);

  convertStepDataToStepDataMsg(refStepData, res.reference_step_data);

  return true;
}

bool OnlineWalkingModule::addStepDataServiceCallback(robotis_op3_motion_module_msgs::AddStepDataArray::Request &req,
                                                     robotis_op3_motion_module_msgs::AddStepDataArray::Response &res)
{
  Robotisop3OnlineWalking *online_walking = Robotisop3OnlineWalking::getInstance();
  res.result = robotis_op3_motion_module_msgs::AddStepDataArray::Response::NO_ERROR;

  if(enable_ == false)
  {
    res.result |= robotis_op3_motion_module_msgs::AddStepDataArray::Response::NOT_ENABLED_WALKING_MODULE;
    std::string status_msg = WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return true;
  }

  if((req.step_data_array.size() > 100)
      && (req.remove_existing_step_data == true)
      && ((online_walking->isRunning() == true)))
  {
    res.result |= robotis_op3_motion_module_msgs::AddStepDataArray::Response::TOO_MANY_STEP_DATA;
    std::string status_msg  = WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return true;
  }

  robotis_framework::StepData step_data, ref_step_data;
  std::vector<robotis_framework::StepData> req_step_data_array;

  online_walking->getReferenceStepDatafotAddition(&ref_step_data);

  for(int i = 0; i < req.step_data_array.size(); i++)
  {
    res.result |= convertStepDataMsgToStepData(req.step_data_array[i], step_data);

    if(step_data.time_data.abs_step_time <= 0)
    {
      res.result |= robotis_op3_motion_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;
    }

    if(i != 0)
    {
      if(step_data.time_data.abs_step_time <= req_step_data_array[req_step_data_array.size() - 1].time_data.abs_step_time)
      {
        res.result |= robotis_op3_motion_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;
      }
    }
    else
    {
      if(step_data.time_data.abs_step_time <= ref_step_data.time_data.abs_step_time)
      {
        res.result |= robotis_op3_motion_module_msgs::AddStepDataArray::Response::PROBLEM_IN_TIME_DATA;
      }
    }

    if(res.result != robotis_op3_motion_module_msgs::AddStepDataArray::Response::NO_ERROR)
    {
      std::string status_msg = WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      return true;
    }

    req_step_data_array.push_back(step_data);
  }

  if(req.remove_existing_step_data == true)
  {
    int exist_num_of_step_data = online_walking->getNumofRemainingUnreservedStepData();
    if(exist_num_of_step_data != 0)
      for(int remove_count  = 0; remove_count < exist_num_of_step_data; remove_count++)
        online_walking->eraseLastStepData();
  }
  else
  {
    if(online_walking->isRunning() == true)
    {
      res.result |= robotis_op3_motion_module_msgs::AddStepDataArray::Response::ROBOT_IS_WALKING_NOW;
      std::string status_msg  = WalkingStatusMSG::FAILED_TO_ADD_STEP_DATA_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      return true;
    }
  }

  if(checkBalanceOnOff() == false)
  {
    std::string status_msg  = WalkingStatusMSG::BALANCE_HAS_BEEN_TURNED_OFF;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return true;
  }

  for(unsigned int i = 0; i < req_step_data_array.size() ; i++)
    online_walking->addStepData(req_step_data_array[i]);

  if( req.auto_start == true)
  {
    online_walking->start();
  }

  return true;
}

bool OnlineWalkingModule::startWalkingServiceCallback(robotis_op3_motion_module_msgs::StartWalking::Request &req,
                                                      robotis_op3_motion_module_msgs::StartWalking::Response &res)
{
  Robotisop3OnlineWalking *online_walking = Robotisop3OnlineWalking::getInstance();
  res.result = robotis_op3_motion_module_msgs::StartWalking::Response::NO_ERROR;

  if(enable_ == false)
  {
    res.result |= robotis_op3_motion_module_msgs::StartWalking::Response::NOT_ENABLED_WALKING_MODULE;
    return true;
  }

  if(online_walking->isRunning() == true)
  {
    res.result |= robotis_op3_motion_module_msgs::StartWalking::Response::ROBOT_IS_WALKING_NOW;
    return true;
  }

  if(checkBalanceOnOff() == false)
  {
    std::string status_msg  = WalkingStatusMSG::BALANCE_HAS_BEEN_TURNED_OFF;
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
    return true;
  }

  if(online_walking->getNumofRemainingUnreservedStepData() == 0)
  {
    res.result |= robotis_op3_motion_module_msgs::StartWalking::Response::NO_STEP_DATA;
    return true;
  }

  if(res.result == robotis_op3_motion_module_msgs::StartWalking::Response::NO_ERROR)
  {
    online_walking->start();
  }

  return true;
}

bool OnlineWalkingModule::IsRunningServiceCallback(robotis_op3_motion_module_msgs::IsRunning::Request &req,
                                                   robotis_op3_motion_module_msgs::IsRunning::Response &res)
{
  bool is_running = isRunning();
  res.is_running = is_running;

  return true;
}

bool OnlineWalkingModule::isRunning()
{
  return Robotisop3OnlineWalking::getInstance()->isRunning();
}

//void OnlineWalkingModule::setJointFeedBackGainCallback(const thormang3_online_walking_module_msgs::JointFeedBackGain::ConstPtr &msg)
//{
//  Robotisop3OnlineWalking *online_walking = Robotisop3OnlineWalking::getInstance();
//
//  joint_feedback_update_with_loop_ = true;
//  joint_feedback_update_sys_time_  = 0.0;
//
//  process_mutex_.lock();
//
//  desired_joint_feedback_gain_.r_leg_hip_y_p_gain  = msg->r_leg_hip_y_p_gain;
//  desired_joint_feedback_gain_.r_leg_hip_y_d_gain  = msg->r_leg_hip_y_d_gain;
//  desired_joint_feedback_gain_.r_leg_hip_r_p_gain  = msg->r_leg_hip_r_p_gain;
//  desired_joint_feedback_gain_.r_leg_hip_r_d_gain  = msg->r_leg_hip_r_d_gain;
//  desired_joint_feedback_gain_.r_leg_hip_p_p_gain  = msg->r_leg_hip_p_p_gain;
//  desired_joint_feedback_gain_.r_leg_hip_p_d_gain  = msg->r_leg_hip_p_d_gain;
//  desired_joint_feedback_gain_.r_leg_kn_p_p_gain   = msg->r_leg_kn_p_p_gain;
//  desired_joint_feedback_gain_.r_leg_kn_p_d_gain   = msg->r_leg_kn_p_d_gain;
//  desired_joint_feedback_gain_.r_leg_an_p_p_gain   = msg->r_leg_an_p_p_gain;
//  desired_joint_feedback_gain_.r_leg_an_p_d_gain   = msg->r_leg_an_p_d_gain;
//  desired_joint_feedback_gain_.r_leg_an_r_p_gain   = msg->r_leg_an_r_p_gain;
//  desired_joint_feedback_gain_.r_leg_an_r_d_gain   = msg->r_leg_an_r_d_gain;
//
//  desired_joint_feedback_gain_.l_leg_hip_y_p_gain  = msg->l_leg_hip_y_p_gain;
//  desired_joint_feedback_gain_.l_leg_hip_y_d_gain  = msg->l_leg_hip_y_d_gain;
//  desired_joint_feedback_gain_.l_leg_hip_r_p_gain  = msg->l_leg_hip_r_p_gain;
//  desired_joint_feedback_gain_.l_leg_hip_r_d_gain  = msg->l_leg_hip_r_d_gain;
//  desired_joint_feedback_gain_.l_leg_hip_p_p_gain  = msg->l_leg_hip_p_p_gain;
//  desired_joint_feedback_gain_.l_leg_hip_p_d_gain  = msg->l_leg_hip_p_d_gain;
//  desired_joint_feedback_gain_.l_leg_kn_p_p_gain   = msg->l_leg_kn_p_p_gain;
//  desired_joint_feedback_gain_.l_leg_kn_p_d_gain   = msg->l_leg_kn_p_d_gain;
//  desired_joint_feedback_gain_.l_leg_an_p_p_gain   = msg->l_leg_an_p_p_gain;
//  desired_joint_feedback_gain_.l_leg_an_p_d_gain   = msg->l_leg_an_p_d_gain;
//  desired_joint_feedback_gain_.l_leg_an_r_p_gain   = msg->l_leg_an_r_p_gain;
//  desired_joint_feedback_gain_.l_leg_an_r_d_gain   = msg->l_leg_an_r_d_gain;
//
//  previous_joint_feedback_gain_.r_leg_hip_y_p_gain = online_walking->leg_angle_feed_back_[0].p_gain_;
//  previous_joint_feedback_gain_.r_leg_hip_y_d_gain = online_walking->leg_angle_feed_back_[0].d_gain_;
//  previous_joint_feedback_gain_.r_leg_hip_r_p_gain = online_walking->leg_angle_feed_back_[1].p_gain_;
//  previous_joint_feedback_gain_.r_leg_hip_r_d_gain = online_walking->leg_angle_feed_back_[1].d_gain_;
//  previous_joint_feedback_gain_.r_leg_hip_p_p_gain = online_walking->leg_angle_feed_back_[2].p_gain_;
//  previous_joint_feedback_gain_.r_leg_hip_p_d_gain = online_walking->leg_angle_feed_back_[2].d_gain_;
//  previous_joint_feedback_gain_.r_leg_kn_p_p_gain  = online_walking->leg_angle_feed_back_[3].p_gain_;
//  previous_joint_feedback_gain_.r_leg_kn_p_d_gain  = online_walking->leg_angle_feed_back_[3].d_gain_;
//  previous_joint_feedback_gain_.r_leg_an_p_p_gain  = online_walking->leg_angle_feed_back_[4].p_gain_;
//  previous_joint_feedback_gain_.r_leg_an_p_d_gain  = online_walking->leg_angle_feed_back_[4].d_gain_;
//  previous_joint_feedback_gain_.r_leg_an_r_p_gain  = online_walking->leg_angle_feed_back_[5].p_gain_;
//  previous_joint_feedback_gain_.r_leg_an_r_d_gain  = online_walking->leg_angle_feed_back_[5].d_gain_;
//
//  previous_joint_feedback_gain_.l_leg_hip_y_p_gain = online_walking->leg_angle_feed_back_[6].p_gain_;
//  previous_joint_feedback_gain_.l_leg_hip_y_d_gain = online_walking->leg_angle_feed_back_[6].d_gain_;
//  previous_joint_feedback_gain_.l_leg_hip_r_p_gain = online_walking->leg_angle_feed_back_[7].p_gain_;
//  previous_joint_feedback_gain_.l_leg_hip_r_d_gain = online_walking->leg_angle_feed_back_[7].d_gain_;
//  previous_joint_feedback_gain_.l_leg_hip_p_p_gain = online_walking->leg_angle_feed_back_[8].p_gain_;
//  previous_joint_feedback_gain_.l_leg_hip_p_d_gain = online_walking->leg_angle_feed_back_[8].d_gain_;
//  previous_joint_feedback_gain_.l_leg_kn_p_p_gain  = online_walking->leg_angle_feed_back_[9].p_gain_;
//  previous_joint_feedback_gain_.l_leg_kn_p_d_gain  = online_walking->leg_angle_feed_back_[9].d_gain_;
//  previous_joint_feedback_gain_.l_leg_an_p_p_gain  = online_walking->leg_angle_feed_back_[10].p_gain_;
//  previous_joint_feedback_gain_.l_leg_an_p_d_gain  = online_walking->leg_angle_feed_back_[10].d_gain_;
//  previous_joint_feedback_gain_.l_leg_an_r_p_gain  = online_walking->leg_angle_feed_back_[11].p_gain_;
//  previous_joint_feedback_gain_.l_leg_an_r_d_gain  = online_walking->leg_angle_feed_back_[11].d_gain_;
//
//  process_mutex_.unlock();
//}

//bool OnlineWalkingModule::setBalanceParamServiceCallback(thormang3_online_walking_module_msgs::SetBalanceParam::Request  &req,
//                                                         thormang3_online_walking_module_msgs::SetBalanceParam::Response &res)
//{
//  THORMANG3OnlineWalking *online_walking = THORMANG3OnlineWalking::getInstance();
//  res.result = thormang3_online_walking_module_msgs::SetBalanceParam::Response::NO_ERROR;
//
//  if( enable_ == false)
//    res.result |= thormang3_online_walking_module_msgs::SetBalanceParam::Response::NOT_ENABLED_WALKING_MODULE;
//
//  if( balance_update_with_loop_ == true)
//  {
//    res.result |= thormang3_online_walking_module_msgs::SetBalanceParam::Response::PREV_REQUEST_IS_NOT_FINISHED;
//  }
//
//  if(res.result == thormang3_online_walking_module_msgs::SetBalanceParam::Response::NO_ERROR)
//  {
//    if( req.updating_duration < 0.0 )
//    {
//      // under 8ms apply immediately
//      setBalanceParam(req.balance_param);
//      std::string status_msg = WalkingStatusMSG::BALANCE_PARAM_SETTING_FINISH_MSG;
//      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
//      std_msgs::String done_msg;
//      done_msg.data = "balance_updated";
//      movement_done_pub_.publish(done_msg);
//      return true;
//    }
//    else
//    {
//      balance_update_duration_ = req.updating_duration;
//    }
//
//    balance_update_sys_time_ = 0.0;
//    balance_update_polynomial_coeff_.resize(6, 1);
//
//    double tf = balance_update_duration_;
//    Eigen::MatrixXd A(6,6), B(6, 1);
//    A <<  0.0,     0.0,     0.0,    0.0,    0.0, 1.0,
//        0.0,   0.0,    0.0,    0.0,       1.0, 0.0,
//        0.0,   0.0,    0.0,    2.0,       0.0, 0.0,
//        tf*tf*tf*tf*tf,     tf*tf*tf*tf,      tf*tf*tf,        tf*tf,     tf, 1.0,
//        5.0*tf*tf*tf*tf,    4.0*tf*tf*tf,    3.0*tf*tf,    2.0*tf,        1.0, 0.0,
//        20.0*tf*tf*tf,      12.0*tf*tf,       6.0*tf,        2.0,        0.0, 0.0;
//
//    B << 0, 0, 0, 1.0, 0, 0;
//    balance_update_polynomial_coeff_ = A.inverse() * B;
//
//    desired_balance_param_ = req.balance_param;
//
//    previous_balance_param_.cob_x_offset_m                  = online_walking->balance_ctrl_.getCOBManualAdjustmentX();
//    previous_balance_param_.cob_y_offset_m                  = online_walking->balance_ctrl_.getCOBManualAdjustmentY();
//
//    previous_balance_param_.hip_roll_swap_angle_rad         = online_walking->hip_roll_feedforward_angle_rad_;
//
//    ////gain
//    //gyro
//    previous_balance_param_.foot_roll_gyro_p_gain           = online_walking->balance_ctrl_.foot_roll_gyro_ctrl_.p_gain_;
//    previous_balance_param_.foot_roll_gyro_d_gain           = online_walking->balance_ctrl_.foot_roll_gyro_ctrl_.d_gain_;
//    previous_balance_param_.foot_pitch_gyro_p_gain          = online_walking->balance_ctrl_.foot_pitch_gyro_ctrl_.p_gain_;
//    previous_balance_param_.foot_pitch_gyro_d_gain          = online_walking->balance_ctrl_.foot_pitch_gyro_ctrl_.d_gain_;
//
//    //orientation
//    previous_balance_param_.foot_roll_angle_p_gain          = online_walking->balance_ctrl_.foot_roll_angle_ctrl_.p_gain_;
//    previous_balance_param_.foot_roll_angle_d_gain          = online_walking->balance_ctrl_.foot_roll_angle_ctrl_.d_gain_;
//    previous_balance_param_.foot_pitch_angle_p_gain         = online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.p_gain_;
//    previous_balance_param_.foot_pitch_angle_d_gain         = online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.d_gain_;
//
//    //force torque
//    previous_balance_param_.foot_x_force_p_gain               = online_walking->balance_ctrl_.right_foot_force_x_ctrl_.p_gain_;
//    previous_balance_param_.foot_y_force_p_gain               = online_walking->balance_ctrl_.right_foot_force_y_ctrl_.p_gain_;
//    previous_balance_param_.foot_z_force_p_gain               = online_walking->balance_ctrl_.right_foot_force_z_ctrl_.p_gain_;
//    previous_balance_param_.foot_roll_torque_p_gain           = online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.p_gain_;
//    previous_balance_param_.foot_pitch_torque_p_gain          = online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.p_gain_;
//
//    previous_balance_param_.foot_x_force_d_gain               = online_walking->balance_ctrl_.right_foot_force_x_ctrl_.d_gain_;
//    previous_balance_param_.foot_y_force_d_gain               = online_walking->balance_ctrl_.right_foot_force_y_ctrl_.d_gain_;
//    previous_balance_param_.foot_z_force_d_gain               = online_walking->balance_ctrl_.right_foot_force_z_ctrl_.d_gain_;
//    previous_balance_param_.foot_roll_torque_d_gain           = online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.d_gain_;
//    previous_balance_param_.foot_pitch_torque_d_gain          = online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.d_gain_;
//
//    ////cut off freq
//    //gyro
//    previous_balance_param_.roll_gyro_cut_off_frequency  = online_walking->balance_ctrl_.roll_gyro_lpf_.getCutOffFrequency();
//    previous_balance_param_.pitch_gyro_cut_off_frequency = online_walking->balance_ctrl_.pitch_gyro_lpf_.getCutOffFrequency();
//
//    //orientation
//    previous_balance_param_.roll_angle_cut_off_frequency  = online_walking->balance_ctrl_.roll_angle_lpf_.getCutOffFrequency();
//    previous_balance_param_.pitch_angle_cut_off_frequency = online_walking->balance_ctrl_.pitch_angle_lpf_.getCutOffFrequency();
//
//    //force torque
//    previous_balance_param_.foot_x_force_cut_off_frequency     = online_walking->balance_ctrl_.right_foot_force_x_lpf_.getCutOffFrequency();
//    previous_balance_param_.foot_y_force_cut_off_frequency     = online_walking->balance_ctrl_.right_foot_force_y_lpf_.getCutOffFrequency();
//    previous_balance_param_.foot_z_force_cut_off_frequency     = online_walking->balance_ctrl_.right_foot_force_z_lpf_.getCutOffFrequency();
//    previous_balance_param_.foot_roll_torque_cut_off_frequency  = online_walking->balance_ctrl_.right_foot_torque_roll_lpf_.getCutOffFrequency();
//    previous_balance_param_.foot_pitch_torque_cut_off_frequency = online_walking->balance_ctrl_.right_foot_torque_pitch_lpf_.getCutOffFrequency();
//
//    balance_update_with_loop_ = true;
//
//    std::string status_msg = WalkingStatusMSG::BALANCE_PARAM_SETTING_START_MSG;
//    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
//  }
//
//  return true;
//}

//bool OnlineWalkingModule::setBalanceParamServiceCallback(thormang3_walking_module_msgs::SetBalanceParam::Request  &req,
//                                                         thormang3_walking_module_msgs::SetBalanceParam::Response &res)
//{
//  THORMANG3OnlineWalking *online_walking = THORMANG3OnlineWalking::getInstance();
//  res.result = thormang3_walking_module_msgs::SetBalanceParam::Response::NO_ERROR;
//
//  if( enable_ == false)
//    res.result |= thormang3_walking_module_msgs::SetBalanceParam::Response::NOT_ENABLED_WALKING_MODULE;
//
//  if( balance_update_with_loop_ == true)
//  {
//    res.result |= thormang3_walking_module_msgs::SetBalanceParam::Response::PREV_REQUEST_IS_NOT_FINISHED;
//  }
//
//  if( ( req.balance_param.foot_roll_angle_time_constant  <= 0.0 )
//      || ( req.balance_param.foot_pitch_angle_time_constant  <= 0.0 )
//      || ( req.balance_param.foot_x_force_time_constant      <= 0.0 )
//      || ( req.balance_param.foot_y_force_time_constant      <= 0.0 )
//      || ( req.balance_param.foot_z_force_time_constant      <= 0.0 )
//      || ( req.balance_param.foot_roll_torque_time_constant  <= 0.0 )
//      || ( req.balance_param.foot_pitch_torque_time_constant <= 0.0 ) )
//  {
//    res.result |= thormang3_walking_module_msgs::SetBalanceParam::Response::TIME_CONST_IS_ZERO_OR_NEGATIVE;
//  }
//
//  if(res.result == thormang3_walking_module_msgs::SetBalanceParam::Response::NO_ERROR)
//  {
//    if( req.updating_duration < 0.0 )
//    {
//      // under 8ms apply immediately
//      setBalanceParam(req.balance_param);
//      std::string status_msg = WalkingStatusMSG::BALANCE_PARAM_SETTING_FINISH_MSG;
//      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
//      return true;
//    }
//    else
//    {
//      balance_update_duration_ = req.updating_duration;
//    }
//
//    balance_update_sys_time_ = 0.0;
//    balance_update_polynomial_coeff_.resize(6, 1);
//
//    double tf = balance_update_duration_;
//    Eigen::MatrixXd A(6,6), B(6, 1);
//    A <<  0.0,     0.0,     0.0,    0.0,    0.0, 1.0,
//        0.0,   0.0,    0.0,    0.0,       1.0, 0.0,
//        0.0,   0.0,    0.0,    2.0,       0.0, 0.0,
//        tf*tf*tf*tf*tf,     tf*tf*tf*tf,      tf*tf*tf,        tf*tf,     tf, 1.0,
//        5.0*tf*tf*tf*tf,    4.0*tf*tf*tf,    3.0*tf*tf,    2.0*tf,        1.0, 0.0,
//        20.0*tf*tf*tf,      12.0*tf*tf,       6.0*tf,        2.0,        0.0, 0.0;
//
//    B << 0, 0, 0, 1.0, 0, 0;
//    balance_update_polynomial_coeff_ = A.inverse() * B;
//
//    desired_balance_param_ = req.balance_param;
//
//    previous_balance_param_.cob_x_offset_m                  = online_walking->balance_ctrl_.getCOBManualAdjustmentX();
//    previous_balance_param_.cob_y_offset_m                  = online_walking->balance_ctrl_.getCOBManualAdjustmentY();
//
//    previous_balance_param_.hip_roll_swap_angle_rad         = online_walking->hip_roll_feedforward_angle_rad_;
//
//    previous_balance_param_.gyro_gain                       = online_walking->balance_ctrl_.getGyroBalanceGainRatio();
//    previous_balance_param_.foot_roll_angle_gain            = online_walking->balance_ctrl_.foot_roll_angle_ctrl_.gain_;
//    previous_balance_param_.foot_pitch_angle_gain           = online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.gain_;
//
//    previous_balance_param_.foot_x_force_gain               = online_walking->balance_ctrl_.right_foot_force_x_ctrl_.gain_;
//    previous_balance_param_.foot_y_force_gain               = online_walking->balance_ctrl_.right_foot_force_y_ctrl_.gain_;
//    previous_balance_param_.foot_z_force_gain               = online_walking->balance_ctrl_.foot_force_z_diff_ctrl_.gain_;
//    previous_balance_param_.foot_roll_torque_gain           = online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.gain_;
//    previous_balance_param_.foot_pitch_torque_gain          = online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.gain_;
//
//    previous_balance_param_.foot_roll_angle_time_constant   = online_walking->balance_ctrl_.foot_roll_angle_ctrl_.time_constant_sec_;
//    previous_balance_param_.foot_pitch_angle_time_constant  = online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.time_constant_sec_;
//
//    previous_balance_param_.foot_x_force_time_constant      = online_walking->balance_ctrl_.right_foot_force_x_ctrl_.time_constant_sec_;
//    previous_balance_param_.foot_y_force_time_constant      = online_walking->balance_ctrl_.right_foot_force_y_ctrl_.time_constant_sec_;
//    previous_balance_param_.foot_z_force_time_constant      = online_walking->balance_ctrl_.foot_force_z_diff_ctrl_.time_constant_sec_;
//    previous_balance_param_.foot_roll_torque_time_constant  = online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.time_constant_sec_;
//    previous_balance_param_.foot_pitch_torque_time_constant = online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.time_constant_sec_;
//
//    balance_update_with_loop_ = true;
//
//    std::string status_msg = WalkingStatusMSG::BALANCE_PARAM_SETTING_START_MSG;
//    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
//  }
//
//  return true;
//}
//#endif
//
//
//#ifdef _USE_PD_BALANCE_
//void OnlineWalkingModule::setBalanceParam(thormang3_online_walking_module_msgs::BalanceParam& balance_param_msg)
//{
//  THORMANG3OnlineWalking *online_walking = THORMANG3OnlineWalking::getInstance();
//
//  online_walking->hip_roll_feedforward_angle_rad_ = balance_param_msg.hip_roll_swap_angle_rad;
//
//  online_walking->balance_ctrl_.setCOBManualAdjustment(balance_param_msg.cob_x_offset_m, balance_param_msg.cob_y_offset_m, 0);
//
//  //// set gain
//  //gyro
//  online_walking->balance_ctrl_.foot_roll_gyro_ctrl_.p_gain_ = balance_param_msg.foot_roll_gyro_p_gain;
//  online_walking->balance_ctrl_.foot_roll_gyro_ctrl_.d_gain_ = balance_param_msg.foot_roll_gyro_d_gain;
//  online_walking->balance_ctrl_.foot_pitch_gyro_ctrl_.p_gain_ = balance_param_msg.foot_pitch_gyro_p_gain;
//  online_walking->balance_ctrl_.foot_pitch_gyro_ctrl_.d_gain_ = balance_param_msg.foot_pitch_gyro_d_gain;
//
//  //orientation
//  online_walking->balance_ctrl_.foot_roll_angle_ctrl_.p_gain_  = balance_param_msg.foot_roll_angle_p_gain;
//  online_walking->balance_ctrl_.foot_roll_angle_ctrl_.d_gain_  = balance_param_msg.foot_roll_angle_d_gain;
//  online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.p_gain_ = balance_param_msg.foot_pitch_angle_p_gain;
//  online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.d_gain_ = balance_param_msg.foot_pitch_angle_d_gain;
//
//  //force torque
//  online_walking->balance_ctrl_.right_foot_force_x_ctrl_.p_gain_      = balance_param_msg.foot_x_force_p_gain;
//  online_walking->balance_ctrl_.right_foot_force_y_ctrl_.p_gain_      = balance_param_msg.foot_y_force_p_gain;
//  online_walking->balance_ctrl_.right_foot_force_z_ctrl_.p_gain_      = balance_param_msg.foot_z_force_p_gain;
//  online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.p_gain_  = balance_param_msg.foot_roll_torque_p_gain;
//  online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.p_gain_ = balance_param_msg.foot_roll_torque_p_gain;
//  online_walking->balance_ctrl_.right_foot_force_x_ctrl_.d_gain_      = balance_param_msg.foot_x_force_d_gain;
//  online_walking->balance_ctrl_.right_foot_force_y_ctrl_.d_gain_      = balance_param_msg.foot_y_force_d_gain;
//  online_walking->balance_ctrl_.right_foot_force_z_ctrl_.d_gain_      = balance_param_msg.foot_z_force_d_gain;
//  online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.d_gain_  = balance_param_msg.foot_roll_torque_d_gain;
//  online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.d_gain_ = balance_param_msg.foot_roll_torque_d_gain;
//
//  online_walking->balance_ctrl_.left_foot_force_x_ctrl_.p_gain_      = balance_param_msg.foot_x_force_p_gain;
//  online_walking->balance_ctrl_.left_foot_force_y_ctrl_.p_gain_      = balance_param_msg.foot_y_force_p_gain;
//  online_walking->balance_ctrl_.left_foot_force_z_ctrl_.p_gain_      = balance_param_msg.foot_z_force_p_gain;
//  online_walking->balance_ctrl_.left_foot_torque_roll_ctrl_.p_gain_  = balance_param_msg.foot_roll_torque_p_gain;
//  online_walking->balance_ctrl_.left_foot_torque_pitch_ctrl_.p_gain_ = balance_param_msg.foot_roll_torque_p_gain;
//  online_walking->balance_ctrl_.left_foot_force_x_ctrl_.d_gain_      = balance_param_msg.foot_x_force_d_gain;
//  online_walking->balance_ctrl_.left_foot_force_y_ctrl_.d_gain_      = balance_param_msg.foot_y_force_d_gain;
//  online_walking->balance_ctrl_.left_foot_force_z_ctrl_.d_gain_      = balance_param_msg.foot_z_force_d_gain;
//  online_walking->balance_ctrl_.left_foot_torque_roll_ctrl_.d_gain_  = balance_param_msg.foot_roll_torque_d_gain;
//  online_walking->balance_ctrl_.left_foot_torque_pitch_ctrl_.d_gain_ = balance_param_msg.foot_roll_torque_d_gain;
//
//  //// set cut off freq
//  online_walking->balance_ctrl_.roll_gyro_lpf_.setCutOffFrequency(balance_param_msg.roll_gyro_cut_off_frequency);
//  online_walking->balance_ctrl_.pitch_gyro_lpf_.setCutOffFrequency(balance_param_msg.pitch_gyro_cut_off_frequency);
//  online_walking->balance_ctrl_.roll_angle_lpf_.setCutOffFrequency(balance_param_msg.roll_angle_cut_off_frequency);
//  online_walking->balance_ctrl_.pitch_angle_lpf_.setCutOffFrequency(balance_param_msg.pitch_angle_cut_off_frequency);
//
//  online_walking->balance_ctrl_.right_foot_force_x_lpf_.setCutOffFrequency(balance_param_msg.foot_x_force_cut_off_frequency);
//  online_walking->balance_ctrl_.right_foot_force_y_lpf_.setCutOffFrequency(balance_param_msg.foot_y_force_cut_off_frequency);
//  online_walking->balance_ctrl_.right_foot_force_z_lpf_.setCutOffFrequency(balance_param_msg.foot_z_force_cut_off_frequency);
//  online_walking->balance_ctrl_.right_foot_torque_roll_lpf_.setCutOffFrequency(balance_param_msg.foot_roll_torque_cut_off_frequency);
//  online_walking->balance_ctrl_.right_foot_torque_pitch_lpf_.setCutOffFrequency(balance_param_msg.foot_pitch_torque_cut_off_frequency);
//
//  online_walking->balance_ctrl_.left_foot_force_x_lpf_.setCutOffFrequency(balance_param_msg.foot_x_force_cut_off_frequency);
//  online_walking->balance_ctrl_.left_foot_force_y_lpf_.setCutOffFrequency(balance_param_msg.foot_y_force_cut_off_frequency);
//  online_walking->balance_ctrl_.left_foot_force_z_lpf_.setCutOffFrequency(balance_param_msg.foot_z_force_cut_off_frequency);
//  online_walking->balance_ctrl_.left_foot_torque_roll_lpf_.setCutOffFrequency(balance_param_msg.foot_roll_torque_cut_off_frequency);
//  online_walking->balance_ctrl_.left_foot_torque_pitch_lpf_.setCutOffFrequency(balance_param_msg.foot_pitch_torque_cut_off_frequency);
//}
//#else
//void OnlineWalkingModule::setBalanceParam(thormang3_walking_module_msgs::BalanceParam& balance_param_msg)
//{
//  THORMANG3OnlineWalking *online_walking = THORMANG3OnlineWalking::getInstance();
//
//  online_walking->hip_roll_feedforward_angle_rad_ = balance_param_msg.hip_roll_swap_angle_rad;
//
//  online_walking->balance_ctrl_.setCOBManualAdjustment(balance_param_msg.cob_x_offset_m, balance_param_msg.cob_y_offset_m, 0);
//
//  // set gain
//  online_walking->balance_ctrl_.setGyroBalanceGainRatio(balance_param_msg.gyro_gain);
//
//  online_walking->balance_ctrl_.foot_roll_angle_ctrl_.gain_  = balance_param_msg.foot_roll_angle_gain;
//  online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.gain_ = balance_param_msg.foot_pitch_angle_gain;
//
//  online_walking->balance_ctrl_.right_foot_force_x_ctrl_.gain_ = balance_param_msg.foot_x_force_gain;
//  online_walking->balance_ctrl_.right_foot_force_y_ctrl_.gain_ = balance_param_msg.foot_y_force_gain;
//  online_walking->balance_ctrl_.left_foot_force_x_ctrl_.gain_  = balance_param_msg.foot_x_force_gain;
//  online_walking->balance_ctrl_.left_foot_force_y_ctrl_.gain_  = balance_param_msg.foot_y_force_gain;
//
//  online_walking->balance_ctrl_.foot_force_z_diff_ctrl_.gain_ = balance_param_msg.foot_z_force_gain;
//  online_walking->balance_ctrl_.right_foot_force_z_ctrl_.gain_ = balance_param_msg.foot_z_force_gain;
//  online_walking->balance_ctrl_.left_foot_force_z_ctrl_.gain_ = balance_param_msg.foot_z_force_gain;
//
//  online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.gain_  = balance_param_msg.foot_roll_torque_gain;
//  online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.gain_ = balance_param_msg.foot_roll_torque_gain;
//  online_walking->balance_ctrl_.left_foot_torque_roll_ctrl_.gain_   = balance_param_msg.foot_pitch_torque_gain;
//  online_walking->balance_ctrl_.left_foot_torque_pitch_ctrl_.gain_  = balance_param_msg.foot_pitch_torque_gain;
//
//  // set time_const
//  online_walking->balance_ctrl_.foot_roll_angle_ctrl_.time_constant_sec_  = balance_param_msg.foot_roll_angle_time_constant;
//  online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.time_constant_sec_ = balance_param_msg.foot_pitch_angle_time_constant;
//
//  online_walking->balance_ctrl_.right_foot_force_x_ctrl_.time_constant_sec_ = balance_param_msg.foot_x_force_time_constant;
//  online_walking->balance_ctrl_.right_foot_force_y_ctrl_.time_constant_sec_ = balance_param_msg.foot_y_force_time_constant;
//  online_walking->balance_ctrl_.left_foot_force_x_ctrl_.time_constant_sec_  = balance_param_msg.foot_x_force_time_constant;
//  online_walking->balance_ctrl_.left_foot_force_y_ctrl_.time_constant_sec_  = balance_param_msg.foot_y_force_time_constant;
//
//  online_walking->balance_ctrl_.foot_force_z_diff_ctrl_.time_constant_sec_ = balance_param_msg.foot_z_force_time_constant;
//  online_walking->balance_ctrl_.right_foot_force_z_ctrl_.time_constant_sec_ = balance_param_msg.foot_z_force_time_constant;
//  online_walking->balance_ctrl_.left_foot_force_z_ctrl_.time_constant_sec_ = balance_param_msg.foot_z_force_time_constant;
//
//  online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.time_constant_sec_  = balance_param_msg.foot_roll_torque_time_constant;
//  online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.time_constant_sec_ = balance_param_msg.foot_pitch_torque_time_constant;
//  online_walking->balance_ctrl_.left_foot_torque_roll_ctrl_.time_constant_sec_   = balance_param_msg.foot_roll_torque_time_constant;
//  online_walking->balance_ctrl_.left_foot_torque_pitch_ctrl_.time_constant_sec_  = balance_param_msg.foot_pitch_torque_time_constant;
//}
//#endif

//void OnlineWalkingModule::updateBalanceParam()
//{
//  double current_update_gain =  balance_update_polynomial_coeff_.coeff(0,0) * robotis_framework::powDI(balance_update_sys_time_ , 5)
//  + balance_update_polynomial_coeff_.coeff(1,0) * robotis_framework::powDI(balance_update_sys_time_ , 4)
//  + balance_update_polynomial_coeff_.coeff(2,0) * robotis_framework::powDI(balance_update_sys_time_ , 3)
//  + balance_update_polynomial_coeff_.coeff(3,0) * robotis_framework::powDI(balance_update_sys_time_ , 2)
//  + balance_update_polynomial_coeff_.coeff(4,0) * robotis_framework::powDI(balance_update_sys_time_ , 1)
//  + balance_update_polynomial_coeff_.coeff(5,0) ;
//
//  current_balance_param_.cob_x_offset_m                  = current_update_gain*(desired_balance_param_.cob_x_offset_m                   - previous_balance_param_.cob_x_offset_m                ) + previous_balance_param_.cob_x_offset_m;
//  current_balance_param_.cob_y_offset_m                  = current_update_gain*(desired_balance_param_.cob_y_offset_m                   - previous_balance_param_.cob_y_offset_m                ) + previous_balance_param_.cob_y_offset_m;
//
//  current_balance_param_.hip_roll_swap_angle_rad         = current_update_gain*(desired_balance_param_.hip_roll_swap_angle_rad         - previous_balance_param_.hip_roll_swap_angle_rad        ) + previous_balance_param_.hip_roll_swap_angle_rad;
//
//#ifdef _USE_PD_BALANCE_
//  current_balance_param_.foot_roll_gyro_p_gain                = current_update_gain*(desired_balance_param_.foot_roll_gyro_p_gain                - previous_balance_param_.foot_roll_gyro_p_gain              ) + previous_balance_param_.foot_roll_gyro_p_gain;
//  current_balance_param_.foot_roll_gyro_d_gain                = current_update_gain*(desired_balance_param_.foot_roll_gyro_d_gain                - previous_balance_param_.foot_roll_gyro_d_gain              ) + previous_balance_param_.foot_roll_gyro_d_gain;
//  current_balance_param_.foot_pitch_gyro_p_gain               = current_update_gain*(desired_balance_param_.foot_pitch_gyro_p_gain               - previous_balance_param_.foot_pitch_gyro_p_gain             ) + previous_balance_param_.foot_pitch_gyro_p_gain;
//  current_balance_param_.foot_pitch_gyro_d_gain               = current_update_gain*(desired_balance_param_.foot_pitch_gyro_d_gain               - previous_balance_param_.foot_pitch_gyro_d_gain             ) + previous_balance_param_.foot_pitch_gyro_d_gain;
//  current_balance_param_.foot_roll_angle_p_gain               = current_update_gain*(desired_balance_param_.foot_roll_angle_p_gain               - previous_balance_param_.foot_roll_angle_p_gain             ) + previous_balance_param_.foot_roll_angle_p_gain;
//  current_balance_param_.foot_roll_angle_d_gain               = current_update_gain*(desired_balance_param_.foot_roll_angle_d_gain               - previous_balance_param_.foot_roll_angle_d_gain             ) + previous_balance_param_.foot_roll_angle_d_gain;
//  current_balance_param_.foot_pitch_angle_p_gain              = current_update_gain*(desired_balance_param_.foot_pitch_angle_p_gain              - previous_balance_param_.foot_pitch_angle_p_gain            ) + previous_balance_param_.foot_pitch_angle_p_gain;
//  current_balance_param_.foot_pitch_angle_d_gain              = current_update_gain*(desired_balance_param_.foot_pitch_angle_d_gain              - previous_balance_param_.foot_pitch_angle_d_gain            ) + previous_balance_param_.foot_pitch_angle_d_gain;
//  current_balance_param_.foot_x_force_p_gain                  = current_update_gain*(desired_balance_param_.foot_x_force_p_gain                  - previous_balance_param_.foot_x_force_p_gain                ) + previous_balance_param_.foot_x_force_p_gain;
//  current_balance_param_.foot_y_force_p_gain                  = current_update_gain*(desired_balance_param_.foot_y_force_p_gain                  - previous_balance_param_.foot_y_force_p_gain                ) + previous_balance_param_.foot_y_force_p_gain;
//  current_balance_param_.foot_z_force_p_gain                  = current_update_gain*(desired_balance_param_.foot_z_force_p_gain                  - previous_balance_param_.foot_z_force_p_gain                ) + previous_balance_param_.foot_z_force_p_gain;
//  current_balance_param_.foot_roll_torque_p_gain              = current_update_gain*(desired_balance_param_.foot_roll_torque_p_gain              - previous_balance_param_.foot_roll_torque_p_gain            ) + previous_balance_param_.foot_roll_torque_p_gain;
//  current_balance_param_.foot_pitch_torque_p_gain             = current_update_gain*(desired_balance_param_.foot_pitch_torque_p_gain             - previous_balance_param_.foot_pitch_torque_p_gain           ) + previous_balance_param_.foot_pitch_torque_p_gain;
//  current_balance_param_.foot_x_force_d_gain                  = current_update_gain*(desired_balance_param_.foot_x_force_d_gain                  - previous_balance_param_.foot_x_force_d_gain                ) + previous_balance_param_.foot_x_force_d_gain;
//  current_balance_param_.foot_y_force_d_gain                  = current_update_gain*(desired_balance_param_.foot_y_force_d_gain                  - previous_balance_param_.foot_y_force_d_gain                ) + previous_balance_param_.foot_y_force_d_gain;
//  current_balance_param_.foot_z_force_d_gain                  = current_update_gain*(desired_balance_param_.foot_z_force_d_gain                  - previous_balance_param_.foot_z_force_d_gain                ) + previous_balance_param_.foot_z_force_d_gain;
//  current_balance_param_.foot_roll_torque_d_gain              = current_update_gain*(desired_balance_param_.foot_roll_torque_d_gain              - previous_balance_param_.foot_roll_torque_d_gain            ) + previous_balance_param_.foot_roll_torque_d_gain;
//  current_balance_param_.foot_pitch_torque_d_gain             = current_update_gain*(desired_balance_param_.foot_pitch_torque_d_gain             - previous_balance_param_.foot_pitch_torque_d_gain           ) + previous_balance_param_.foot_pitch_torque_d_gain;
//
//  current_balance_param_.roll_gyro_cut_off_frequency          = current_update_gain*(desired_balance_param_.roll_gyro_cut_off_frequency          - previous_balance_param_.roll_gyro_cut_off_frequency        ) + previous_balance_param_.roll_gyro_cut_off_frequency;
//  current_balance_param_.pitch_gyro_cut_off_frequency         = current_update_gain*(desired_balance_param_.pitch_gyro_cut_off_frequency         - previous_balance_param_.pitch_gyro_cut_off_frequency       ) + previous_balance_param_.pitch_gyro_cut_off_frequency;
//  current_balance_param_.roll_angle_cut_off_frequency         = current_update_gain*(desired_balance_param_.roll_angle_cut_off_frequency         - previous_balance_param_.roll_angle_cut_off_frequency       ) + previous_balance_param_.roll_angle_cut_off_frequency;
//  current_balance_param_.pitch_angle_cut_off_frequency        = current_update_gain*(desired_balance_param_.pitch_angle_cut_off_frequency        - previous_balance_param_.pitch_angle_cut_off_frequency      ) + previous_balance_param_.pitch_angle_cut_off_frequency;
//  current_balance_param_.foot_x_force_cut_off_frequency       = current_update_gain*(desired_balance_param_.foot_x_force_cut_off_frequency       - previous_balance_param_.foot_x_force_cut_off_frequency     ) + previous_balance_param_.foot_x_force_cut_off_frequency;
//  current_balance_param_.foot_y_force_cut_off_frequency       = current_update_gain*(desired_balance_param_.foot_y_force_cut_off_frequency       - previous_balance_param_.foot_y_force_cut_off_frequency     ) + previous_balance_param_.foot_y_force_cut_off_frequency;
//  current_balance_param_.foot_z_force_cut_off_frequency       = current_update_gain*(desired_balance_param_.foot_z_force_cut_off_frequency       - previous_balance_param_.foot_z_force_cut_off_frequency     ) + previous_balance_param_.foot_z_force_cut_off_frequency;
//  current_balance_param_.foot_roll_torque_cut_off_frequency   = current_update_gain*(desired_balance_param_.foot_roll_torque_cut_off_frequency   - previous_balance_param_.foot_roll_torque_cut_off_frequency ) + previous_balance_param_.foot_roll_torque_cut_off_frequency;
//  current_balance_param_.foot_pitch_torque_cut_off_frequency  = current_update_gain*(desired_balance_param_.foot_pitch_torque_cut_off_frequency  - previous_balance_param_.foot_pitch_torque_cut_off_frequency) + previous_balance_param_.foot_pitch_torque_cut_off_frequency;
//#else
//  current_balance_param_.gyro_gain                       = current_update_gain*(desired_balance_param_.gyro_gain                         - previous_balance_param_.gyro_gain                    ) + previous_balance_param_.gyro_gain;
//  current_balance_param_.foot_roll_angle_gain            = current_update_gain*(desired_balance_param_.foot_roll_angle_gain            - previous_balance_param_.foot_roll_angle_gain           ) + previous_balance_param_.foot_roll_angle_gain;
//  current_balance_param_.foot_pitch_angle_gain           = current_update_gain*(desired_balance_param_.foot_pitch_angle_gain           - previous_balance_param_.foot_pitch_angle_gain          ) + previous_balance_param_.foot_pitch_angle_gain;
//
//  current_balance_param_.foot_x_force_gain               = current_update_gain*(desired_balance_param_.foot_x_force_gain               - previous_balance_param_.foot_x_force_gain              ) + previous_balance_param_.foot_x_force_gain;
//  current_balance_param_.foot_y_force_gain               = current_update_gain*(desired_balance_param_.foot_y_force_gain               - previous_balance_param_.foot_y_force_gain              ) + previous_balance_param_.foot_y_force_gain;
//  current_balance_param_.foot_z_force_gain               = current_update_gain*(desired_balance_param_.foot_z_force_gain               - previous_balance_param_.foot_z_force_gain              ) + previous_balance_param_.foot_z_force_gain;
//  current_balance_param_.foot_roll_torque_gain           = current_update_gain*(desired_balance_param_.foot_roll_torque_gain           - previous_balance_param_.foot_roll_torque_gain          ) + previous_balance_param_.foot_roll_torque_gain;
//  current_balance_param_.foot_pitch_torque_gain          = current_update_gain*(desired_balance_param_.foot_pitch_torque_gain          - previous_balance_param_.foot_pitch_torque_gain         ) + previous_balance_param_.foot_pitch_torque_gain;
//
//  current_balance_param_.foot_roll_angle_time_constant   = current_update_gain*(desired_balance_param_.foot_roll_angle_time_constant   - previous_balance_param_.foot_roll_angle_time_constant  ) + previous_balance_param_.foot_roll_angle_time_constant;
//  current_balance_param_.foot_pitch_angle_time_constant  = current_update_gain*(desired_balance_param_.foot_pitch_angle_time_constant  - previous_balance_param_.foot_pitch_angle_time_constant ) + previous_balance_param_.foot_pitch_angle_time_constant;
//
//  current_balance_param_.foot_x_force_time_constant      = current_update_gain*(desired_balance_param_.foot_x_force_time_constant      - previous_balance_param_.foot_x_force_time_constant     ) + previous_balance_param_.foot_x_force_time_constant;
//  current_balance_param_.foot_y_force_time_constant      = current_update_gain*(desired_balance_param_.foot_y_force_time_constant      - previous_balance_param_.foot_y_force_time_constant     ) + previous_balance_param_.foot_y_force_time_constant;
//  current_balance_param_.foot_z_force_time_constant      = current_update_gain*(desired_balance_param_.foot_z_force_time_constant      - previous_balance_param_.foot_z_force_time_constant     ) + previous_balance_param_.foot_z_force_time_constant;
//  current_balance_param_.foot_roll_torque_time_constant  = current_update_gain*(desired_balance_param_.foot_roll_torque_time_constant  - previous_balance_param_.foot_roll_torque_time_constant ) + previous_balance_param_.foot_roll_torque_time_constant;
//  current_balance_param_.foot_pitch_torque_time_constant = current_update_gain*(desired_balance_param_.foot_pitch_torque_time_constant - previous_balance_param_.foot_pitch_torque_time_constant) + previous_balance_param_.foot_pitch_torque_time_constant;
//#endif
//  setBalanceParam(current_balance_param_);
//}

//void OnlineWalkingModule::setJointFeedBackGain(thormang3_online_walking_module_msgs::JointFeedBackGain& msg)
//{
//  THORMANG3OnlineWalking *online_walking = THORMANG3OnlineWalking::getInstance();
//  online_walking->leg_angle_feed_back_[0].p_gain_  = msg.r_leg_hip_y_p_gain ;
//  online_walking->leg_angle_feed_back_[0].d_gain_  = msg.r_leg_hip_y_d_gain ;
//  online_walking->leg_angle_feed_back_[1].p_gain_  = msg.r_leg_hip_r_p_gain ;
//  online_walking->leg_angle_feed_back_[1].d_gain_  = msg.r_leg_hip_r_d_gain ;
//  online_walking->leg_angle_feed_back_[2].p_gain_  = msg.r_leg_hip_p_p_gain ;
//  online_walking->leg_angle_feed_back_[2].d_gain_  = msg.r_leg_hip_p_d_gain ;
//  online_walking->leg_angle_feed_back_[3].p_gain_  = msg.r_leg_kn_p_p_gain  ;
//  online_walking->leg_angle_feed_back_[3].d_gain_  = msg.r_leg_kn_p_d_gain  ;
//  online_walking->leg_angle_feed_back_[4].p_gain_  = msg.r_leg_an_p_p_gain  ;
//  online_walking->leg_angle_feed_back_[4].d_gain_  = msg.r_leg_an_p_d_gain  ;
//  online_walking->leg_angle_feed_back_[5].p_gain_  = msg.r_leg_an_r_p_gain  ;
//  online_walking->leg_angle_feed_back_[5].d_gain_  = msg.r_leg_an_r_d_gain  ;
//
//  online_walking->leg_angle_feed_back_[6].p_gain_  = msg.l_leg_hip_y_p_gain ;
//  online_walking->leg_angle_feed_back_[6].d_gain_  = msg.l_leg_hip_y_d_gain ;
//  online_walking->leg_angle_feed_back_[7].p_gain_  = msg.l_leg_hip_r_p_gain ;
//  online_walking->leg_angle_feed_back_[7].d_gain_  = msg.l_leg_hip_r_d_gain ;
//  online_walking->leg_angle_feed_back_[8].p_gain_  = msg.l_leg_hip_p_p_gain ;
//  online_walking->leg_angle_feed_back_[8].d_gain_  = msg.l_leg_hip_p_d_gain ;
//  online_walking->leg_angle_feed_back_[9].p_gain_  = msg.l_leg_kn_p_p_gain  ;
//  online_walking->leg_angle_feed_back_[9].d_gain_  = msg.l_leg_kn_p_d_gain  ;
//  online_walking->leg_angle_feed_back_[10].p_gain_ = msg.l_leg_an_p_p_gain  ;
//  online_walking->leg_angle_feed_back_[10].d_gain_ = msg.l_leg_an_p_d_gain  ;
//  online_walking->leg_angle_feed_back_[11].p_gain_ = msg.l_leg_an_r_p_gain  ;
//  online_walking->leg_angle_feed_back_[11].d_gain_ = msg.l_leg_an_r_d_gain  ;
//}

//void OnlineWalkingModule::updateJointFeedBackGain()
//{
//  double current_update_gain =  joint_feedback_update_polynomial_coeff_.coeff(0,0) * robotis_framework::powDI(joint_feedback_update_sys_time_, 5)
//  + joint_feedback_update_polynomial_coeff_.coeff(1,0) * robotis_framework::powDI(joint_feedback_update_sys_time_ , 4)
//  + joint_feedback_update_polynomial_coeff_.coeff(2,0) * robotis_framework::powDI(joint_feedback_update_sys_time_ , 3)
//  + joint_feedback_update_polynomial_coeff_.coeff(3,0) * robotis_framework::powDI(joint_feedback_update_sys_time_ , 2)
//  + joint_feedback_update_polynomial_coeff_.coeff(4,0) * robotis_framework::powDI(joint_feedback_update_sys_time_ , 1)
//  + joint_feedback_update_polynomial_coeff_.coeff(5,0) ;
//
//
//  current_joint_feedback_gain_.r_leg_hip_y_p_gain = current_update_gain*(desired_joint_feedback_gain_.r_leg_hip_y_p_gain - previous_joint_feedback_gain_.r_leg_hip_y_p_gain ) + previous_joint_feedback_gain_.r_leg_hip_y_p_gain  ;
//  current_joint_feedback_gain_.r_leg_hip_y_d_gain = current_update_gain*(desired_joint_feedback_gain_.r_leg_hip_y_d_gain - previous_joint_feedback_gain_.r_leg_hip_y_d_gain ) + previous_joint_feedback_gain_.r_leg_hip_y_d_gain  ;
//  current_joint_feedback_gain_.r_leg_hip_r_p_gain = current_update_gain*(desired_joint_feedback_gain_.r_leg_hip_r_p_gain - previous_joint_feedback_gain_.r_leg_hip_r_p_gain ) + previous_joint_feedback_gain_.r_leg_hip_r_p_gain  ;
//  current_joint_feedback_gain_.r_leg_hip_r_d_gain = current_update_gain*(desired_joint_feedback_gain_.r_leg_hip_r_d_gain - previous_joint_feedback_gain_.r_leg_hip_r_d_gain ) + previous_joint_feedback_gain_.r_leg_hip_r_d_gain  ;
//  current_joint_feedback_gain_.r_leg_hip_p_p_gain = current_update_gain*(desired_joint_feedback_gain_.r_leg_hip_p_p_gain - previous_joint_feedback_gain_.r_leg_hip_p_p_gain ) + previous_joint_feedback_gain_.r_leg_hip_p_p_gain  ;
//  current_joint_feedback_gain_.r_leg_hip_p_d_gain = current_update_gain*(desired_joint_feedback_gain_.r_leg_hip_p_d_gain - previous_joint_feedback_gain_.r_leg_hip_p_d_gain ) + previous_joint_feedback_gain_.r_leg_hip_p_d_gain  ;
//  current_joint_feedback_gain_.r_leg_kn_p_p_gain  = current_update_gain*(desired_joint_feedback_gain_.r_leg_kn_p_p_gain  - previous_joint_feedback_gain_.r_leg_kn_p_p_gain  ) + previous_joint_feedback_gain_.r_leg_kn_p_p_gain   ;
//  current_joint_feedback_gain_.r_leg_kn_p_d_gain  = current_update_gain*(desired_joint_feedback_gain_.r_leg_kn_p_d_gain  - previous_joint_feedback_gain_.r_leg_kn_p_d_gain  ) + previous_joint_feedback_gain_.r_leg_kn_p_d_gain   ;
//  current_joint_feedback_gain_.r_leg_an_p_p_gain  = current_update_gain*(desired_joint_feedback_gain_.r_leg_an_p_p_gain  - previous_joint_feedback_gain_.r_leg_an_p_p_gain  ) + previous_joint_feedback_gain_.r_leg_an_p_p_gain   ;
//  current_joint_feedback_gain_.r_leg_an_p_d_gain  = current_update_gain*(desired_joint_feedback_gain_.r_leg_an_p_d_gain  - previous_joint_feedback_gain_.r_leg_an_p_d_gain  ) + previous_joint_feedback_gain_.r_leg_an_p_d_gain   ;
//  current_joint_feedback_gain_.r_leg_an_r_p_gain  = current_update_gain*(desired_joint_feedback_gain_.r_leg_an_r_p_gain  - previous_joint_feedback_gain_.r_leg_an_r_p_gain  ) + previous_joint_feedback_gain_.r_leg_an_r_p_gain   ;
//  current_joint_feedback_gain_.r_leg_an_r_d_gain  = current_update_gain*(desired_joint_feedback_gain_.r_leg_an_r_d_gain  - previous_joint_feedback_gain_.r_leg_an_r_d_gain  ) + previous_joint_feedback_gain_.r_leg_an_r_d_gain   ;
//
//  current_joint_feedback_gain_.l_leg_hip_y_p_gain = current_update_gain*(desired_joint_feedback_gain_.l_leg_hip_y_p_gain - previous_joint_feedback_gain_.l_leg_hip_y_p_gain ) + previous_joint_feedback_gain_.l_leg_hip_y_p_gain  ;
//  current_joint_feedback_gain_.l_leg_hip_y_d_gain = current_update_gain*(desired_joint_feedback_gain_.l_leg_hip_y_d_gain - previous_joint_feedback_gain_.l_leg_hip_y_d_gain ) + previous_joint_feedback_gain_.l_leg_hip_y_d_gain  ;
//  current_joint_feedback_gain_.l_leg_hip_r_p_gain = current_update_gain*(desired_joint_feedback_gain_.l_leg_hip_r_p_gain - previous_joint_feedback_gain_.l_leg_hip_r_p_gain ) + previous_joint_feedback_gain_.l_leg_hip_r_p_gain  ;
//  current_joint_feedback_gain_.l_leg_hip_r_d_gain = current_update_gain*(desired_joint_feedback_gain_.l_leg_hip_r_d_gain - previous_joint_feedback_gain_.l_leg_hip_r_d_gain ) + previous_joint_feedback_gain_.l_leg_hip_r_d_gain  ;
//  current_joint_feedback_gain_.l_leg_hip_p_p_gain = current_update_gain*(desired_joint_feedback_gain_.l_leg_hip_p_p_gain - previous_joint_feedback_gain_.l_leg_hip_p_p_gain ) + previous_joint_feedback_gain_.l_leg_hip_p_p_gain  ;
//  current_joint_feedback_gain_.l_leg_hip_p_d_gain = current_update_gain*(desired_joint_feedback_gain_.l_leg_hip_p_d_gain - previous_joint_feedback_gain_.l_leg_hip_p_d_gain ) + previous_joint_feedback_gain_.l_leg_hip_p_d_gain  ;
//  current_joint_feedback_gain_.l_leg_kn_p_p_gain  = current_update_gain*(desired_joint_feedback_gain_.l_leg_kn_p_p_gain  - previous_joint_feedback_gain_.l_leg_kn_p_p_gain  ) + previous_joint_feedback_gain_.l_leg_kn_p_p_gain   ;
//  current_joint_feedback_gain_.l_leg_kn_p_d_gain  = current_update_gain*(desired_joint_feedback_gain_.l_leg_kn_p_d_gain  - previous_joint_feedback_gain_.l_leg_kn_p_d_gain  ) + previous_joint_feedback_gain_.l_leg_kn_p_d_gain   ;
//  current_joint_feedback_gain_.l_leg_an_p_p_gain  = current_update_gain*(desired_joint_feedback_gain_.l_leg_an_p_p_gain  - previous_joint_feedback_gain_.l_leg_an_p_p_gain  ) + previous_joint_feedback_gain_.l_leg_an_p_p_gain   ;
//  current_joint_feedback_gain_.l_leg_an_p_d_gain  = current_update_gain*(desired_joint_feedback_gain_.l_leg_an_p_d_gain  - previous_joint_feedback_gain_.l_leg_an_p_d_gain  ) + previous_joint_feedback_gain_.l_leg_an_p_d_gain   ;
//  current_joint_feedback_gain_.l_leg_an_r_p_gain  = current_update_gain*(desired_joint_feedback_gain_.l_leg_an_r_p_gain  - previous_joint_feedback_gain_.l_leg_an_r_p_gain  ) + previous_joint_feedback_gain_.l_leg_an_r_p_gain   ;
//  current_joint_feedback_gain_.l_leg_an_r_d_gain  = current_update_gain*(desired_joint_feedback_gain_.l_leg_an_r_d_gain  - previous_joint_feedback_gain_.l_leg_an_r_d_gain  ) + previous_joint_feedback_gain_.l_leg_an_r_d_gain   ;
//
//
//  setJointFeedBackGain(current_joint_feedback_gain_);
//}

bool OnlineWalkingModule::checkBalanceOnOff()
{
  return true;
//  if(gazebo_)
//    return true;
//
//  THORMANG3OnlineWalking *online_walking = THORMANG3OnlineWalking::getInstance();
//
//#ifdef _USE_PD_BALANCE_
//  if ((fabs(online_walking->balance_ctrl_.foot_roll_gyro_ctrl_.p_gain_           ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.foot_roll_gyro_ctrl_.d_gain_           ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.foot_pitch_gyro_ctrl_.p_gain_          ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.foot_pitch_gyro_ctrl_.d_gain_          ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.foot_roll_angle_ctrl_.p_gain_          ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.foot_roll_angle_ctrl_.d_gain_          ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.p_gain_         ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.d_gain_         ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.right_foot_force_x_ctrl_.p_gain_       ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.right_foot_force_y_ctrl_.p_gain_       ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.right_foot_force_z_ctrl_.p_gain_       ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.p_gain_   ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.p_gain_  ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.right_foot_force_x_ctrl_.d_gain_       ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.right_foot_force_y_ctrl_.d_gain_       ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.right_foot_force_z_ctrl_.d_gain_       ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.d_gain_   ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.d_gain_  ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.left_foot_force_x_ctrl_.p_gain_        ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.left_foot_force_y_ctrl_.p_gain_        ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.left_foot_force_z_ctrl_.p_gain_        ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.left_foot_torque_roll_ctrl_.p_gain_    ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.left_foot_torque_pitch_ctrl_.p_gain_   ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.left_foot_force_x_ctrl_.d_gain_        ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.left_foot_force_y_ctrl_.d_gain_        ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.left_foot_force_z_ctrl_.d_gain_        ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.left_foot_torque_roll_ctrl_.d_gain_    ) < 1e-7) &&
//      (fabs(online_walking->balance_ctrl_.left_foot_torque_pitch_ctrl_.d_gain_   ) < 1e-7))
//  {
//    return false;
//  }
//  else
//    return true;
//
//#else
//  if ((fabs(online_walking->balance_ctrl_.getGyroBalanceGainRatio())           < 1e-6) &&
//      (fabs(online_walking->balance_ctrl_.foot_roll_angle_ctrl_.gain_)         < 1e-6) &&
//      (fabs(online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.gain_)        < 1e-6) &&
//      (fabs(online_walking->balance_ctrl_.right_foot_force_x_ctrl_.gain_)      < 1e-6) &&
//      (fabs(online_walking->balance_ctrl_.right_foot_force_y_ctrl_.gain_)      < 1e-6) &&
//      (fabs(online_walking->balance_ctrl_.left_foot_force_x_ctrl_.gain_)       < 1e-6) &&
//      (fabs(online_walking->balance_ctrl_.left_foot_force_y_ctrl_.gain_)       < 1e-6) &&
//      (fabs(online_walking->balance_ctrl_.foot_force_z_diff_ctrl_.gain_)       < 1e-6) &&
//      (fabs(online_walking->balance_ctrl_.right_foot_force_z_ctrl_.gain_)      < 1e-6) &&
//      (fabs(online_walking->balance_ctrl_.left_foot_force_z_ctrl_.gain_)       < 1e-6) &&
//      (fabs(online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.gain_)  < 1e-6) &&
//      (fabs(online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.gain_) < 1e-6) &&
//      (fabs(online_walking->balance_ctrl_.left_foot_torque_roll_ctrl_.gain_)   < 1e-6) &&
//      (fabs(online_walking->balance_ctrl_.left_foot_torque_pitch_ctrl_.gain_)  < 1e-6))
//  {
//    return false;
//  }
//  else
//    return true;
//#endif
}

bool OnlineWalkingModule::removeExistingStepDataServiceCallback(robotis_op3_motion_module_msgs::RemoveExistingStepData::Request  &req,
                                                                robotis_op3_motion_module_msgs::RemoveExistingStepData::Response &res)
{
  Robotisop3OnlineWalking *online_walking = Robotisop3OnlineWalking::getInstance();

  res.result = robotis_op3_motion_module_msgs::RemoveExistingStepData::Response::NO_ERROR;

  if(isRunning())
  {
    res.result |= robotis_op3_motion_module_msgs::RemoveExistingStepData::Response::ROBOT_IS_WALKING_NOW;
  }
  else
  {
    int exist_num_of_step_data = online_walking->getNumofRemainingUnreservedStepData();
    if(exist_num_of_step_data != 0)
      for(int remove_count  = 0; remove_count < exist_num_of_step_data; remove_count++)
        online_walking->eraseLastStepData();
  }
  return true;
}

//void OnlineWalkingModule::imuDataOutputCallback(const sensor_msgs::Imu::ConstPtr &msg)
//{
//  Robotisop3OnlineWalking *online_walking = Robotisop3OnlineWalking::getInstance();
//
//  online_walking->setCurrentIMUSensorOutput(-1.0*(msg->angular_velocity.x), -1.0*(msg->angular_velocity.y),
//                                            msg->orientation.x, msg->orientation.y, msg->orientation.z,
//                                            msg->orientation.w);
//}

void OnlineWalkingModule::onModuleEnable()
{
  std::string status_msg = WalkingStatusMSG::WALKING_MODULE_IS_ENABLED_MSG;
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
}

void OnlineWalkingModule::onModuleDisable()
{
  previous_running_ = present_running = false;

  Robotisop3OnlineWalking *online_walking = Robotisop3OnlineWalking::getInstance();
  std::string status_msg = WalkingStatusMSG::WALKING_MODULE_IS_DISABLED_MSG;
  balance_update_with_loop_ = false;

//  online_walking->leg_angle_feed_back_[0].p_gain_ = 0;
//  online_walking->leg_angle_feed_back_[0].d_gain_ = 0;
//  online_walking->leg_angle_feed_back_[1].p_gain_ = 0;
//  online_walking->leg_angle_feed_back_[1].d_gain_ = 0;
//  online_walking->leg_angle_feed_back_[2].p_gain_ = 0;
//  online_walking->leg_angle_feed_back_[2].d_gain_ = 0;
//  online_walking->leg_angle_feed_back_[3].p_gain_ = 0;
//  online_walking->leg_angle_feed_back_[3].d_gain_ = 0;
//  online_walking->leg_angle_feed_back_[4].p_gain_ = 0;
//  online_walking->leg_angle_feed_back_[4].d_gain_ = 0;
//  online_walking->leg_angle_feed_back_[5].p_gain_ = 0;
//  online_walking->leg_angle_feed_back_[5].d_gain_ = 0;
//
//  online_walking->leg_angle_feed_back_[6].p_gain_ = 0;
//  online_walking->leg_angle_feed_back_[6].d_gain_ = 0;
//  online_walking->leg_angle_feed_back_[7].p_gain_ = 0;
//  online_walking->leg_angle_feed_back_[7].d_gain_ = 0;
//  online_walking->leg_angle_feed_back_[8].p_gain_ = 0;
//  online_walking->leg_angle_feed_back_[8].d_gain_ = 0;
//  online_walking->leg_angle_feed_back_[9].p_gain_ = 0;
//  online_walking->leg_angle_feed_back_[9].d_gain_ = 0;
//  online_walking->leg_angle_feed_back_[10].p_gain_ = 0;
//  online_walking->leg_angle_feed_back_[10].d_gain_ = 0;
//  online_walking->leg_angle_feed_back_[11].p_gain_ = 0;
//  online_walking->leg_angle_feed_back_[11].d_gain_ = 0;
//
//#ifdef _USE_PD_BALANCE_
//   online_walking->balance_ctrl_.foot_roll_gyro_ctrl_.p_gain_           = 0;
//   online_walking->balance_ctrl_.foot_roll_gyro_ctrl_.d_gain_           = 0;
//   online_walking->balance_ctrl_.foot_pitch_gyro_ctrl_.p_gain_          = 0;
//   online_walking->balance_ctrl_.foot_pitch_gyro_ctrl_.d_gain_          = 0;
//   online_walking->balance_ctrl_.foot_roll_angle_ctrl_.p_gain_          = 0;
//   online_walking->balance_ctrl_.foot_roll_angle_ctrl_.d_gain_          = 0;
//   online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.p_gain_         = 0;
//   online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.d_gain_         = 0;
//   online_walking->balance_ctrl_.right_foot_force_x_ctrl_.p_gain_       = 0;
//   online_walking->balance_ctrl_.right_foot_force_y_ctrl_.p_gain_       = 0;
//   online_walking->balance_ctrl_.right_foot_force_z_ctrl_.p_gain_       = 0;
//   online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.p_gain_   = 0;
//   online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.p_gain_  = 0;
//   online_walking->balance_ctrl_.right_foot_force_x_ctrl_.d_gain_       = 0;
//   online_walking->balance_ctrl_.right_foot_force_y_ctrl_.d_gain_       = 0;
//   online_walking->balance_ctrl_.right_foot_force_z_ctrl_.d_gain_       = 0;
//   online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.d_gain_   = 0;
//   online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.d_gain_  = 0;
//   online_walking->balance_ctrl_.left_foot_force_x_ctrl_.p_gain_        = 0;
//   online_walking->balance_ctrl_.left_foot_force_y_ctrl_.p_gain_        = 0;
//   online_walking->balance_ctrl_.left_foot_force_z_ctrl_.p_gain_        = 0;
//   online_walking->balance_ctrl_.left_foot_torque_roll_ctrl_.p_gain_    = 0;
//   online_walking->balance_ctrl_.left_foot_torque_pitch_ctrl_.p_gain_   = 0;
//   online_walking->balance_ctrl_.left_foot_force_x_ctrl_.d_gain_        = 0;
//   online_walking->balance_ctrl_.left_foot_force_y_ctrl_.d_gain_        = 0;
//   online_walking->balance_ctrl_.left_foot_force_z_ctrl_.d_gain_        = 0;
//   online_walking->balance_ctrl_.left_foot_torque_roll_ctrl_.d_gain_    = 0;
//   online_walking->balance_ctrl_.left_foot_torque_pitch_ctrl_.d_gain_   = 0;
//#else
//  online_walking->balance_ctrl_.setGyroBalanceGainRatio(0.0);
//
//  online_walking->balance_ctrl_.foot_roll_angle_ctrl_.gain_  = 0.0;
//  online_walking->balance_ctrl_.foot_pitch_angle_ctrl_.gain_ = 0.0;
//
//  online_walking->balance_ctrl_.right_foot_force_x_ctrl_.gain_ = 0.0;
//  online_walking->balance_ctrl_.right_foot_force_y_ctrl_.gain_ = 0.0;
//  online_walking->balance_ctrl_.left_foot_force_x_ctrl_.gain_  = 0.0;
//  online_walking->balance_ctrl_.left_foot_force_y_ctrl_.gain_  = 0.0;
//
//  online_walking->balance_ctrl_.foot_force_z_diff_ctrl_.gain_ = 0.0;
//  online_walking->balance_ctrl_.right_foot_force_z_ctrl_.gain_ = 0.0;
//  online_walking->balance_ctrl_.left_foot_force_z_ctrl_.gain_ = 0.0;
//
//  online_walking->balance_ctrl_.right_foot_torque_roll_ctrl_.gain_  = 0.0;
//  online_walking->balance_ctrl_.right_foot_torque_pitch_ctrl_.gain_ = 0.0;
//  online_walking->balance_ctrl_.left_foot_torque_roll_ctrl_.gain_   = 0.0;
//  online_walking->balance_ctrl_.left_foot_torque_pitch_ctrl_.gain_  = 0.0;
//#endif
  publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
}

void OnlineWalkingModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors)
{
  if(enable_ == false)
    return;

  Robotisop3OnlineWalking *online_walking = Robotisop3OnlineWalking::getInstance();

//  r_foot_fx_N_  = sensors["r_foot_fx_scaled_N"];
//  r_foot_fy_N_  = sensors["r_foot_fy_scaled_N"];
//  r_foot_fz_N_  = sensors["r_foot_fz_scaled_N"];
//  r_foot_Tx_Nm_ = sensors["r_foot_tx_scaled_Nm"];
//  r_foot_Ty_Nm_ = sensors["r_foot_ty_scaled_Nm"];
//  r_foot_Tz_Nm_ = sensors["r_foot_tz_scaled_Nm"];
//
//  l_foot_fx_N_  = sensors["l_foot_fx_scaled_N"];
//  l_foot_fy_N_  = sensors["l_foot_fy_scaled_N"];
//  l_foot_fz_N_  = sensors["l_foot_fz_scaled_N"];
//  l_foot_Tx_Nm_ = sensors["l_foot_tx_scaled_Nm"];
//  l_foot_Ty_Nm_ = sensors["l_foot_ty_scaled_Nm"];
//  l_foot_Tz_Nm_ = sensors["l_foot_tz_scaled_Nm"];


//  r_foot_fx_N_ = robotis_framework::sign(r_foot_fx_N_) * fmin( fabs(r_foot_fx_N_), 2000.0);
//  r_foot_fy_N_ = robotis_framework::sign(r_foot_fy_N_) * fmin( fabs(r_foot_fy_N_), 2000.0);
//  r_foot_fz_N_ = robotis_framework::sign(r_foot_fz_N_) * fmin( fabs(r_foot_fz_N_), 2000.0);
//  r_foot_Tx_Nm_ = robotis_framework::sign(r_foot_Tx_Nm_) *fmin(fabs(r_foot_Tx_Nm_), 300.0);
//  r_foot_Ty_Nm_ = robotis_framework::sign(r_foot_Ty_Nm_) *fmin(fabs(r_foot_Ty_Nm_), 300.0);
//  r_foot_Tz_Nm_ = robotis_framework::sign(r_foot_Tz_Nm_) *fmin(fabs(r_foot_Tz_Nm_), 300.0);
//
//  l_foot_fx_N_ = robotis_framework::sign(l_foot_fx_N_) * fmin( fabs(l_foot_fx_N_), 2000.0);
//  l_foot_fy_N_ = robotis_framework::sign(l_foot_fy_N_) * fmin( fabs(l_foot_fy_N_), 2000.0);
//  l_foot_fz_N_ = robotis_framework::sign(l_foot_fz_N_) * fmin( fabs(l_foot_fz_N_), 2000.0);
//  l_foot_Tx_Nm_ = robotis_framework::sign(l_foot_Tx_Nm_) *fmin(fabs(l_foot_Tx_Nm_), 300.0);
//  l_foot_Ty_Nm_ = robotis_framework::sign(l_foot_Ty_Nm_) *fmin(fabs(l_foot_Ty_Nm_), 300.0);
//  l_foot_Tz_Nm_ = robotis_framework::sign(l_foot_Tz_Nm_) *fmin(fabs(l_foot_Tz_Nm_), 300.0);


//  if(balance_update_with_loop_ == true)
//  {
//    balance_update_sys_time_ += control_cycle_msec_ * 0.001;
//    if(balance_update_sys_time_ >= balance_update_duration_ )
//    {
//      balance_update_sys_time_ = balance_update_duration_;
//      balance_update_with_loop_ = false;
//      setBalanceParam(desired_balance_param_);
//      std::string status_msg = WalkingStatusMSG::BALANCE_PARAM_SETTING_FINISH_MSG;
//      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
//      std_msgs::String done_msg;
//      done_msg.data = "balance_updated";
//      movement_done_pub_.publish(done_msg);
//    }
//    else
//    {
//      updateBalanceParam();
//    }
//  }
//
//  if(joint_feedback_update_with_loop_ == true)
//  {
//    joint_feedback_update_sys_time_ += control_cycle_msec_ * 0.001;
//    if(joint_feedback_update_sys_time_ >= joint_feedback_update_duration_ )
//    {
//      joint_feedback_update_sys_time_ = joint_feedback_update_duration_;
//      joint_feedback_update_with_loop_ = false;
//      setJointFeedBackGain(desired_joint_feedback_gain_);
//
//      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Joint FeedBack Gain Update Finished");
//      std_msgs::String done_msg;
//      done_msg.data = "joint_feedback_gain_updated";
//      movement_done_pub_.publish(done_msg);
//    }
//    else
//    {
//      updateJointFeedBackGain();
//    }
//  }

  online_walking->current_right_fx_N_  = r_foot_fx_N_;
  online_walking->current_right_fy_N_  = r_foot_fy_N_;
  online_walking->current_right_fz_N_  = r_foot_fz_N_;
  online_walking->current_right_tx_Nm_ = r_foot_Tx_Nm_;
  online_walking->current_right_ty_Nm_ = r_foot_Ty_Nm_;
  online_walking->current_right_tz_Nm_ = r_foot_Tz_Nm_;

  online_walking->current_left_fx_N_  = l_foot_fx_N_;
  online_walking->current_left_fy_N_  = l_foot_fy_N_;
  online_walking->current_left_fz_N_  = l_foot_fz_N_;
  online_walking->current_left_tx_Nm_ = l_foot_Tx_Nm_;
  online_walking->current_left_ty_Nm_ = l_foot_Ty_Nm_;
  online_walking->current_left_tz_Nm_ = l_foot_Tz_Nm_;

//  online_walking->curr_angle_rad_[0]  = result_["r_leg_hip_y"]->goal_position_;
//  online_walking->curr_angle_rad_[1]  = result_["r_leg_hip_r"]->goal_position_;
//  online_walking->curr_angle_rad_[2]  = result_["r_leg_hip_p"]->goal_position_;
//  online_walking->curr_angle_rad_[3]  = result_["r_leg_kn_p" ]->goal_position_;
//  online_walking->curr_angle_rad_[4]  = result_["r_leg_an_p" ]->goal_position_;
//  online_walking->curr_angle_rad_[5]  = result_["r_leg_an_r" ]->goal_position_;
//
//  online_walking->curr_angle_rad_[6]  = result_["l_leg_hip_y"]->goal_position_;
//  online_walking->curr_angle_rad_[7]  = result_["l_leg_hip_r"]->goal_position_;
//  online_walking->curr_angle_rad_[8]  = result_["l_leg_hip_p"]->goal_position_;
//  online_walking->curr_angle_rad_[9]  = result_["l_leg_kn_p" ]->goal_position_;
//  online_walking->curr_angle_rad_[10] = result_["l_leg_an_p" ]->goal_position_;
//  online_walking->curr_angle_rad_[11] = result_["l_leg_an_r" ]->goal_position_;

//  for(std::map<std::string, robotis_framework::DynamixelState*>::iterator result_it = result_.begin();
//      result_it != result_.end();
//      result_it++)
//  {
//    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxls_it = dxls.find(result_it->first);
//    if(dxls_it != dxls.end())
//      online_walking->curr_angle_rad_[joint_name_to_index_[result_it->first]] = dxls_it->second->dxl_state_->present_position_;
//  }

  process_mutex_.lock();
  online_walking->process();
  process_mutex_.unlock();

  desired_matrix_g_to_pelvis_ = online_walking->mat_g_to_pelvis_;
  desired_matrix_g_to_rfoot_  = online_walking->mat_g_to_rfoot_;
  desired_matrix_g_to_lfoot_  = online_walking->mat_g_to_lfoot_;

  r_foot_target_ft_.coeffRef(0,0) = online_walking->r_target_fx_N_;
  r_foot_target_ft_.coeffRef(1,0) = online_walking->r_target_fy_N_;
  r_foot_target_ft_.coeffRef(2,0) = online_walking->r_target_fz_N_;
  l_foot_target_ft_.coeffRef(0,0) = online_walking->l_target_fx_N_;
  l_foot_target_ft_.coeffRef(1,0) = online_walking->l_target_fy_N_;
  l_foot_target_ft_.coeffRef(2,0) = online_walking->l_target_fz_N_;

  publishRobotPose();

  result_["r_hip_yaw"  ]->goal_position_ = online_walking->out_angle_rad_[0];
  result_["r_hip_roll" ]->goal_position_ = online_walking->out_angle_rad_[1];
  result_["r_hip_pitch"]->goal_position_ = online_walking->out_angle_rad_[2];
  result_["r_knee"     ]->goal_position_ = online_walking->out_angle_rad_[3];
  result_["r_ank_pitch"]->goal_position_ = online_walking->out_angle_rad_[4];
  result_["r_ank_roll" ]->goal_position_ = online_walking->out_angle_rad_[5];

  result_["l_hip_yaw"  ]->goal_position_ = online_walking->out_angle_rad_[6];
  result_["l_hip_roll" ]->goal_position_ = online_walking->out_angle_rad_[7];
  result_["l_hip_pitch"]->goal_position_ = online_walking->out_angle_rad_[8];
  result_["l_knee"     ]->goal_position_ = online_walking->out_angle_rad_[9];
  result_["l_ank_pitch"]->goal_position_ = online_walking->out_angle_rad_[10];
  result_["l_ank_roll" ]->goal_position_ = online_walking->out_angle_rad_[11];

  for(std::map<std::string, robotis_framework::DynamixelState*>::iterator it = result_.begin();
      it != result_.end();
      it++)
  {
    std::map<std::string, JointPose>::iterator pose_it = joint_name_to_pose_.find(it->first);

    double prev_pos = pose_it->second.position_;
    double prev_vel = pose_it->second.velocity_;

    pose_it->second.position_     = it->second->goal_position_;
    pose_it->second.velocity_     = (it->second->goal_position_ - prev_pos)/(control_cycle_msec_ * 0.001);
    pose_it->second.acceleration_ = (pose_it->second.velocity_  - prev_vel)/(control_cycle_msec_ * 0.001);
  }

//#ifdef _TUNE_JOUINT_FEEDBACK_GAIN_
//  walking_joint_states_msg_.header.stamp = ros::Time::now();
//  walking_joint_states_msg_.r_goal_hip_y = online_walking->r_leg_out_angle_rad_[0];
//  walking_joint_states_msg_.r_goal_hip_r = online_walking->r_leg_out_angle_rad_[1];
//  walking_joint_states_msg_.r_goal_hip_p = online_walking->r_leg_out_angle_rad_[2];
//  walking_joint_states_msg_.r_goal_kn_p  = online_walking->r_leg_out_angle_rad_[3];
//  walking_joint_states_msg_.r_goal_an_p  = online_walking->r_leg_out_angle_rad_[4];
//  walking_joint_states_msg_.r_goal_an_r  = online_walking->r_leg_out_angle_rad_[5];
//  walking_joint_states_msg_.l_goal_hip_y = online_walking->l_leg_out_angle_rad_[0];
//  walking_joint_states_msg_.l_goal_hip_r = online_walking->l_leg_out_angle_rad_[1];
//  walking_joint_states_msg_.l_goal_hip_p = online_walking->l_leg_out_angle_rad_[2];
//  walking_joint_states_msg_.l_goal_kn_p  = online_walking->l_leg_out_angle_rad_[3];
//  walking_joint_states_msg_.l_goal_an_p  = online_walking->l_leg_out_angle_rad_[4];
//  walking_joint_states_msg_.l_goal_an_r  = online_walking->l_leg_out_angle_rad_[5];
//
//  walking_joint_states_msg_.r_present_hip_y = online_walking->curr_angle_rad_[0];
//  walking_joint_states_msg_.r_present_hip_r = online_walking->curr_angle_rad_[1];
//  walking_joint_states_msg_.r_present_hip_p = online_walking->curr_angle_rad_[2];
//  walking_joint_states_msg_.r_present_kn_p  = online_walking->curr_angle_rad_[3];
//  walking_joint_states_msg_.r_present_an_p  = online_walking->curr_angle_rad_[4];
//  walking_joint_states_msg_.r_present_an_r  = online_walking->curr_angle_rad_[5];
//  walking_joint_states_msg_.l_present_hip_y = online_walking->curr_angle_rad_[6];
//  walking_joint_states_msg_.l_present_hip_r = online_walking->curr_angle_rad_[7];
//  walking_joint_states_msg_.l_present_hip_p = online_walking->curr_angle_rad_[8];
//  walking_joint_states_msg_.l_present_kn_p  = online_walking->curr_angle_rad_[9];
//  walking_joint_states_msg_.l_present_an_p  = online_walking->curr_angle_rad_[10];
//  walking_joint_states_msg_.l_present_an_r  = online_walking->curr_angle_rad_[11];
//  walking_joint_states_pub_.publish(walking_joint_states_msg_);
//#endif

  present_running = isRunning();
  if(previous_running_ != present_running)
  {
    if(present_running == true)
    {
      std::string status_msg = WalkingStatusMSG::WALKING_START_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
    }
    else
    {
      std::string status_msg = WalkingStatusMSG::WALKING_FINISH_MSG;
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
    }
  }
  previous_running_ = present_running;
}

void OnlineWalkingModule::stop()
{
  return;
}

