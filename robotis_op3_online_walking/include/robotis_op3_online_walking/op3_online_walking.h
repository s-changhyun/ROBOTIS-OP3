
#ifndef ROBOTIS_OP3_MOTION_MODULE_ONLINE_WALKING_OP3_ONLINE_WALKING_H_
#define ROBOTIS_OP3_MOTION_MODULE_ONLINE_WALKING_OP3_ONLINE_WALKING_H_

#include <vector>
#include <boost/thread.hpp>

#include "robotis_framework_common/singleton.h"
#include "robotis_op3_kinematics_dynamics/kinematics_dynamics.h"
//#include "thormang3_balance_control/thormang3_balance_control.h"
#include "robotis_math/robotis_math.h"
//#include "scilab_optimization/scilab_optimization.h"

namespace robotis_op3
{

class Robotisop3OnlineWalking : public robotis_framework::Singleton<Robotisop3OnlineWalking>
{
public:
  enum
  {
    BalancingPhase0 = 0, // DSP : START
    BalancingPhase1 = 1, // DSP : R--O->L
    BalancingPhase2 = 2, // SSP : L_BALANCING1
    BalancingPhase3 = 3, // SSP : L_BALANCING2
    BalancingPhase4 = 4, // DSP : R--O<-L
    BalancingPhase5 = 5, // DSP : R<-O--L
    BalancingPhase6 = 6, // SSP : R_BALANCING1
    BalancingPhase7 = 7, // SSP : R_BALANCING2
    BalancingPhase8 = 8, // DSP : R->O--L
    BalancingPhase9 = 9  // DSP : END
  };

  Robotisop3OnlineWalking();
  virtual ~Robotisop3OnlineWalking();

  void initialize(int control_time_unit_msec);
  void reInitialize();
  void start();
  void stop();
  void process();
  bool isRunning();

  bool addStepData(robotis_framework::StepData step_data);
  void eraseLastStepData();
  int  getNumofRemainingUnreservedStepData();
  void getReferenceStepDatafotAddition(robotis_framework::StepData *ref_step_data_for_addition);

  void setPreviewRefHeight(double height);
  bool setInitialPose(double r_foot_x, double r_foot_y, double r_foot_z, double r_foot_roll, double r_foot_pitch, double r_foot_yaw,
      double l_foot_x, double l_foot_y, double l_foot_z, double l_foot_roll, double l_foot_pitch, double l_foot_yaw,
      double center_of_body_x, double center_of_body_y, double center_of_body_z,
      double center_of_body_roll, double center_of_body_pitch, double center_of_body_yaw);

  void setInitialRightShoulderAngle(double shoulder_angle_rad);
  void setInitialLeftShoulderAngle(double shoulder_angle_rad);
  void setInitialRightElbowAngle(double elbow_angle_rad);
  void setInitialLeftElbowAngle(double elbow_angle_rad);

  Eigen::MatrixXd mat_pelvis_to_g_,  mat_g_to_pelvis_;
  Eigen::MatrixXd mat_robot_to_pelvis_, mat_pelvis_to_robot_;
  Eigen::MatrixXd mat_robot_to_g_, mat_g_to_robot_;
  Eigen::MatrixXd mat_pelvis_to_rhip_, mat_rhip_to_pelvis_;
  Eigen::MatrixXd mat_pelvis_to_lhip_, mat_lhip_to_pelvis_;

  Eigen::MatrixXd mat_g_to_rfoot_, mat_g_to_lfoot_;

  double r_shoulder_out_angle_rad_;
  double l_shoulder_out_angle_rad_;
  double r_elbow_out_angle_rad_;
  double l_elbow_out_angle_rad_;
  double r_leg_out_angle_rad_[6];
  double l_leg_out_angle_rad_[6];
  double out_angle_rad_[16];

  double hip_roll_feedforward_angle_rad_;

  // balance control
  //int balance_error_;
  //RobotisBalanceControl balance_ctrl_;

  double r_target_fx_N_;
  double l_target_fx_N_;
  double r_target_fy_N_;
  double l_target_fy_N_;
  double r_target_fz_N_;
  double l_target_fz_N_;

  // sensor value
  double current_right_fx_N_,  current_right_fy_N_,  current_right_fz_N_;
  double current_right_tx_Nm_, current_right_ty_Nm_, current_right_tz_Nm_;
  double current_left_fx_N_,  current_left_fy_N_,  current_left_fz_N_;
  double current_left_tx_Nm_, current_left_ty_Nm_, current_left_tz_Nm_;

  Eigen::Quaterniond quat_current_imu_;
  Eigen::MatrixXd mat_current_imu_;
  double current_imu_roll_rad_, current_imu_pitch_rad_;
  double current_gyro_roll_rad_per_sec_, current_gyro_pitch_rad_per_sec_;

private:
  void initializePreviewMatrices();
  void calcStepIdxData();
  void calcRefZMP();
  void calcDesiredPose();

  double wsin(double time, double period, double period_shift, double mag, double mag_shift);
  double wsigmoid(double time, double period, double time_shift, double mag, double mag_shift, double sigmoid_ratio, double distortion_ratio);


  int control_time_unit_msec_;
  KinematicsDynamics* robotis_op3_kd_;

  double left_fz_trajectory_start_time_;
  double left_fz_trajectory_end_time_;
  double left_fz_trajectory_target_;
  double left_fz_trajectory_shift_;

  double total_mass_of_robot_;
  double right_dsp_fz_N_, right_ssp_fz_N_;
  double left_dsp_fz_N_,  left_ssp_fz_N_;

  Eigen::MatrixXd mat_robot_to_pelvis_modified_, mat_cob_to_pelvis_modified_;
  Eigen::MatrixXd mat_robot_to_rf_modified_;
  Eigen::MatrixXd mat_robot_to_lf_modified_;
  Eigen::MatrixXd mat_robot_to_rfoot_;
  Eigen::MatrixXd mat_robot_to_lfoot_;


  std::vector<robotis_framework::StepData> added_step_data_;

  double goal_waist_yaw_angle_rad_;
  robotis_framework::StepData current_step_data_;
  robotis_framework::StepData reference_step_data_for_addition_;
  robotis_framework::Pose3D initial_right_foot_pose_, initial_left_foot_pose_, initial_body_pose_;
  robotis_framework::Pose3D present_right_foot_pose_, present_left_foot_pose_, present_body_pose_;
  robotis_framework::Pose3D previous_step_right_foot_pose_, previous_step_left_foot_pose_, previous_step_body_pose_;
  robotis_framework::Pose3D rhip_to_rfoot_pose_, lhip_to_lfoot_pose_;
  robotis_framework::FifthOrderPolynomialTrajectory foot_x_tra_,    foot_y_tra_,     foot_z_tra_;
  robotis_framework::FifthOrderPolynomialTrajectory foot_roll_tra_, foot_pitch_tra_, foot_yaw_tra_;
  robotis_framework::FifthOrderPolynomialTrajectory foot_z_swap_tra_;
  robotis_framework::FifthOrderPolynomialTrajectory body_z_tra_, body_roll_tra_, body_pitch_tra_, body_yaw_tra_;
  robotis_framework::FifthOrderPolynomialTrajectory waist_yaw_tra_;
  robotis_framework::FifthOrderPolynomialTrajectory body_z_swap_tra_;
  robotis_framework::FifthOrderPolynomialTrajectory hip_roll_swap_tra_;
  robotis_framework::FifthOrderPolynomialTrajectory l_target_fz_tra_;

  double present_waist_yaw_angle_rad_;
  double previous_step_waist_yaw_angle_rad_;
  double r_init_shoulder_angle_rad_, r_init_elbow_angle_rad_;
  double l_init_shoulder_angle_rad_, l_init_elbow_angle_rad_;
  double r_shoulder_dir_, r_elbow_dir_;
  double l_shoulder_dir_, l_elbow_dir_;
  double shouler_swing_gain_, elbow_swing_gain_;

  Eigen::MatrixXd mat_rfoot_to_rft_, mat_lfoot_to_lft_;
  Eigen::MatrixXd rot_x_pi_3d_, rot_z_pi_3d_;

  Eigen::VectorXi step_idx_data_;
  boost::mutex mutex_lock_;

  //Time for Preview Control and Dynamics Regulator
  double preview_time_;
  int preview_size_;

  //These matrix and parameters are for preview control
  double lipm_height_;
  double k_s_;
  double sum_of_zmp_x_ ;
  double sum_of_zmp_y_ ;
  double sum_of_cx_ ;
  double sum_of_cy_ ;
  Eigen::MatrixXd A_, b_, c_;
  Eigen::MatrixXd k_x_;
  Eigen::MatrixXd f_;
  Eigen::MatrixXd u_x, u_y;
  Eigen::MatrixXd x_lipm_, y_lipm_;

  int current_start_idx_for_ref_zmp_;

  double ref_zmp_x_at_this_time_, ref_zmp_y_at_this_time_;
  Eigen::MatrixXd reference_zmp_x_, reference_zmp_y_;

  bool real_running, ctrl_running;

  double walking_time_;    //Absolute Time
  double reference_time_;  //Absolute Time
  int balancing_index_;
  int current_step_data_status_;
};

}




#endif /* ROBOTIS_OP3_MOTION_MODULE_ONLINE_WALKING_OP3_ONLINE_WALKING_H_ */
