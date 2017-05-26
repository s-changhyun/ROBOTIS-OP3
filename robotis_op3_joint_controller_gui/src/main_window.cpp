/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/robotis_op3_joint_controller_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robotis_op3_joint_controller_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode_(argc,argv)
{
  ui_.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui_.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  setWindowIcon(QIcon(":/images/icon.png"));
  ui_.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode_, SIGNAL(rosShutdown()), this, SLOT(close()));

  /*********************
  ** Logging
  **********************/
  ui_.view_logging->setModel(qnode_.loggingModel());
  QObject::connect(&qnode_, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  joint_name_.push_back("r_hip_yaw");
  joint_name_.push_back("l_hip_yaw");
  joint_name_.push_back("r_hip_roll");
  joint_name_.push_back("l_hip_roll");
  joint_name_.push_back("r_hip_pitch");
  joint_name_.push_back("l_hip_pitch");
  joint_name_.push_back("r_knee");
  joint_name_.push_back("l_knee");
  joint_name_.push_back("r_ank_pitch");
  joint_name_.push_back("l_ank_pitch");
  joint_name_.push_back("r_ank_roll");
  joint_name_.push_back("l_ank_roll");

  ff_torque_spinbox_.push_back(ui_.id_7_ff_torque_spinbox);
  ff_torque_spinbox_.push_back(ui_.id_8_ff_torque_spinbox);
  ff_torque_spinbox_.push_back(ui_.id_9_ff_torque_spinbox);
  ff_torque_spinbox_.push_back(ui_.id_10_ff_torque_spinbox);
  ff_torque_spinbox_.push_back(ui_.id_11_ff_torque_spinbox);
  ff_torque_spinbox_.push_back(ui_.id_12_ff_torque_spinbox);
  ff_torque_spinbox_.push_back(ui_.id_13_ff_torque_spinbox);
  ff_torque_spinbox_.push_back(ui_.id_14_ff_torque_spinbox);
  ff_torque_spinbox_.push_back(ui_.id_15_ff_torque_spinbox);
  ff_torque_spinbox_.push_back(ui_.id_16_ff_torque_spinbox);
  ff_torque_spinbox_.push_back(ui_.id_17_ff_torque_spinbox);
  ff_torque_spinbox_.push_back(ui_.id_18_ff_torque_spinbox);

  fb_p_torque_spinbox_.push_back(ui_.id_7_fb_p_torque_spinbox);
  fb_p_torque_spinbox_.push_back(ui_.id_8_fb_p_torque_spinbox);
  fb_p_torque_spinbox_.push_back(ui_.id_9_fb_p_torque_spinbox);
  fb_p_torque_spinbox_.push_back(ui_.id_10_fb_p_torque_spinbox);
  fb_p_torque_spinbox_.push_back(ui_.id_11_fb_p_torque_spinbox);
  fb_p_torque_spinbox_.push_back(ui_.id_12_fb_p_torque_spinbox);
  fb_p_torque_spinbox_.push_back(ui_.id_13_fb_p_torque_spinbox);
  fb_p_torque_spinbox_.push_back(ui_.id_14_fb_p_torque_spinbox);
  fb_p_torque_spinbox_.push_back(ui_.id_15_fb_p_torque_spinbox);
  fb_p_torque_spinbox_.push_back(ui_.id_16_fb_p_torque_spinbox);
  fb_p_torque_spinbox_.push_back(ui_.id_17_fb_p_torque_spinbox);
  fb_p_torque_spinbox_.push_back(ui_.id_18_fb_p_torque_spinbox);

  ff_joint_vel_spinbox_.push_back(ui_.id_7_ff_joint_vel_spinbox);
  ff_joint_vel_spinbox_.push_back(ui_.id_8_ff_joint_vel_spinbox);
  ff_joint_vel_spinbox_.push_back(ui_.id_9_ff_joint_vel_spinbox);
  ff_joint_vel_spinbox_.push_back(ui_.id_10_ff_joint_vel_spinbox);
  ff_joint_vel_spinbox_.push_back(ui_.id_11_ff_joint_vel_spinbox);
  ff_joint_vel_spinbox_.push_back(ui_.id_12_ff_joint_vel_spinbox);
  ff_joint_vel_spinbox_.push_back(ui_.id_13_ff_joint_vel_spinbox);
  ff_joint_vel_spinbox_.push_back(ui_.id_14_ff_joint_vel_spinbox);
  ff_joint_vel_spinbox_.push_back(ui_.id_15_ff_joint_vel_spinbox);
  ff_joint_vel_spinbox_.push_back(ui_.id_16_ff_joint_vel_spinbox);
  ff_joint_vel_spinbox_.push_back(ui_.id_17_ff_joint_vel_spinbox);
  ff_joint_vel_spinbox_.push_back(ui_.id_18_ff_joint_vel_spinbox);

  fb_p_joint_pos_spinbox_.push_back(ui_.id_7_fb_p_joint_pos_spinbox);
  fb_p_joint_pos_spinbox_.push_back(ui_.id_8_fb_p_joint_pos_spinbox);
  fb_p_joint_pos_spinbox_.push_back(ui_.id_9_fb_p_joint_pos_spinbox);
  fb_p_joint_pos_spinbox_.push_back(ui_.id_10_fb_p_joint_pos_spinbox);
  fb_p_joint_pos_spinbox_.push_back(ui_.id_11_fb_p_joint_pos_spinbox);
  fb_p_joint_pos_spinbox_.push_back(ui_.id_12_fb_p_joint_pos_spinbox);
  fb_p_joint_pos_spinbox_.push_back(ui_.id_13_fb_p_joint_pos_spinbox);
  fb_p_joint_pos_spinbox_.push_back(ui_.id_14_fb_p_joint_pos_spinbox);
  fb_p_joint_pos_spinbox_.push_back(ui_.id_15_fb_p_joint_pos_spinbox);
  fb_p_joint_pos_spinbox_.push_back(ui_.id_16_fb_p_joint_pos_spinbox);
  fb_p_joint_pos_spinbox_.push_back(ui_.id_17_fb_p_joint_pos_spinbox);
  fb_p_joint_pos_spinbox_.push_back(ui_.id_18_fb_p_joint_pos_spinbox);

  fb_d_joint_pos_spinbox_.push_back(ui_.id_7_fb_d_joint_pos_spinbox);
  fb_d_joint_pos_spinbox_.push_back(ui_.id_8_fb_d_joint_pos_spinbox);
  fb_d_joint_pos_spinbox_.push_back(ui_.id_9_fb_d_joint_pos_spinbox);
  fb_d_joint_pos_spinbox_.push_back(ui_.id_10_fb_d_joint_pos_spinbox);
  fb_d_joint_pos_spinbox_.push_back(ui_.id_11_fb_d_joint_pos_spinbox);
  fb_d_joint_pos_spinbox_.push_back(ui_.id_12_fb_d_joint_pos_spinbox);
  fb_d_joint_pos_spinbox_.push_back(ui_.id_13_fb_d_joint_pos_spinbox);
  fb_d_joint_pos_spinbox_.push_back(ui_.id_14_fb_d_joint_pos_spinbox);
  fb_d_joint_pos_spinbox_.push_back(ui_.id_15_fb_d_joint_pos_spinbox);
  fb_d_joint_pos_spinbox_.push_back(ui_.id_16_fb_d_joint_pos_spinbox);
  fb_d_joint_pos_spinbox_.push_back(ui_.id_17_fb_d_joint_pos_spinbox);
  fb_d_joint_pos_spinbox_.push_back(ui_.id_18_fb_d_joint_pos_spinbox);

  fb_i_joint_pos_spinbox_.push_back(ui_.id_7_fb_i_joint_pos_spinbox);
  fb_i_joint_pos_spinbox_.push_back(ui_.id_8_fb_i_joint_pos_spinbox);
  fb_i_joint_pos_spinbox_.push_back(ui_.id_9_fb_i_joint_pos_spinbox);
  fb_i_joint_pos_spinbox_.push_back(ui_.id_10_fb_i_joint_pos_spinbox);
  fb_i_joint_pos_spinbox_.push_back(ui_.id_11_fb_i_joint_pos_spinbox);
  fb_i_joint_pos_spinbox_.push_back(ui_.id_12_fb_i_joint_pos_spinbox);
  fb_i_joint_pos_spinbox_.push_back(ui_.id_13_fb_i_joint_pos_spinbox);
  fb_i_joint_pos_spinbox_.push_back(ui_.id_14_fb_i_joint_pos_spinbox);
  fb_i_joint_pos_spinbox_.push_back(ui_.id_15_fb_i_joint_pos_spinbox);
  fb_i_joint_pos_spinbox_.push_back(ui_.id_16_fb_i_joint_pos_spinbox);
  fb_i_joint_pos_spinbox_.push_back(ui_.id_17_fb_i_joint_pos_spinbox);
  fb_i_joint_pos_spinbox_.push_back(ui_.id_18_fb_i_joint_pos_spinbox);

  fs_friction_spinbox_.push_back(ui_.id_1_fs_gain_spinbox);
  fs_friction_spinbox_.push_back(ui_.id_2_fs_gain_spinbox);
  fs_friction_spinbox_.push_back(ui_.id_3_fs_gain_spinbox);
  fs_friction_spinbox_.push_back(ui_.id_4_fs_gain_spinbox);
  fs_friction_spinbox_.push_back(ui_.id_5_fs_gain_spinbox);
  fs_friction_spinbox_.push_back(ui_.id_6_fs_gain_spinbox);
  fs_friction_spinbox_.push_back(ui_.id_7_fs_gain_spinbox);
  fs_friction_spinbox_.push_back(ui_.id_8_fs_gain_spinbox);
  fs_friction_spinbox_.push_back(ui_.id_9_fs_gain_spinbox);
  fs_friction_spinbox_.push_back(ui_.id_10_fs_gain_spinbox);
  fs_friction_spinbox_.push_back(ui_.id_11_fs_gain_spinbox);
  fs_friction_spinbox_.push_back(ui_.id_12_fs_gain_spinbox);

  fd_friction_spinbox_.push_back(ui_.id_1_fd_gain_spinbox);
  fd_friction_spinbox_.push_back(ui_.id_2_fd_gain_spinbox);
  fd_friction_spinbox_.push_back(ui_.id_3_fd_gain_spinbox);
  fd_friction_spinbox_.push_back(ui_.id_4_fd_gain_spinbox);
  fd_friction_spinbox_.push_back(ui_.id_5_fd_gain_spinbox);
  fd_friction_spinbox_.push_back(ui_.id_6_fd_gain_spinbox);
  fd_friction_spinbox_.push_back(ui_.id_7_fd_gain_spinbox);
  fd_friction_spinbox_.push_back(ui_.id_8_fd_gain_spinbox);
  fd_friction_spinbox_.push_back(ui_.id_9_fd_gain_spinbox);
  fd_friction_spinbox_.push_back(ui_.id_10_fd_gain_spinbox);
  fd_friction_spinbox_.push_back(ui_.id_11_fd_gain_spinbox);
  fd_friction_spinbox_.push_back(ui_.id_12_fd_gain_spinbox);

  fv_friction_spinbox_.push_back(ui_.id_1_fv_gain_spinbox);
  fv_friction_spinbox_.push_back(ui_.id_2_fv_gain_spinbox);
  fv_friction_spinbox_.push_back(ui_.id_3_fv_gain_spinbox);
  fv_friction_spinbox_.push_back(ui_.id_4_fv_gain_spinbox);
  fv_friction_spinbox_.push_back(ui_.id_5_fv_gain_spinbox);
  fv_friction_spinbox_.push_back(ui_.id_6_fv_gain_spinbox);
  fv_friction_spinbox_.push_back(ui_.id_7_fv_gain_spinbox);
  fv_friction_spinbox_.push_back(ui_.id_8_fv_gain_spinbox);
  fv_friction_spinbox_.push_back(ui_.id_9_fv_gain_spinbox);
  fv_friction_spinbox_.push_back(ui_.id_10_fv_gain_spinbox);
  fv_friction_spinbox_.push_back(ui_.id_11_fv_gain_spinbox);
  fv_friction_spinbox_.push_back(ui_.id_12_fv_gain_spinbox);

  /****************************
  ** Connect
  ****************************/
  qRegisterMetaType<robotis_op3_motion_module_msgs::JointControllerGain>("robotis_op3_motion_module_msgs::JointControlGain");
  QObject::connect(&qnode_,
                   SIGNAL(updateJointControllerGain(robotis_op3_motion_module_msgs::JointControllerGain)),
                   this,
                   SLOT(updateJointControllerGainSpinbox(robotis_op3_motion_module_msgs::JointControllerGain)));

  qRegisterMetaType<robotis_op3_motion_module_msgs::FrictionGain>("robotis_op3_motion_module_msgs::FrictionGain");
  QObject::connect(&qnode_,
                   SIGNAL(updateFrictionGain(robotis_op3_motion_module_msgs::FrictionGain)),
                   this,
                   SLOT(updateFrictionGainSpinbox(robotis_op3_motion_module_msgs::FrictionGain)));

  /*********************
  ** Auto Start
  **********************/
  qnode_.init();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::updateJointControllerGainSpinbox(robotis_op3_motion_module_msgs::JointControllerGain msg)
{
  for (int name_index=0; name_index<msg.joint_name.size(); name_index++)
  {
    for (int joint_index=0; joint_index<joint_name_.size(); joint_index++)
    {
      if (msg.joint_name[joint_index] == joint_name_[name_index])
      {
        ((QDoubleSpinBox *) ff_torque_spinbox_[name_index])->setValue(msg.ff_torque_gain[joint_index]);
        ((QDoubleSpinBox *) fb_p_torque_spinbox_[name_index])->setValue(msg.fb_p_torque_gain[joint_index]);
        ((QDoubleSpinBox *) ff_joint_vel_spinbox_[name_index])->setValue(msg.ff_joint_vel_gain[joint_index]);
        ((QDoubleSpinBox *) fb_p_joint_pos_spinbox_[name_index])->setValue(msg.fb_p_joint_pos_gain[joint_index]);
        ((QDoubleSpinBox *) fb_d_joint_pos_spinbox_[name_index])->setValue(msg.fb_d_joint_pos_gain[joint_index]);
        ((QDoubleSpinBox *) fb_i_joint_pos_spinbox_[name_index])->setValue(msg.fb_i_joint_pos_gain[joint_index]);
        break;
      }
    }
  }
}

void MainWindow::updateFrictionGainSpinbox(robotis_op3_motion_module_msgs::FrictionGain msg)
{
  for (int name_index=0; name_index<msg.joint_name.size(); name_index++)
  {
    for (int joint_index=0; joint_index<joint_name_.size(); joint_index++)
    {
      if (msg.joint_name[joint_index] == joint_name_[name_index])
      {
        ((QDoubleSpinBox *) fs_friction_spinbox_[name_index])->setValue(msg.friction_static_gain[joint_index]);
        ((QDoubleSpinBox *) fd_friction_spinbox_[name_index])->setValue(msg.friction_dyanmic_gain[joint_index]);
        ((QDoubleSpinBox *) fv_friction_spinbox_[name_index])->setValue(msg.friction_viscous_gain[joint_index]);
        break;
      }
    }
  }
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView()
{
  ui_.view_logging->scrollToBottom();
}

void MainWindow::on_zero_gain_button_clicked(bool check)
{
  for (int name_index=0; name_index<joint_name_.size(); name_index++)
  {
    ((QDoubleSpinBox *) ff_torque_spinbox_[name_index])->setValue(1.0);
    ((QDoubleSpinBox *) fb_p_torque_spinbox_[name_index])->setValue(0.0);
    ((QDoubleSpinBox *) ff_joint_vel_spinbox_[name_index])->setValue(0.0);
    ((QDoubleSpinBox *) fb_p_joint_pos_spinbox_[name_index])->setValue(0.0);
    ((QDoubleSpinBox *) fb_d_joint_pos_spinbox_[name_index])->setValue(0.0);
    ((QDoubleSpinBox *) fb_i_joint_pos_spinbox_[name_index])->setValue(0.0);
  }
}

void MainWindow::on_set_gain_button_clicked(bool check)
{
  robotis_op3_motion_module_msgs::JointControllerGain msg;

  for (int name_index=0; name_index<joint_name_.size(); name_index++)
  {
    msg.joint_name.push_back(joint_name_[name_index]);
    msg.ff_torque_gain.push_back(((QDoubleSpinBox *) ff_torque_spinbox_[name_index])->value());
    msg.fb_p_torque_gain.push_back(((QDoubleSpinBox *) fb_p_torque_spinbox_[name_index])->value());
    msg.ff_joint_vel_gain.push_back(((QDoubleSpinBox *) ff_joint_vel_spinbox_[name_index])->value());
    msg.fb_p_joint_pos_gain.push_back(((QDoubleSpinBox *) fb_p_joint_pos_spinbox_[name_index])->value());
    msg.fb_d_joint_pos_gain.push_back(((QDoubleSpinBox *) fb_d_joint_pos_spinbox_[name_index])->value());
    msg.fb_i_joint_pos_gain.push_back(((QDoubleSpinBox *) fb_i_joint_pos_spinbox_[name_index])->value());
  }

  qnode_.sendJointControllerGain(msg);
}

void MainWindow::on_load_gain_button_clicked(bool check)
{
  qnode_.loadJointControllerGain();
}

void MainWindow::on_save_gain_button_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "save_gain";

  qnode_.saveJointControllerGain(msg);
}

void MainWindow::on_set_value_button_clicked(bool check)
{
  robotis_op3_motion_module_msgs::WholebodyJointPose msg;
  msg.mov_time = ui_.mov_time_spinbox->value();
  msg.joint_state.name.push_back(ui_.joint_name_combobox->currentText().toStdString());
  msg.joint_state.position.push_back(ui_.joint_value_spinbox->value()*DEGREE2RADIAN);

  qnode_.sendJointValue(msg);
}

void MainWindow::on_set_external_force_button_clicked(bool check)
{
  geometry_msgs::WrenchStamped msg;
  msg.header.frame_id = ui_.joint_group_combobox->currentText().toStdString();
  msg.wrench.force.x = ui_.external_force_x_spinbox->value();
  msg.wrench.force.y = ui_.external_force_y_spinbox->value();
  msg.wrench.force.z = ui_.external_force_z_spinbox->value();
  msg.wrench.torque.x = 0.0;
  msg.wrench.torque.y = 0.0;
  msg.wrench.torque.z = 0.0;

  qnode_.sendExternalForce(msg);
}

void MainWindow::on_set_mode_button_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "motion_module";

  qnode_.setMotionModule(msg);
}

void MainWindow::on_gazebo_init_button_clicked(bool check)
{
  std_msgs::Float32 msg;
  msg.data = -0.025;

  qnode_.sendGazeboRobotInit(msg);
}

void MainWindow::on_gazebo_detach_button_clicked(bool check)
{
  std_msgs::Bool msg;
  msg.data = true;

  qnode_.sendGaeboRobotDetach(msg);
}

void MainWindow::on_set_joint_ctrl_button_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "joint_ctrl";

  qnode_.sendMotionControlMode(msg);
}

void MainWindow::on_set_walking_ctrl_button_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "walking";

  qnode_.sendMotionControlMode(msg);
}

void MainWindow::on_apply_torque_offset_button_clicked(bool check)
{
  sensor_msgs::JointState msg;
  msg.name.push_back(ui_.joint_name_combobox->currentText().toStdString());
  msg.effort.push_back(ui_.torque_offset_spinbox->value());

  qnode_.sendJointTorqueOffset(msg);
}

void MainWindow::on_load_torque_offset_button_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "load_joint_torque_offset";

  qnode_.setJointTorqueOffset(msg);
}

void MainWindow::on_save_torque_offset_button_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "save_joint_torque_offset";

  qnode_.setJointTorqueOffset(msg);
}

void MainWindow::on_zero_friction_gain_button_clicked(bool check)
{
  for (int name_index=0; name_index<joint_name_.size(); name_index++)
  {
    ((QDoubleSpinBox *) fs_friction_spinbox_[name_index])->setValue(0.0);
    ((QDoubleSpinBox *) fd_friction_spinbox_[name_index])->setValue(0.0);
    ((QDoubleSpinBox *) fv_friction_spinbox_[name_index])->setValue(0.0);
  }
}

void MainWindow::on_set_friction_gain_button_clicked(bool check)
{
  robotis_op3_motion_module_msgs::FrictionGain msg;

  for (int name_index=0; name_index<joint_name_.size(); name_index++)
  {
    msg.joint_name.push_back(joint_name_[name_index]);
    msg.friction_static_gain.push_back(((QDoubleSpinBox *) fs_friction_spinbox_[name_index])->value());
    msg.friction_dyanmic_gain.push_back(((QDoubleSpinBox *) fd_friction_spinbox_[name_index])->value());
    msg.friction_viscous_gain.push_back(((QDoubleSpinBox *) fv_friction_spinbox_[name_index])->value());
  }

  qnode_.sendFrictionGain(msg);
}

void MainWindow::on_load_friction_gain_button_clicked(bool check)
{
  qnode_.loadFrictionGain();
}

void MainWindow::on_save_friction_gain_button_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "save_gain";

  qnode_.saveFrictionGain(msg);
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered()
{
  QMessageBox::about(this, tr("About ..."),tr("<p>Copyright Robotis</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}


}  // namespace robotis_op3_joint_controller_gui

