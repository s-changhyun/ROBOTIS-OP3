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
#include "../include/robotis_op3_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robotis_op3_gui {

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

  present_task_space_position_spinbox_.push_back(ui_.pre_position_x_spinbox);
  present_task_space_position_spinbox_.push_back(ui_.pre_position_y_spinbox);
  present_task_space_position_spinbox_.push_back(ui_.pre_position_z_spinbox);
  present_task_space_orientation_spinbox_.push_back(ui_.pre_orientation_r_spinbox);
  present_task_space_orientation_spinbox_.push_back(ui_.pre_orientation_p_spinbox);
  present_task_space_orientation_spinbox_.push_back(ui_.pre_orientation_y_spinbox);

  desired_task_space_position_spinbox_.push_back(ui_.des_position_x_spinbox);
  desired_task_space_position_spinbox_.push_back(ui_.des_position_y_spinbox);
  desired_task_space_position_spinbox_.push_back(ui_.des_position_z_spinbox);
  desired_task_space_orientation_spinbox_.push_back(ui_.des_orientation_r_spinbox);
  desired_task_space_orientation_spinbox_.push_back(ui_.des_orientation_p_spinbox);
  desired_task_space_orientation_spinbox_.push_back(ui_.des_orientation_y_spinbox);

  /****************************
  ** Connect
  ****************************/
  qRegisterMetaType<geometry_msgs::Pose>("geometry_msgs::Pose");
  QObject::connect(&qnode_,
                   SIGNAL(updateWholebodyKinematicsPose(geometry_msgs::Pose)),
                   this,
                   SLOT(updateWholebodyKinematicsPoseSpinbox(geometry_msgs::Pose)));

  /*********************
  ** Auto Start
  **********************/
  qnode_.init();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::on_set_mode_button_clicked(bool check)
{
  qnode_.sendSetModeMsg();
}

void MainWindow::on_go_zero_pose_button_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "zero_pose";

  qnode_.parseWholebodyJointPose(msg);
}

void MainWindow::on_go_initial_pose_button_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "init_pose";

  qnode_.parseWholebodyJointPose(msg);
}

void MainWindow::on_send_des_pose_pushbutton_clicked(bool check)
{
  robotis_op3_motion_module_msgs::WholebodyKinematicsPose msg;
  msg.name = ui_.group_name_combobox->currentText().toStdString();
  msg.mov_time = ui_.mov_time_spinbox->value();

  if (msg.name == "pelvis")
  {
    msg.pelvis_pose.position.x = ui_.des_position_x_spinbox->value();
    msg.pelvis_pose.position.y = ui_.des_position_y_spinbox->value();
    msg.pelvis_pose.position.z = ui_.des_position_z_spinbox->value();

    Eigen::Quaterniond orientation =
        robotis_framework::convertRPYToQuaternion(ui_.des_orientation_r_spinbox->value()*DEGREE2RADIAN,
                                                  ui_.des_orientation_p_spinbox->value()*DEGREE2RADIAN,
                                                  ui_.des_orientation_y_spinbox->value()*DEGREE2RADIAN);
    msg.pelvis_pose.orientation.x = orientation.x();
    msg.pelvis_pose.orientation.y = orientation.y();
    msg.pelvis_pose.orientation.z = orientation.z();
    msg.pelvis_pose.orientation.w = orientation.w();
  }
  else if (msg.name == "left_arm")
  {
    msg.l_arm_pose.position.x = ui_.des_position_x_spinbox->value();
    msg.l_arm_pose.position.y = ui_.des_position_y_spinbox->value();
    msg.l_arm_pose.position.z = ui_.des_position_z_spinbox->value();

    Eigen::Quaterniond orientation =
        robotis_framework::convertRPYToQuaternion(ui_.des_orientation_r_spinbox->value()*DEGREE2RADIAN,
                                                  ui_.des_orientation_p_spinbox->value()*DEGREE2RADIAN,
                                                  ui_.des_orientation_y_spinbox->value()*DEGREE2RADIAN);
    msg.l_arm_pose.orientation.x = orientation.x();
    msg.l_arm_pose.orientation.y = orientation.y();
    msg.l_arm_pose.orientation.z = orientation.z();
    msg.l_arm_pose.orientation.w = orientation.w();
  }
  else if (msg.name == "right_arm")
  {
    msg.r_arm_pose.position.x = ui_.des_position_x_spinbox->value();
    msg.r_arm_pose.position.y = ui_.des_position_y_spinbox->value();
    msg.r_arm_pose.position.z = ui_.des_position_z_spinbox->value();

    Eigen::Quaterniond orientation =
        robotis_framework::convertRPYToQuaternion(ui_.des_orientation_r_spinbox->value()*DEGREE2RADIAN,
                                                  ui_.des_orientation_p_spinbox->value()*DEGREE2RADIAN,
                                                  ui_.des_orientation_y_spinbox->value()*DEGREE2RADIAN);
    msg.r_arm_pose.orientation.x = orientation.x();
    msg.r_arm_pose.orientation.y = orientation.y();
    msg.r_arm_pose.orientation.z = orientation.z();
    msg.r_arm_pose.orientation.w = orientation.w();
  }
  else if (msg.name == "left_foot")
  {
    msg.l_foot_pose.position.x = ui_.des_position_x_spinbox->value();
    msg.l_foot_pose.position.y = ui_.des_position_y_spinbox->value();
    msg.l_foot_pose.position.z = ui_.des_position_z_spinbox->value();

    Eigen::Quaterniond orientation =
        robotis_framework::convertRPYToQuaternion(ui_.des_orientation_r_spinbox->value()*DEGREE2RADIAN,
                                                  ui_.des_orientation_p_spinbox->value()*DEGREE2RADIAN,
                                                  ui_.des_orientation_y_spinbox->value()*DEGREE2RADIAN);
    msg.l_foot_pose.orientation.x = orientation.x();
    msg.l_foot_pose.orientation.y = orientation.y();
    msg.l_foot_pose.orientation.z = orientation.z();
    msg.l_foot_pose.orientation.w = orientation.w();
  }
  else if (msg.name == "right_foot")
  {
    msg.r_foot_pose.position.x = ui_.des_position_x_spinbox->value();
    msg.r_foot_pose.position.y = ui_.des_position_y_spinbox->value();
    msg.r_foot_pose.position.z = ui_.des_position_z_spinbox->value();

    Eigen::Quaterniond orientation =
        robotis_framework::convertRPYToQuaternion(ui_.des_orientation_r_spinbox->value()*DEGREE2RADIAN,
                                                  ui_.des_orientation_p_spinbox->value()*DEGREE2RADIAN,
                                                  ui_.des_orientation_y_spinbox->value()*DEGREE2RADIAN);
    msg.r_foot_pose.orientation.x = orientation.x();
    msg.r_foot_pose.orientation.y = orientation.y();
    msg.r_foot_pose.orientation.z = orientation.z();
    msg.r_foot_pose.orientation.w = orientation.w();
  }

  qnode_.sendWholebodyKinematicsPose(msg);
}

void MainWindow::on_get_pre_pose_pushbutton_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = ui_.group_name_combobox->currentText().toStdString();

  qnode_.getWholebodyKinematicsPose(msg);
}

void MainWindow::on_forward_button_clicked(bool check)
{
  robotis_op3_motion_module_msgs::FootStepCommand msg;
  msg.command = "forward";
  msg.step_num = ui_.step_number_spinbox->value();
  msg.step_time = ui_.step_time_spinbox->value();
  msg.step_length = ui_.step_length_spinbox->value();
  msg.side_step_length = ui_.side_length_spinbox->value();
  msg.step_angle_rad = ui_.step_angle_spinbox->value()*DEGREE2RADIAN;

  qnode_.sendFootStepCommand(msg);
}

void MainWindow::on_backward_button_clicked(bool check)
{
  robotis_op3_motion_module_msgs::FootStepCommand msg;
  msg.command = "backward";
  msg.step_num = ui_.step_number_spinbox->value();
  msg.step_time = ui_.step_time_spinbox->value();
  msg.step_length = ui_.step_length_spinbox->value();
  msg.side_step_length = ui_.side_length_spinbox->value();
  msg.step_angle_rad = ui_.step_angle_spinbox->value()*DEGREE2RADIAN;

  qnode_.sendFootStepCommand(msg);
}

void MainWindow::on_stop_button_clicked(bool check)
{
  robotis_op3_motion_module_msgs::FootStepCommand msg;
  msg.command = "stop";
  msg.step_num = ui_.step_number_spinbox->value();
  msg.step_time = ui_.step_time_spinbox->value();
  msg.step_length = ui_.step_length_spinbox->value();
  msg.side_step_length = ui_.side_length_spinbox->value();
  msg.step_angle_rad = ui_.step_angle_spinbox->value()*DEGREE2RADIAN;

  qnode_.sendFootStepCommand(msg);
}

void MainWindow::on_left_button_clicked(bool check)
{
  robotis_op3_motion_module_msgs::FootStepCommand msg;
  msg.command = "left";
  msg.step_num = ui_.step_number_spinbox->value();
  msg.step_time = ui_.step_time_spinbox->value();
  msg.step_length = ui_.step_length_spinbox->value();
  msg.side_step_length = ui_.side_length_spinbox->value();
  msg.step_angle_rad = ui_.step_angle_spinbox->value()*DEGREE2RADIAN;

  qnode_.sendFootStepCommand(msg);
}

void MainWindow::on_right_button_clicked(bool check)
{
  robotis_op3_motion_module_msgs::FootStepCommand msg;
  msg.command = "right";
  msg.step_num = ui_.step_number_spinbox->value();
  msg.step_time = ui_.step_time_spinbox->value();
  msg.step_length = ui_.step_length_spinbox->value();
  msg.side_step_length = ui_.side_length_spinbox->value();
  msg.step_angle_rad = ui_.step_angle_spinbox->value()*DEGREE2RADIAN;

  qnode_.sendFootStepCommand(msg);
}

void MainWindow::on_turn_right_button_clicked(bool check)
{
  robotis_op3_motion_module_msgs::FootStepCommand msg;
  msg.command = "turn right";
  msg.step_num = ui_.step_number_spinbox->value();
  msg.step_time = ui_.step_time_spinbox->value();
  msg.step_length = ui_.step_length_spinbox->value();
  msg.side_step_length = ui_.side_length_spinbox->value();
  msg.step_angle_rad = ui_.step_angle_spinbox->value()*DEGREE2RADIAN;

  qnode_.sendFootStepCommand(msg);
}

void MainWindow::on_turn_left_button_clicked(bool check)
{
  robotis_op3_motion_module_msgs::FootStepCommand msg;
  msg.command = "turn left";
  msg.step_num = ui_.step_number_spinbox->value();
  msg.step_time = ui_.step_time_spinbox->value();
  msg.step_length = ui_.step_length_spinbox->value();
  msg.side_step_length = ui_.side_length_spinbox->value();
  msg.step_angle_rad = ui_.step_angle_spinbox->value()*DEGREE2RADIAN;

  qnode_.sendFootStepCommand(msg);
}

void MainWindow::updateWholebodyKinematicsPoseSpinbox(geometry_msgs::Pose msg)
{
  Eigen::MatrixXd position = Eigen::MatrixXd::Zero(3,1);
  position.coeffRef(0,0) = msg.position.x;
  position.coeffRef(1,0) = msg.position.y;
  position.coeffRef(2,0) = msg.position.z;

  for (int index=0; index<present_task_space_position_spinbox_.size(); index++)
  {
    ((QDoubleSpinBox *) present_task_space_position_spinbox_[index])->setValue(position.coeff(index,0));
    ((QDoubleSpinBox *) desired_task_space_position_spinbox_[index])->setValue(position.coeff(index,0));
  }

  Eigen::Quaterniond quaterion;
  quaterion.x() = msg.orientation.x;
  quaterion.y() = msg.orientation.y;
  quaterion.z() = msg.orientation.z;
  quaterion.w() = msg.orientation.w;

  Eigen::MatrixXd orientation = robotis_framework::convertQuaternionToRPY(quaterion);

  for (int index=0; index<present_task_space_orientation_spinbox_.size(); index++)
  {
    ((QDoubleSpinBox *) present_task_space_orientation_spinbox_[index])->setValue(orientation.coeff(index,0) * RADIAN2DEGREE);
    ((QDoubleSpinBox *) desired_task_space_orientation_spinbox_[index])->setValue(orientation.coeff(index,0) * RADIAN2DEGREE);
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


}  // namespace robotis_op3_gui

