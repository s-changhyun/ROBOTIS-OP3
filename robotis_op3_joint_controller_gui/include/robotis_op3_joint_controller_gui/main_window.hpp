/**
 * @file /include/robotis_op3_joint_controller_gui/main_window.hpp
 *
 * @brief Qt based gui for robotis_op3_joint_controller_gui.
 *
 * @date November 2010
 **/
#ifndef robotis_op3_joint_controller_gui_MAIN_WINDOW_H
#define robotis_op3_joint_controller_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace robotis_op3_joint_controller_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();

  void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:
  /******************************************
  ** Auto-connections (connectSlotsByName())
  *******************************************/
  void on_actionAbout_triggered();

  /******************************************
  ** Manual connections
  *******************************************/
  void updateLoggingView(); // no idea why this can't connect automatically

  void updateJointControllerGainSpinbox(robotis_op3_motion_module_msgs::JointControllerGain msg);
  void updateFrictionGainSpinbox(robotis_op3_motion_module_msgs::FrictionGain msg);

  void on_set_mode_button_clicked(bool check);
  void on_gazebo_init_button_clicked(bool check);
  void on_gazebo_detach_button_clicked(bool check);

  void on_zero_gain_button_clicked(bool check);
  void on_set_gain_button_clicked(bool check);
  void on_load_gain_button_clicked(bool check);
  void on_save_gain_button_clicked(bool check);

  void on_set_joint_ctrl_button_clicked(bool check);
  void on_set_walking_ctrl_button_clicked(bool check);

  void on_set_value_button_clicked(bool check);
  void on_set_external_force_button_clicked(bool check);

  void on_apply_torque_offset_button_clicked(bool check);
  void on_load_torque_offset_button_clicked(bool check);
  void on_save_torque_offset_button_clicked(bool check);

  void on_zero_friction_gain_button_clicked(bool check);
  void on_set_friction_gain_button_clicked(bool check);
  void on_load_friction_gain_button_clicked(bool check);
  void on_save_friction_gain_button_clicked(bool check);

private:
  Ui::MainWindowDesign ui_;
  QNode qnode_;

  std::vector<std::string> joint_name_;

  QList<QAbstractSpinBox *> ff_torque_spinbox_;
  QList<QAbstractSpinBox *> fb_p_torque_spinbox_;

  QList<QAbstractSpinBox *> ff_joint_vel_spinbox_;
  QList<QAbstractSpinBox *> fb_p_joint_pos_spinbox_;
  QList<QAbstractSpinBox *> fb_d_joint_pos_spinbox_;
  QList<QAbstractSpinBox *> fb_i_joint_pos_spinbox_;

  QList<QAbstractSpinBox *> fs_friction_spinbox_;
  QList<QAbstractSpinBox *> fd_friction_spinbox_;
  QList<QAbstractSpinBox *> fv_friction_spinbox_;
};

}  // namespace robotis_op3_joint_controller_gui

#endif // robotis_op3_joint_controller_gui_MAIN_WINDOW_H
