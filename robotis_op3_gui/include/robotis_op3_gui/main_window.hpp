/**
 * @file /include/robotis_op3_gui/main_window.hpp
 *
 * @brief Qt based gui for robotis_op3_gui.
 *
 * @date November 2010
 **/
#ifndef robotis_op3_gui_MAIN_WINDOW_H
#define robotis_op3_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace robotis_op3_gui {

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

  void on_set_mode_button_clicked(bool check);
  void on_go_zero_pose_button_clicked(bool check);
  void on_go_initial_pose_button_clicked(bool check);

  void on_send_des_pose_pushbutton_clicked(bool check);
  void on_get_pre_pose_pushbutton_clicked(bool check);

  void on_forward_button_clicked(bool check);
  void on_backward_button_clicked(bool check);
  void on_stop_button_clicked(bool check);
  void on_left_button_clicked(bool check);
  void on_right_button_clicked(bool check);
  void on_turn_right_button_clicked(bool check);
  void on_turn_left_button_clicked(bool check);

  void updateWholebodyKinematicsPoseSpinbox(geometry_msgs::Pose msg);

  /******************************************
  ** Manual connections
  *******************************************/
  void updateLoggingView(); // no idea why this can't connect automatically

private:
  Ui::MainWindowDesign ui_;
  QNode qnode_;

  QList<QAbstractSpinBox *> present_task_space_position_spinbox_;
  QList<QAbstractSpinBox *> present_task_space_orientation_spinbox_;

  QList<QAbstractSpinBox *> desired_task_space_position_spinbox_;
  QList<QAbstractSpinBox *> desired_task_space_orientation_spinbox_;
};

}  // namespace robotis_op3_gui

#endif // robotis_op3_gui_MAIN_WINDOW_H
