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

/*
 * kinematics_dynamics.cpp
 *
 *  Created on: June 7, 2016
 *      Author: sch
 */

#include "robotis_op3_kinematics_dynamics/kinematics_dynamics.h"
#include <iostream>

namespace robotis_op3
{

KinematicsDynamics::KinematicsDynamics() {}
KinematicsDynamics::~KinematicsDynamics() {}

KinematicsDynamics::KinematicsDynamics(TreeSelect tree)
{
  for (int id=0; id<=ALL_JOINT_ID; id++)
    link_data_[id] = new LinkData();

  if ( tree == WholeBody )
  {
    link_data_[0]->name_              = "base";
    link_data_[0]->parent_            = -1;
    link_data_[0]->sibling_           = -1;
    link_data_[0]->child_             = 16;

    /* ----- passive joint -----*/

    link_data_[16]->name_              = "passive_x";
    link_data_[16]->parent_            = 0;
    link_data_[16]->sibling_           = -1;
    link_data_[16]->child_             = 17;
    link_data_[16]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);

    link_data_[17]->name_              = "passive_y";
    link_data_[17]->parent_            = 16;
    link_data_[17]->sibling_           = -1;
    link_data_[17]->child_             = 18;
    link_data_[17]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);

    link_data_[18]->name_              = "passive_z";
    link_data_[18]->parent_            = 17;
    link_data_[18]->sibling_           = -1;
    link_data_[18]->child_             = 19;
    link_data_[18]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.27915 - 0.0294945); // z: 0.27915

    link_data_[19]->name_              = "passive_yaw";
    link_data_[19]->parent_            = 18;
    link_data_[19]->sibling_           = -1;
    link_data_[19]->child_             = 20;
    link_data_[19]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0,0.0,1.0);

    link_data_[20]->name_              = "passive_pitch";
    link_data_[20]->parent_            = 19;
    link_data_[20]->sibling_           = -1;
    link_data_[20]->child_             = 21;
    link_data_[20]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0,1.0,0.0);

    link_data_[21]->name_              = "passive_roll";
    link_data_[21]->parent_            = 20;
    link_data_[21]->sibling_           = -1;
    link_data_[21]->child_             = 15;
    link_data_[21]->joint_axis_        = robotis_framework::getTransitionXYZ(1.0,0.0,0.0);

    /* ----- body -----*/

    link_data_[15]->name_              = "body";
    link_data_[15]->parent_            = 21;
    link_data_[15]->sibling_           = -1;
    link_data_[15]->child_             = 1;
    link_data_[15]->mass_              = 0.72235;
    link_data_[15]->relative_position_ = robotis_framework::getTransitionXYZ(0.0,0.0,0.0);
    link_data_[15]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0,0.0,0.0);
    link_data_[15]->center_of_mass_    = robotis_framework::getTransitionXYZ(-0.01638,0.00007,0.04972);
    link_data_[15]->inertia_           = robotis_framework::getInertiaXYZ(0.00142735, -0.00000474, 0.00004289, 0.00103312, -0.00000098, 0.00150436);

    /* ----- right leg -----*/

    link_data_[1]->name_              = "r_hip_yaw";
    link_data_[1]->parent_            = 15;
    link_data_[1]->sibling_           = 2;
    link_data_[1]->child_             = 3;
    link_data_[1]->mass_              = 0.01181;
    link_data_[1]->relative_position_ = robotis_framework::getTransitionXYZ(0.0,-0.035,0.0);
    link_data_[1]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0,0.0,-1.0);
    link_data_[1]->center_of_mass_    = robotis_framework::getTransitionXYZ(-0.00157,0.0,-0.00774);
    link_data_[1]->inertia_           = robotis_framework::getInertiaXYZ(0.00000151,0.0,0.00000001,0.00000430,0.0,0.00000412);

    link_data_[3]->name_              = "r_hip_roll";
    link_data_[3]->parent_            = 1;
    link_data_[3]->sibling_           = -1;
    link_data_[3]->child_             = 5;
    link_data_[3]->mass_              = 0.17886;
    link_data_[3]->relative_position_ = robotis_framework::getTransitionXYZ(-0.024,0.0,-0.0285);
    link_data_[3]->joint_axis_        = robotis_framework::getTransitionXYZ(-1.0,0.0,0.0);
    link_data_[3]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.00388,-0.00028,-0.01214);
    link_data_[3]->inertia_           = robotis_framework::getInertiaXYZ(0.00004661,0.00000101,-0.00000131,0.00012523,0.00000006,0.00010857);

    link_data_[5]->name_              = "r_hip_pitch";
    link_data_[5]->parent_            = 3;
    link_data_[5]->sibling_           = -1;
    link_data_[5]->child_             = 7;
    link_data_[5]->mass_              = 0.11543;
    link_data_[5]->relative_position_ = robotis_framework::getTransitionXYZ(0.0241,-0.019,0.0);
    link_data_[5]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0,-1.0,0.0);
    link_data_[5]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.00059,0.01901,-0.08408);
    link_data_[5]->inertia_           = robotis_framework::getInertiaXYZ(0.00010499,0.00000001,-0.00000071,0.00009613,-0.00000353,0.00002493);

    link_data_[7]->name_              = "r_knee";
    link_data_[7]->parent_            = 5;
    link_data_[7]->sibling_           = -1;
    link_data_[7]->child_             = 9;
    link_data_[7]->mass_              = 0.04015;
    link_data_[7]->relative_position_ = robotis_framework::getTransitionXYZ(0.0,0.0,-0.11015);
    link_data_[7]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0,-1.0,0.0);
    link_data_[7]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.00000,0.02151,-0.05500);
    link_data_[7]->inertia_           = robotis_framework::getInertiaXYZ(0.00003715,0.0,0.0,0.00002751,0.0,0.00001511);

    link_data_[9]->name_              = "r_ank_pitch";
    link_data_[9]->parent_            = 7;
    link_data_[9]->sibling_           = -1;
    link_data_[9]->child_             = 11;
    link_data_[9]->mass_              = 0.17886;
    link_data_[9]->relative_position_ = robotis_framework::getTransitionXYZ(0.0,0.0,-0.110);
    link_data_[9]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0,1.0,0.0);
    link_data_[9]->center_of_mass_    = robotis_framework::getTransitionXYZ(-0.02022,0.01872,0.01214);
    link_data_[9]->inertia_           = robotis_framework::getInertiaXYZ(0.00004661,0.00000101,0.00000131,0.00012523,-0.00000006,0.00010857);

    link_data_[11]->name_              = "r_ank_roll";
    link_data_[11]->parent_            = 9;
    link_data_[11]->sibling_           = -1;
    link_data_[11]->child_             = 13;
    link_data_[11]->mass_              = 0.06934;
    link_data_[11]->relative_position_ = robotis_framework::getTransitionXYZ(-0.0241,0.019,0.0);
    link_data_[11]->joint_axis_        = robotis_framework::getTransitionXYZ(1.0,0.0,0.0);
    link_data_[11]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.02373,-0.01037,-0.02760);
    link_data_[11]->inertia_           = robotis_framework::getInertiaXYZ(0.00004034,0.00000019,0.00000012,0.00007874,-0.00000101,0.00011579);

    link_data_[13]->name_              = "r_foot";
    link_data_[13]->parent_            = 11;
    link_data_[13]->sibling_           = -1;
    link_data_[13]->child_             = -1;
    link_data_[13]->relative_position_ = robotis_framework::getTransitionXYZ(0.0,0.0,-0.0305);

    /* ----- left leg -----*/

    link_data_[2]->name_              = "l_hip_yaw";
    link_data_[2]->parent_            = 15;
    link_data_[2]->sibling_           = -1;
    link_data_[2]->child_             = 4;
    link_data_[2]->mass_              = 0.01181;
    link_data_[2]->relative_position_ = robotis_framework::getTransitionXYZ(0.0,0.035,0.0);
    link_data_[2]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0,0.0,-1.0);
    link_data_[2]->center_of_mass_    = robotis_framework::getTransitionXYZ(-0.00157,0.0,-0.00774);
    link_data_[2]->inertia_           = robotis_framework::getInertiaXYZ(0.00000151,0.0,0.00000001,0.00000430,0.0,0.00000412);

    link_data_[4]->name_              = "l_hip_roll";
    link_data_[4]->parent_            = 2;
    link_data_[4]->sibling_           = -1;
    link_data_[4]->child_             = 6;
    link_data_[4]->mass_              = 0.17886;
    link_data_[4]->relative_position_ = robotis_framework::getTransitionXYZ(-0.024,0.0,-0.0285);
    link_data_[4]->joint_axis_        = robotis_framework::getTransitionXYZ(-1.0,0.0,0.0);
    link_data_[4]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.00388,0.00028,-0.01214);
    link_data_[4]->inertia_           = robotis_framework::getInertiaXYZ(0.00004661,-0.00000101,-0.00000131,0.00012523,-0.00000006,0.00010857);

    link_data_[6]->name_              = "l_hip_pitch";
    link_data_[6]->parent_            = 4;
    link_data_[6]->sibling_           = -1;
    link_data_[6]->child_             = 8;
    link_data_[6]->mass_              = 0.11543;
    link_data_[6]->relative_position_ = robotis_framework::getTransitionXYZ(0.0241,0.019,0.0);
    link_data_[6]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0,1.0,0.0);
    link_data_[6]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.00059,-0.01901,-0.08408);
    link_data_[6]->inertia_           = robotis_framework::getInertiaXYZ(0.00010499,-0.00000001,-0.00000071,0.00009613,0.00000353,0.00002493);

    link_data_[8]->name_              = "l_knee";
    link_data_[8]->parent_            = 6;
    link_data_[8]->sibling_           = -1;
    link_data_[8]->child_             = 10;
    link_data_[8]->mass_              = 0.04015;
    link_data_[8]->relative_position_ = robotis_framework::getTransitionXYZ(0.0,0.0,-0.11015);
    link_data_[8]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0,1.0,0.0);
    link_data_[8]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.00000,-0.02151,-0.05500);
    link_data_[8]->inertia_           = robotis_framework::getInertiaXYZ(0.00003715,0.0,0.0,0.00002751,0.0,0.00001511);

    link_data_[10]->name_              = "l_ank_pitch";
    link_data_[10]->parent_            = 8;
    link_data_[10]->sibling_           = -1;
    link_data_[10]->child_             = 12;
    link_data_[10]->mass_              = 0.17886;
    link_data_[10]->relative_position_ = robotis_framework::getTransitionXYZ(0.0,0.0,-0.110);
    link_data_[10]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0,-1.0,0.0);
    link_data_[10]->center_of_mass_    = robotis_framework::getTransitionXYZ(-0.02022,-0.01872,0.01214);
    link_data_[10]->inertia_           = robotis_framework::getInertiaXYZ(0.00004661,-0.00000101,0.00000131,0.00012523,0.00000006,0.00010857);

    link_data_[12]->name_              = "l_ank_roll";
    link_data_[12]->parent_            = 10;
    link_data_[12]->sibling_           = -1;
    link_data_[12]->child_             = 14;
    link_data_[12]->mass_              = 0.06934;
    link_data_[12]->relative_position_ = robotis_framework::getTransitionXYZ(-0.0241,-0.019,0.0);
    link_data_[12]->joint_axis_        = robotis_framework::getTransitionXYZ(1.0,0.0,0.0);
    link_data_[12]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.02373,0.01037,-0.02760);
    link_data_[12]->inertia_           = robotis_framework::getInertiaXYZ(0.00004034,-0.00000019,0.00000012,0.00007874,0.00000101,0.00011579);

    link_data_[14]->name_              = "l_foot";
    link_data_[14]->parent_            = 12;
    link_data_[14]->sibling_           = -1;
    link_data_[14]->child_             = -1;
    link_data_[14]->relative_position_ = robotis_framework::getTransitionXYZ(0.0,0.0,-0.0305);
  }

  thigh_length_m_ = std::fabs(link_data_[ID_R_LEG_START+2*3]->relative_position_.coeff(2,0));
  calf_length_m_  = std::fabs(link_data_[ID_R_LEG_START+2*4]->relative_position_.coeff(2,0));
  ankle_length_m_ = std::fabs(link_data_[ID_R_LEG_END]->relative_position_.coeff(2,0));
  leg_side_offset_m_ 	= 2.0*(std::fabs(link_data_[ID_R_LEG_START]->relative_position_.coeff(1,0)));
}

std::vector<int> KinematicsDynamics::findRoute(int to)
{
  int id = link_data_[to]->parent_;

  std::vector<int> idx;

  if(id == 0)
  {
    idx.push_back(0);
    idx.push_back(to);
  }
  else
  {
    idx = findRoute(id);
    idx.push_back(to);
  }

  return idx;
}

std::vector<int> KinematicsDynamics::findRoute(int from, int to)
{
  int id = link_data_[to]->parent_;

  std::vector<int> idx;

  if(id == from)
  {
    idx.push_back(from);
    idx.push_back(to);
  }
  else if (id != 0)
  {
    idx = findRoute(from, id);
    idx.push_back(to);
  }

  return idx;
}

double KinematicsDynamics::calcTotalMass(int joint_id)
{
  double mass;

  if (joint_id == -1)
    mass = 0.0;
  else
    mass = link_data_[joint_id]->mass_ + calcTotalMass(link_data_[ joint_id ]->sibling_) + calcTotalMass(link_data_[joint_id]->child_);

  return mass;
}

Eigen::Vector3d KinematicsDynamics::calcMassCenter(int joint_id)
{
  Eigen::Vector3d mc;
  mc.fill(0);

  if(joint_id != -1)
  {
    mc = link_data_[ joint_id ]->mass_ * ( link_data_[ joint_id ]->orientation_ * link_data_[ joint_id ]->center_of_mass_ + link_data_[ joint_id ]->position_ );
    mc = mc + calcMassCenter( link_data_[ joint_id ]->sibling_ ) + calcMassCenter( link_data_[ joint_id ]->child_ );
  }

  return mc;
}

void KinematicsDynamics::calcJointsCenterOfMass(int joint_id)
{
  if(joint_id != -1)
  {
    LinkData *temp_data = link_data_[ joint_id ];
    temp_data->joint_center_of_mass_
      = ( temp_data->orientation_ * temp_data->center_of_mass_ + temp_data->position_ );

    calcJointsCenterOfMass(temp_data->sibling_);
    calcJointsCenterOfMass(temp_data->child_);
  }
  else
    return;
}

Eigen::Vector3d KinematicsDynamics::calcCenterOfMass(const Eigen::Vector3d& mc)
{
  double mass ;
  Eigen::Vector3d COM;

  mass = calcTotalMass(0);
  COM = mc/mass;

  return COM;
}

void KinematicsDynamics::calcForwardKinematics(int joint_id)
{
  if (joint_id == -1)
    return;

  if (joint_id == 0)
  {
    link_data_[0]->position_ = Eigen::MatrixXd::Zero(3,1);
    link_data_[0]->orientation_ =
        robotis_framework::calcRodrigues( robotis_framework::calcHatto( link_data_[0]->joint_axis_ ), link_data_[ 0 ]->joint_angle_ );
  }

  if ( joint_id != 0 )
  {
    int parent = link_data_[joint_id]->parent_;

    link_data_[joint_id]->position_ =
        link_data_[parent]->orientation_ * link_data_[joint_id]->relative_position_ + link_data_[parent]->position_;
    link_data_[ joint_id ]->orientation_ =
        link_data_[ parent ]->orientation_ *
        robotis_framework::calcRodrigues(robotis_framework::calcHatto(link_data_[joint_id]->joint_axis_), link_data_[joint_id]->joint_angle_);

    link_data_[joint_id]->transformation_.block<3,1>(0,3) = link_data_[joint_id]->position_;
    link_data_[joint_id]->transformation_.block<3,3>(0,0) = link_data_[joint_id]->orientation_;
  }

  calcForwardKinematics(link_data_[joint_id]->sibling_);
  calcForwardKinematics(link_data_[joint_id]->child_);
}

Eigen::MatrixXd KinematicsDynamics::calcJacobian(const std::vector<int>& idx)
{
  int idx_size = idx.size();
  int end = idx_size-1;

  Eigen::MatrixXd tar_position = link_data_[idx[end]]->position_;
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6,idx_size);

  for (int id=0; id<idx_size; id++)
  {
    int curr_id = idx[id];

    Eigen::MatrixXd tar_orientation = link_data_[curr_id]->orientation_ * link_data_[curr_id]->joint_axis_;

    jacobian.block(0,id,3,1) = robotis_framework::calcCross(tar_orientation,tar_position-link_data_[curr_id]->position_);
    jacobian.block(3,id,3,1) = tar_orientation;
  }

  return jacobian;
}

Eigen::MatrixXd KinematicsDynamics::calcJacobianWholebody(const std::vector<int>& idx)
{
  int idx_size = idx.size();
  int end = idx_size-1;

  Eigen::MatrixXd tar_position = link_data_[idx[end]]->position_;
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6,idx_size);

  for (int id=0; id<idx_size; id++)
  {
    int curr_id = idx[id];

    Eigen::MatrixXd tar_orientation = link_data_[curr_id]->orientation_ * link_data_[curr_id]->joint_axis_;

    jacobian.block(0,id,3,1) = robotis_framework::calcCross(tar_orientation,tar_position-link_data_[curr_id]->position_);
    jacobian.block(3,id,3,1) = tar_orientation;

    if (curr_id == ID_PELVIS_X)
      jacobian.coeffRef(0,id) = 1.0;
    else if (curr_id == ID_PELVIS_Y)
      jacobian.coeffRef(1,id) = 1.0;
    else if (curr_id == ID_PELVIS_Z)
      jacobian.coeffRef(2,id) = 1.0;
  }

  return jacobian;
}


Eigen::MatrixXd KinematicsDynamics::calcJacobianCOM(const std::vector<int>& idx)
{
  int idx_size = idx.size();

  Eigen::MatrixXd jacobian_com = Eigen::MatrixXd::Zero(6,idx_size);

  double total_mass = calcTotalMass(0);

  for (int id=0; id<idx_size; id++)
  {
    int curr_id = idx[id];
    double mass = calcTotalMass(curr_id);

    Eigen::MatrixXd og = calcMassCenter(curr_id)/mass-link_data_[curr_id]->position_;
    Eigen::MatrixXd tar_orientation = link_data_[curr_id]->orientation_ * link_data_[curr_id]->joint_axis_;

    jacobian_com.block(0,id,3,1) = (mass / total_mass) * robotis_framework::calcCross(tar_orientation, og);
    jacobian_com.block(3,id,3,1) = (mass / total_mass) * tar_orientation;
  }

  return jacobian_com;
}

Eigen::MatrixXd KinematicsDynamics::calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position, Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation)
{
  Eigen::MatrixXd pos_err = tar_position - curr_position;
  Eigen::MatrixXd ori_err = curr_orientation.transpose() * tar_orientation;
  Eigen::MatrixXd ori_err_dash = curr_orientation * robotis_framework::convertRotToOmega(ori_err);

  Eigen::MatrixXd err = Eigen::MatrixXd::Zero(6,1);
  err.block<3,1>(0,0) = pos_err;
  err.block<3,1>(3,0) = ori_err_dash;

  return err;
}

bool KinematicsDynamics::calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err)
{
  bool ik_success = false;
  bool limit_success = false;

  //  calcForwardKinematics(0);

  std::vector<int> idx = findRoute(to);

  for (int iter=0; iter<max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);

    Eigen::MatrixXd curr_position = link_data_[to]->position_;
    Eigen::MatrixXd curr_orientation = link_data_[to]->orientation_;

    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm()<ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd jacobian_trans = jacobian * jacobian.transpose();
    Eigen::MatrixXd jacobian_inverse = jacobian.transpose() * jacobian_trans.inverse();

    Eigen::MatrixXd delta_angle = jacobian_inverse * err ;

    for (int id=0; id<idx.size(); id++)
    {
      int joint_num = idx[id];
      link_data_[joint_num]->joint_angle_ += delta_angle.coeff(id);
    }

    calcForwardKinematics(0);
  }

  for ( int id = 0; id < idx.size(); id++ )
  {
    int joint_num      = 	idx[ id ];

    if ( link_data_[ joint_num ]->joint_angle_ >= link_data_[ joint_num ]->joint_limit_max_ )
    {
      limit_success = false;
      break;
    }
    else if ( link_data_[ joint_num ]->joint_angle_ <= link_data_[ joint_num ]->joint_limit_min_ )
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    return false;
}

bool KinematicsDynamics::calcInverseKinematics(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err)
{
  bool ik_success = false;
  bool limit_success = false;

  //  calcForwardKinematics(0);

  std::vector<int> idx = findRoute(from, to);

  for (int iter=0; iter<max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);

    Eigen::MatrixXd curr_position = link_data_[to]->position_;
    Eigen::MatrixXd curr_orientation = link_data_[to]->orientation_;

    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm()<ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd jacobian_trans = jacobian * jacobian.transpose();
    Eigen::MatrixXd jacobian_inv = jacobian.transpose() * jacobian_trans.inverse();

    Eigen::MatrixXd delta_angle = jacobian_inv * err ;

    for (int id=0; id<idx.size(); id++)
    {
      int joint_num = idx[id];
      link_data_[joint_num]->joint_angle_ +=delta_angle.coeff(id);
    }

    calcForwardKinematics(0);
  }

  for ( int id = 0; id < idx.size(); id++ )
  {
    int joint_num      =   idx[ id ];

    if ( link_data_[ joint_num ]->joint_angle_ >= link_data_[ joint_num ]->joint_limit_max_ )
    {
      limit_success = false;
      break;
    }
    else if ( link_data_[ joint_num ]->joint_angle_ <= link_data_[ joint_num ]->joint_limit_min_ )
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    return false;
}

bool KinematicsDynamics::calcInverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err , Eigen::MatrixXd weight)
{
  bool ik_success = false;
  bool limit_success = false;

  //  calcForwardKinematics(0);

  std::vector<int> idx = findRoute(to);

  /* weight */
  Eigen::MatrixXd weight_matrix = Eigen::MatrixXd::Identity(idx.size(), idx.size());

  for ( int ix = 0; ix < idx.size(); ix++ )
    weight_matrix.coeffRef(ix,ix) = weight.coeff(idx[ix],0);

  /* damping */
  Eigen::MatrixXd eval = Eigen::MatrixXd::Zero(6,6);

  double p_damping = 1e-5;
  double R_damping = 1e-5;

  for (int ix=0; ix<3; ix++)
  {
    eval.coeffRef(ix,ix) = p_damping;
    eval.coeffRef(ix+3,ix+3) = R_damping;
  }

  /* ik */
  for (int iter=0; iter<max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);

    Eigen::MatrixXd curr_position = link_data_[to]->position_;
    Eigen::MatrixXd curr_orientation = link_data_[to]->orientation_;

    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm()<ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd jacobian_trans = (jacobian * weight_matrix * jacobian.transpose() + eval);
    Eigen::MatrixXd jacobian_inv = weight_matrix * jacobian.transpose() * jacobian_trans.inverse();

    Eigen::MatrixXd delta_angle = jacobian_inv * err ;

    for (int id=0; id<idx.size(); id++)
    {
      int joint_id = idx[id];
      link_data_[joint_id]->joint_angle_ += delta_angle.coeff(id);
    }

    calcForwardKinematics(0);
  }

  /* check joint limit */
  for ( int id = 0; id < idx.size(); id++ )
  {
    int joint_num      =   idx[ id ];

    if ( link_data_[ joint_num ]->joint_angle_ >= link_data_[ joint_num ]->joint_limit_max_ )
    {
      limit_success = false;
      break;
    }
    else if ( link_data_[ joint_num ]->joint_angle_ <= link_data_[ joint_num ]->joint_limit_min_ )
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    return false;
}

bool KinematicsDynamics::calcInverseKinematics
(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err, Eigen::MatrixXd weight)
{
  bool ik_success = false;
  bool limit_success = false;

  //  calcForwardKinematics(0);

  std::vector<int> idx = findRoute(from, to);

  /* weight */
  Eigen::MatrixXd weight_matrix = Eigen::MatrixXd::Identity(idx.size(),idx.size());

  for (int ix = 0; ix < idx.size(); ix++)
    weight_matrix.coeffRef(ix, ix) = weight.coeff(idx[ix], 0);

  /* damping */
  Eigen::MatrixXd eval = Eigen::MatrixXd::Zero(6, 6);

  double p_damping = 1e-5;
  double R_damping = 1e-5;

  for (int ix = 0; ix < 3; ix++)
  {
    eval.coeffRef(ix, ix) = p_damping;
    eval.coeffRef(ix + 3, ix + 3) = R_damping;
  }

  /* ik */
  for (int iter = 0; iter < max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);
    Eigen::MatrixXd curr_position = link_data_[to]->position_;
    Eigen::MatrixXd curr_orientation = link_data_[to]->orientation_;
    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm() < ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd jacobian_trans = (jacobian * weight_matrix * jacobian.transpose() + eval);
    Eigen::MatrixXd jacobian_inv = weight_matrix * jacobian.transpose() * jacobian_trans.inverse();
    Eigen::MatrixXd delta_angle = jacobian_inv * err;

    for (int id = 0; id < idx.size(); id++)
    {
      int joint_id = idx[id];
      link_data_[joint_id]->joint_angle_ += delta_angle.coeff(id);
    }
    calcForwardKinematics(0);
  }

  /* check joint limit */
  for (int id = 0; id < idx.size(); id++)
  {
    int _joint_num = idx[id];
    if (link_data_[_joint_num]->joint_angle_ >= link_data_[_joint_num]->joint_limit_max_)
    {
      limit_success = false;
      break;
    }
    else if (link_data_[_joint_num]->joint_angle_ <= link_data_[_joint_num]->joint_limit_min_)
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    return false;
}

bool KinematicsDynamics::calcInverseKinematicsForWholebody
(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err, Eigen::MatrixXd weight)
{
  bool ik_success = false;
  bool limit_success = false;

  //  calcForwardKinematics(0);

  std::vector<int> idx = findRoute(from, to);

  /* weight */
  Eigen::MatrixXd weight_matrix = Eigen::MatrixXd::Identity(idx.size(),idx.size());

  for (int ix = 0; ix < idx.size(); ix++)
    weight_matrix.coeffRef(ix, ix) = weight.coeff(idx[ix], 0);

  /* damping */
  Eigen::MatrixXd eval = Eigen::MatrixXd::Zero(6, 6);

  double p_damping = 1e-5;
  double R_damping = 1e-5;

  for (int ix = 0; ix < 3; ix++)
  {
    eval.coeffRef(ix, ix) = p_damping;
    eval.coeffRef(ix + 3, ix + 3) = R_damping;
  }

  /* ik */
  for (int iter = 0; iter < max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobianWholebody(idx);
    Eigen::MatrixXd curr_position = link_data_[to]->position_;
    Eigen::MatrixXd curr_orientation = link_data_[to]->orientation_;
    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm() < ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd jacobian_trans = (jacobian * weight_matrix * jacobian.transpose() + eval);
    Eigen::MatrixXd jacobian_inv = weight_matrix * jacobian.transpose() * jacobian_trans.inverse();
    Eigen::MatrixXd delta_angle = jacobian_inv * err;

    for (int id = 0; id < idx.size(); id++)
    {
      int joint_id = idx[id];
//      link_data_[joint_id]->joint_angle_ += delta_angle.coeff(id);

      if (joint_id == ID_PELVIS_X)
        link_data_[joint_id]->relative_position_.coeffRef(0,0) =
            link_data_[joint_id]->relative_position_.coeff(0,0) + delta_angle.coeff(id);
      else if (joint_id == ID_PELVIS_Y)
        link_data_[joint_id]->relative_position_.coeffRef(1,0) =
            link_data_[joint_id]->relative_position_.coeff(1,0) + delta_angle.coeff(id);
      else if (joint_id == ID_PELVIS_Z)
        link_data_[joint_id]->relative_position_.coeffRef(2,0) =
            link_data_[joint_id]->relative_position_.coeff(2,0) + delta_angle.coeff(id);
      else
        link_data_[joint_id]->joint_angle_ += delta_angle.coeff(id);
    }
    calcForwardAllKinematics(0);
  }

  /* check joint limit */
  for (int id = 0; id < idx.size(); id++)
  {
    int _joint_num = idx[id];
    if (link_data_[_joint_num]->joint_angle_ >= link_data_[_joint_num]->joint_limit_max_)
    {
      limit_success = false;
      break;
    }
    else if (link_data_[_joint_num]->joint_angle_ <= link_data_[_joint_num]->joint_limit_min_)
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    return false;
}

bool KinematicsDynamics::calcInverseKinematicsForLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw)
{
  //Eigen::MatrixXd target_transform;
  Eigen::Matrix4d trans_ad, trans_da, trans_cd, trans_dc, trans_ac;
  Eigen::Vector3d vec;

  bool  invertible;
  double rac, arc_cos, arc_tan, k, l, m, n, s, c, theta;
  double thigh_length = thigh_length_m_;
  double calf_length = calf_length_m_;
  double ankle_length = ankle_length_m_;

  trans_ad = robotis_framework::getTransformationXYZRPY(x, y, z, roll, pitch, yaw);

  vec.coeffRef(0) = trans_ad.coeff(0,3) + trans_ad.coeff(0,2) * ankle_length;
  vec.coeffRef(1) = trans_ad.coeff(1,3) + trans_ad.coeff(1,2) * ankle_length;
  vec.coeffRef(2) = trans_ad.coeff(2,3) + trans_ad.coeff(2,2) * ankle_length;

  // Get Knee
  rac = vec.norm();
  arc_cos = acos((rac * rac - thigh_length * thigh_length - calf_length * calf_length) / (2.0 * thigh_length * calf_length));
  if(std::isnan(arc_cos) == 1)
    return false;
  *(out + 3) = arc_cos;

  // Get Ankle Roll
  trans_ad.computeInverseWithCheck(trans_da, invertible);
  if(invertible == false)
    return false;

  k = sqrt(trans_da.coeff(1,3) * trans_da.coeff(1,3) +  trans_da.coeff(2,3) * trans_da.coeff(2,3));
  l = sqrt(trans_da.coeff(1,3) * trans_da.coeff(1,3) + (trans_da.coeff(2,3) - ankle_length)*(trans_da.coeff(2,3) - ankle_length));
  m = (k * k - l * l - ankle_length * ankle_length) / (2.0 * l * ankle_length);

  if(m > 1.0)
    m = 1.0;
  else if(m < -1.0)
    m = -1.0;
  arc_cos = acos(m);

  if(std::isnan(arc_cos) == 1)
    return false;

  if(trans_da.coeff(1,3) < 0.0)
    *(out + 5) = -arc_cos;
  else
    *(out + 5) = arc_cos;

  // Get Hip Yaw
  trans_cd = robotis_framework::getTransformationXYZRPY(0, 0, -ankle_length, *(out + 5), 0, 0);
  trans_cd.computeInverseWithCheck(trans_dc, invertible);
  if(invertible == false)
    return false;

  trans_ac = trans_ad * trans_dc;
  arc_tan = atan2(-trans_ac.coeff(0,1) , trans_ac.coeff(1,1));
  if(std::isinf(arc_tan) != 0)
    return false;
  *(out) = arc_tan;

  // Get Hip Roll
  arc_tan = atan2(trans_ac.coeff(2,1), -trans_ac.coeff(0,1) * sin(*(out)) + trans_ac.coeff(1,1) * cos(*(out)));
  if(std::isinf(arc_tan) != 0)
    return false;
  *(out + 1) = arc_tan;

  // Get Hip Pitch and Ankle Pitch
  arc_tan = atan2(trans_ac.coeff(0,2) * cos(*(out)) + trans_ac.coeff(1,2) * sin(*(out)), trans_ac.coeff(0,0) * cos(*(out)) + trans_ac.coeff(1,0) * sin(*(out)));
  if(std::isinf(arc_tan) == 1)
    return false;
  theta = arc_tan;
  k = sin(*(out + 3)) * calf_length;
  l = -thigh_length - cos(*(out + 3)) * calf_length;
  m = cos(*(out)) * vec.coeff(0) + sin(*(out)) * vec.coeff(1);
  n = cos(*(out + 1)) * vec.coeff(2) + sin(*(out)) * sin(*(out + 1)) * vec.coeff(0) - cos(*(out)) * sin(*(out + 1)) * vec.coeff(1);
  s = (k * n + l * m) / (k * k + l * l);
  c = (n - k * s) / l;
  arc_tan = atan2(s, c);
  if(std::isinf(arc_tan) == 1)
    return false;
  *(out + 2) = arc_tan;
  *(out + 4) = theta - *(out + 3) - *(out + 2);

  return true;
}

bool KinematicsDynamics::calcInverseKinematicsForRightLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw)
{
  if(calcInverseKinematicsForLeg(out, x, y, z, roll, pitch, yaw) == true) {

    *(out + 0) = out[0] * (link_data_[ID_R_LEG_START + 2*0]->joint_axis_.coeff(2,0));
    *(out + 1) = out[1] * (link_data_[ID_R_LEG_START + 2*1]->joint_axis_.coeff(0,0));
    *(out + 2) = out[2] * (link_data_[ID_R_LEG_START + 2*2]->joint_axis_.coeff(1,0));
    *(out + 3) = out[3] * (link_data_[ID_R_LEG_START + 2*3]->joint_axis_.coeff(1,0));
    *(out + 4) = out[4] * (link_data_[ID_R_LEG_START + 2*4]->joint_axis_.coeff(1,0));
    *(out + 5) = out[5] * (link_data_[ID_R_LEG_START + 2*5]->joint_axis_.coeff(0,0));
    return true;
  }
  else
    return false;
}

bool KinematicsDynamics::calcInverseKinematicsForLeftLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw)
{
  if(calcInverseKinematicsForLeg(out, x, y, z, roll, pitch, yaw) == true) {

    out[0] = out[0] * (link_data_[ID_L_LEG_START + 2*0]->joint_axis_.coeff(2,0));
    out[1] = out[1] * (link_data_[ID_L_LEG_START + 2*1]->joint_axis_.coeff(0,0));
    out[2] = out[2] * (link_data_[ID_L_LEG_START + 2*2]->joint_axis_.coeff(1,0));
    out[3] = out[3] * (link_data_[ID_L_LEG_START + 2*3]->joint_axis_.coeff(1,0));
    out[4] = out[4] * (link_data_[ID_L_LEG_START + 2*4]->joint_axis_.coeff(1,0));
    out[5] = out[5] * (link_data_[ID_L_LEG_START + 2*5]->joint_axis_.coeff(0,0));
    return true;
  }
  else
    return false;
}

//void KinematicsDynamics::calcEulerDynamics(int joint_id)
//{
//  Eigen::MatrixXd I =
//      link_data_[joint_id]->orientation_ * link_data_[joint_id]->inertia_ * link_data_[joint_id]->orientation_.transpose();
//  Eigen::MatrixXd L =
//      I * link_data_[joint_id]->angular_velocity_;
//  link_data_[joint_id]->angular_acceleration_ =
//      I.inverse() * (-1.0 * robotis_framework::calcCross(link_data_[joint_id]->angular_velocity_,L));
//}

void KinematicsDynamics::calcForwardAllKinematics(int joint_id)
{
  if (joint_id == -1)
    return;

  if (joint_id == 0)
    link_data_[0]->linear_acceleration_.coeffRef(2,0) = 9.81;

  if ( joint_id != 0 )
  {
    int parent = link_data_[joint_id]->parent_;

    // position and orientation
    link_data_[joint_id]->position_ =
        link_data_[parent]->orientation_ * link_data_[joint_id]->relative_position_ + link_data_[parent]->position_;
    link_data_[joint_id]->orientation_ =
        link_data_[parent]->orientation_ *
        robotis_framework::calcRodrigues(robotis_framework::calcHatto(link_data_[joint_id]->joint_axis_), link_data_[joint_id]->joint_angle_);

    // spatial velocity
    Eigen::Vector3d sw = link_data_[parent]->orientation_ * link_data_[joint_id]->joint_axis_;
    Eigen::Vector3d sv = robotis_framework::calcCross(link_data_[joint_id]->position_,sw);

    link_data_[joint_id]->angular_velocity_ =
        link_data_[parent]->angular_velocity_ + sw * link_data_[joint_id]->joint_velocity_;
    link_data_[joint_id]->linear_velocity_ =
        link_data_[parent]->linear_velocity_ + sv * link_data_[joint_id]->joint_velocity_;

//    if ((link_data_[joint_id]->joint_axis_).norm() == 1.0)
//    {
//      ROS_INFO("----- ID : %d -----", joint_id);
//      PRINT_MAT(sw);
//      PRINT_MAT(sv);
//      PRINT_MAT(link_data_[joint_id]->angular_velocity_);
//      PRINT_MAT(link_data_[joint_id]->linear_velocity_);
//    }

    // spatial acceleration
    Eigen::Vector3d dsv =
        robotis_framework::calcCross(link_data_[parent]->angular_velocity_,sv) +
        robotis_framework::calcCross(link_data_[parent]->linear_velocity_,sw);
    Eigen::Vector3d dsw =
        robotis_framework::calcCross(link_data_[parent]->angular_velocity_,sw);

    link_data_[joint_id]->angular_acceleration_ =
        link_data_[parent]->angular_acceleration_ +
        dsw * link_data_[joint_id]->joint_velocity_ + sw * link_data_[joint_id]->joint_acceleration_;
    link_data_[joint_id]->linear_acceleration_ =
        link_data_[parent]->linear_acceleration_ +
        dsv * link_data_[joint_id]->joint_velocity_ + sv * link_data_[joint_id]->joint_acceleration_;

    link_data_[joint_id]->spatial_velocity_ = sv;
    link_data_[joint_id]->spatial_angular_velocity_ = sw;
  }

  calcForwardAllKinematics(link_data_[joint_id]->sibling_);
  calcForwardAllKinematics(link_data_[joint_id]->child_);
}

Eigen::VectorXd KinematicsDynamics::calcInverseDynamics(int joint_id)
{
  Eigen::VectorXd spatial_force;
  spatial_force.resize(6);
  spatial_force.fill(0);
  if (joint_id == -1)
    return spatial_force;

  Eigen::Vector3d center_of_mass =
      link_data_[joint_id]->orientation_ * link_data_[joint_id]->center_of_mass_ + link_data_[joint_id]->position_;

  Eigen::Matrix3d inertia_tensor =
      link_data_[joint_id]->orientation_ * link_data_[joint_id]->inertia_ * link_data_[joint_id]->orientation_.transpose();

  Eigen::Matrix3d hatto_center_of_mass = robotis_framework::calcHatto(center_of_mass);
  Eigen::Matrix3d inertia_tensor_new = inertia_tensor + link_data_[joint_id]->mass_ * hatto_center_of_mass * hatto_center_of_mass.transpose();

  Eigen::Vector3d momentum =
      link_data_[joint_id]->mass_ *
      (link_data_[joint_id]->linear_velocity_ + robotis_framework::calcCross(link_data_[joint_id]->angular_velocity_,center_of_mass));

  Eigen::Vector3d angular_momentum =
      link_data_[joint_id]->mass_ *
      robotis_framework::calcCross(center_of_mass,link_data_[joint_id]->linear_velocity_) +
      inertia_tensor_new * link_data_[joint_id]->angular_velocity_;

  Eigen::Vector3d force_default =
      link_data_[joint_id]->mass_ *
      (link_data_[joint_id]->linear_acceleration_ + robotis_framework::calcCross(link_data_[joint_id]->angular_acceleration_,center_of_mass)) +
      robotis_framework::calcCross(link_data_[joint_id]->angular_velocity_, momentum);

  Eigen::Vector3d torque_default =
      link_data_[joint_id]->mass_ *
      robotis_framework::calcCross(center_of_mass, link_data_[joint_id]->linear_acceleration_) +
      inertia_tensor_new * link_data_[joint_id]->angular_acceleration_ +
      robotis_framework::calcCross(link_data_[joint_id]->linear_velocity_, momentum) +
      robotis_framework::calcCross(link_data_[joint_id]->angular_velocity_, angular_momentum);

  Eigen::VectorXd force_torque_default;
  force_torque_default.resize(6);
  force_torque_default.block<3,1>(0,0) = force_default;
  force_torque_default.block<3,1>(3,0) = torque_default;

  /* ---------- */

  Eigen::MatrixXd external_force_matrix = Eigen::MatrixXd::Identity(6,6);
  external_force_matrix.block<3,3>(0,0) = link_data_[joint_id]->orientation_;
  external_force_matrix.block<3,3>(3,3) = link_data_[joint_id]->orientation_;
  external_force_matrix.block<3,3>(3,0) = robotis_framework::calcHatto(link_data_[joint_id]->position_) * link_data_[joint_id]->orientation_;

  Eigen::VectorXd external_force = external_force_matrix * link_data_[joint_id]->external_force_;

//  if (joint_id == 17 || joint_id == 18)
//  {
//    ROS_INFO("----- ID : %d -----", joint_id);
//    PRINT_MAT(link_data_[joint_id]->external_force_);
//    PRINT_MAT(external_force_matrix);
//    PRINT_MAT(external_force);
//  }

  /* ---------- */

  Eigen::VectorXd force_torque_child = calcInverseDynamics(link_data_[joint_id]->child_);

  spatial_force = force_torque_default + force_torque_child - external_force;

  Eigen::MatrixXd joint_torque =
      link_data_[joint_id]->spatial_velocity_.transpose() * spatial_force.block<3,1>(0,0) +
      link_data_[joint_id]->spatial_angular_velocity_.transpose() * spatial_force.block<3,1>(3,0);

  link_data_[joint_id]->joint_torque_ = joint_torque.coeff(0,0);


  Eigen::VectorXd force_torque_sibling = calcInverseDynamics(link_data_[joint_id]->sibling_);
  spatial_force = spatial_force + force_torque_sibling;

  return spatial_force;
}

Eigen::VectorXd KinematicsDynamics::calcInverseDynamicsFloatingBase(int joint_id)
{
  Eigen::VectorXd spatial_force;// = Eigen::MatrixXd::Zero(6,1);
  spatial_force.resize(6);
  spatial_force.fill(0);
  if (joint_id == -1)
    return spatial_force;

  Eigen::Vector3d center_of_mass =
      link_data_[joint_id]->orientation_ * link_data_[joint_id]->center_of_mass_ + link_data_[joint_id]->position_;

  Eigen::Matrix3d inertia_tensor =
      link_data_[joint_id]->orientation_ * link_data_[joint_id]->inertia_ * link_data_[joint_id]->orientation_.transpose();

  Eigen::Matrix3d hatto_center_of_mass = robotis_framework::calcHatto(center_of_mass);
  Eigen::Matrix3d inertia_tensor_new = inertia_tensor + link_data_[joint_id]->mass_ * hatto_center_of_mass * hatto_center_of_mass.transpose();

//  if ((link_data_[joint_id]->joint_axis_).norm() == 1.0)
//  {
//    ROS_INFO("----- ID : %d -----", joint_id);
//    PRINT_MAT(inertia_tensor_new);
//  }

  Eigen::Vector3d momentum =
      link_data_[joint_id]->mass_ *
      (link_data_[joint_id]->linear_velocity_ + robotis_framework::calcCross(link_data_[joint_id]->angular_velocity_,center_of_mass));

  Eigen::Vector3d angular_momentum =
      link_data_[joint_id]->mass_ *
      robotis_framework::calcCross(center_of_mass,link_data_[joint_id]->linear_velocity_) +
      inertia_tensor_new * link_data_[joint_id]->angular_velocity_;

  Eigen::Vector3d force_default =
      link_data_[joint_id]->mass_ *
      (link_data_[joint_id]->linear_acceleration_ + robotis_framework::calcCross(link_data_[joint_id]->angular_acceleration_,center_of_mass)) +
      robotis_framework::calcCross(link_data_[joint_id]->angular_velocity_, momentum);

  Eigen::Vector3d torque_default =
      link_data_[joint_id]->mass_ *
      robotis_framework::calcCross(center_of_mass, link_data_[joint_id]->linear_acceleration_) +
      inertia_tensor_new * link_data_[joint_id]->angular_acceleration_ +
      robotis_framework::calcCross(link_data_[joint_id]->linear_velocity_, momentum) +
      robotis_framework::calcCross(link_data_[joint_id]->angular_velocity_, angular_momentum);

  Eigen::VectorXd force_torque_default;// = Eigen::MatrixXd::Zero(6,1);
  force_torque_default.resize(6);
  force_torque_default.block<3,1>(0,0) = force_default;
  force_torque_default.block<3,1>(3,0) = torque_default;


//  if ((link_data_[joint_id]->joint_axis_).norm() == 1.0)
//  {
//    ROS_INFO("----- ID : %d -----", joint_id);
//    PRINT_MAT(force_torque_default);
//  }

  /* ---------- */

  Eigen::MatrixXd external_force_matrix = Eigen::MatrixXd::Identity(6,6);
  external_force_matrix.block<3,3>(0,0) = link_data_[joint_id]->orientation_;
  external_force_matrix.block<3,3>(3,3) = link_data_[joint_id]->orientation_;
  external_force_matrix.block<3,3>(3,0) = robotis_framework::calcHatto(link_data_[joint_id]->position_) * link_data_[joint_id]->orientation_;

//  Eigen::MatrixXd external_torque_matrix = Eigen::MatrixXd::Identity(6,6);
//  external_torque_matrix.block<3,3>(0,0) = link_data_[joint_id]->orientation_;
//  external_torque_matrix.block<3,3>(3,3) = link_data_[joint_id]->orientation_;
//  external_torque_matrix.block<3,3>(3,0) = -1.0 * link_data_[joint_id]->orientation_ * robotis_framework::calcHatto(link_data_[joint_id]->position_);

//  Eigen::MatrixXd force_ex = Eigen::VectorXd::Zero(6,1);
//  force_ex.block<3,1>(0,0) = link_data_[joint_id]->external_force_.block<3,1>(0,0);

//  Eigen::MatrixXd torque_ex = Eigen::VectorXd::Zero(6,1);
//  torque_ex.block<3,1>(3,0) = link_data_[joint_id]->external_force_.block<3,1>(3,0);

  Eigen::VectorXd external_force = external_force_matrix * link_data_[joint_id]->external_force_;

  if (joint_id == 3)
  {
    ROS_INFO("----- ID : %d -----", joint_id);
    PRINT_MAT(link_data_[joint_id]->external_force_);
    PRINT_MAT(external_force_matrix);
    PRINT_MAT(external_force);
  }


//  Eigen::VectorXd external_force = external_force_matrix * force_ex + external_torque_matrix * torque_ex;

  /* ---------- */

//  Eigen::VectorXd external_force = external_force_matrix * link_data_[joint_id]->external_force_;

//  if ((link_data_[joint_id]->joint_axis_).norm() == 1.0)
//  {
//    ROS_INFO("ID : %d", joint_id);
//    PRINT_MAT(link_data_[joint_id]->external_force_);
//    PRINT_MAT(external_force_matrix);
//    PRINT_MAT(external_force);
//  }

  //Eigen::VectorXd external_force = link_data_[joint_id]->external_force_;

  Eigen::VectorXd force_torque_child = calcInverseDynamics(link_data_[joint_id]->child_);

  spatial_force = force_torque_default + force_torque_child - external_force;

//  if ((link_data_[joint_id]->joint_axis_).norm() == 1.0)
//  {
//    ROS_INFO("----- ID : %d -----", joint_id);
//    PRINT_MAT(spatial_force);
//  }

  Eigen::MatrixXd joint_torque =
      link_data_[joint_id]->spatial_velocity_.transpose() * spatial_force.block<3,1>(0,0) +
      link_data_[joint_id]->spatial_angular_velocity_.transpose() * spatial_force.block<3,1>(3,0);

//  if ((link_data_[joint_id]->joint_axis_).norm() == 1.0)
//  {
//    ROS_INFO("ID : %d", joint_id);
//    PRINT_MAT(link_data_[joint_id]->spatial_velocity_.transpose());
//    PRINT_MAT(link_data_[joint_id]->spatial_angular_velocity_.transpose());
//    PRINT_MAT(spatial_force);
//  }

  link_data_[joint_id]->joint_torque_ = joint_torque.coeff(0,0);


  Eigen::VectorXd force_torque_sibling = calcInverseDynamics(link_data_[joint_id]->sibling_);
  spatial_force = spatial_force + force_torque_sibling;

  return spatial_force;
}

}
