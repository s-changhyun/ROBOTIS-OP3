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
 * link_data.h
 *
 *  Created on: June 7, 2016
 *      Author: sch
 */

#ifndef ROBOTIS_OP3_KINEMATICS_DYNAMICS_LINK_DATA_H_
#define ROBOTIS_OP3_KINEMATICS_DYNAMICS_LINK_DATA_H_

#include "robotis_math/robotis_math.h"

namespace robotis_op3
{

class LinkData
{
public:

  LinkData();
  ~LinkData();

  std::string name_;

  int parent_;
  int sibling_;
  int child_;

  double mass_;

  Eigen::Vector3d relative_position_;
  Eigen::Vector3d joint_axis_;
  Eigen::Vector3d center_of_mass_;
  Eigen::Matrix3d inertia_;

  double joint_limit_max_;
  double joint_limit_min_;

  double joint_angle_;
  double joint_velocity_;
  double joint_acceleration_;

  Eigen::Vector3d joint_center_of_mass_;

  Eigen::Vector3d position_;
  Eigen::Matrix3d orientation_;

  Eigen::Vector3d linear_velocity_;
  Eigen::Vector3d angular_velocity_;

  Eigen::Vector3d linear_acceleration_;
  Eigen::Vector3d angular_acceleration_;

  // joint torque
  double joint_torque_;
  Eigen::Vector3d spatial_velocity_;
  Eigen::Vector3d spatial_angular_velocity_;
  Eigen::VectorXd external_force_;

  Eigen::Matrix4d transformation_;
};

}

#endif /* ROBOTIS_OP3_KINEMATICS_DYNAMICS_LINK_DATA_H_ */
