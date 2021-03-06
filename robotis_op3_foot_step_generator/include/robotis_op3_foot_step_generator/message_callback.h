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

#ifndef ROBOTIS_OP3_FOOT_STEP_GENERATOR_MESSAGE_CALLBACK_H_
#define ROBOTIS_OP3_FOOT_STEP_GENERATOR_MESSAGE_CALLBACK_H_


#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include "robotis_op3_motion_module_msgs/FootStepCommand.h"
#include "robotis_op3_motion_module_msgs/Step2DArray.h"

#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_op3_motion_module_msgs/RobotPose.h"
#include "robotis_op3_motion_module_msgs/GetReferenceStepData.h"
#include "robotis_op3_motion_module_msgs/AddStepDataArray.h"
#include "robotis_op3_motion_module_msgs/StartWalking.h"
#include "robotis_op3_motion_module_msgs/IsRunning.h"
#include "robotis_op3_motion_module_msgs/RemoveExistingStepData.h"

#include "robotis_foot_step_generator.h"


void initialize(void);

void walkingModuleStatusMSGCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg);

void footStepCommandCallback(const robotis_op3_motion_module_msgs::FootStepCommand::ConstPtr& msg);
void step2DArrayCallback(const robotis_op3_motion_module_msgs::Step2DArray::ConstPtr& msg);

bool isRunning(void);


#endif /* ROBOTIS_OP3_FOOT_STEP_GENERATOR_MESSAGE_CALLBACK_H_ */
