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
 *  kinematics_dynamics_define.h
 *
 *  Created on: June 7, 2016
 *      Author: sch
 */

#ifndef THORMANG3_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_DEFINE_H_
#define THORMANG3_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_DEFINE_H_

namespace robotis_op3
{

#define MAX_JOINT_ID    (12)
#define ALL_JOINT_ID    (21)

#define MAX_LEG_ID      (6)
#define MAX_ITER        (5)

#define ID_BODY         (15)

#define ID_R_ARM_START  (0)
#define ID_L_ARM_START  (0)

#define ID_R_LEG_START  (1)
#define ID_L_LEG_START  (2)
#define ID_R_LEG_END    (13)
#define ID_L_LEG_END    (14)

#define ID_R_ANKLE_ROLL (11)
#define ID_L_ANKLE_ROLL (12)

#define ID_PELVIS_X     (16)
#define ID_PELVIS_Y     (17)
#define ID_PELVIS_Z     (18)
#define ID_PELVIS_YAW   (19)
#define ID_PELVIS_PITCH (20)
#define ID_PELVIS_ROLL  (21)

#define ID_BASE         (0)

#define GRAVITY_ACCELERATION (9.81)

}

#endif /* THORMANG3_KINEMATICS_DYNAMICS_KINEMATICS_DYNAMICS_DEFINE_H_ */
