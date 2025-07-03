/*
 * Copyright (c) 2025, the ypspur_ros authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef YPSPUR_ROS_DIRECT_YPSPUR_H
#define YPSPUR_ROS_DIRECT_YPSPUR_H

#include <functional>

namespace YP
{
#include <ypspur/ypspur-coordinator.h>
}

namespace direct_ypspur
{
// Hook
void registerOdometryHook(const std::function<void(const YP::OdometryPtr, const YP::ErrorStatePtr)> fn);
void unregisterOdometryHook();

/*
// TODO(at-wat): implementing

// System
int YP_get_parameter(int param_id, double* value);

// PWS Control
int YPSpur_adjust_pos(int cs, double x, double y, double theta);
int YPSpur_free(void);
int YPSpur_set_accel(double v);
int YPSpur_set_angaccel(double w);
int YPSpur_set_angvel(double w);
int YPSpur_set_vel(double v);
int YPSpur_vel(double v, double w);
int YP_openfree(void);

// Joint Control
int YP_joint_ang(int id, double a);
int YP_joint_ang_vel(int id, double a, double v);
int YP_joint_vel(int id, double v);
int YP_set_joint_accel(int id, double a);
int YP_set_joint_vel(int id, double v);

// AUX
int YP_set_io_data(unsigned char data);
int YP_set_io_dir(unsigned char dir);
*/
}  // namespace direct_ypspur

#endif  // YPSPUR_ROS_DIRECT_YPSPUR_H
