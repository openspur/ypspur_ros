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

#include <functional>

#include <ypspur_ros/direct_ypspur.h>

namespace YP
{
#include <ypspur/ypparam.h>
#include <ypspur/ypspur-coordinator.h>
}  // namespace YP

namespace
{
std::function<void(const YP::OdometryPtr, const YP::ErrorStatePtr)> g_odometry_hook;
}

extern "C" void odometry_hook_wrapper(const YP::OdometryPtr odom, const YP::ErrorStatePtr err)
{
  if (g_odometry_hook)
  {
    g_odometry_hook(odom, err);
  }
}

namespace direct_ypspur
{
void registerOdometryHook(const std::function<void(const YP::OdometryPtr, const YP::ErrorStatePtr)> fn)
{
  g_odometry_hook = fn;
  YP::ypsc_set_odometry_hook(odometry_hook_wrapper);
}

void unregisterOdometryHook()
{
  YP::ypsc_set_odometry_hook(nullptr);
  g_odometry_hook = nullptr;
}

double YP_get_parameter(const int param_id)
{
  YP::YPSpur_msg cmd, res;
  cmd.msg_type = YPSPUR_MSG_CMD;
  cmd.type = YP::YPSPUR_PARAM_GET;
  cmd.cs = param_id;
  YP::ypsc_command(&cmd, &res);
  return res.data[0];
}

void YPSpur_adjust_pos(const int cs, const double x, const double y, const double theta)
{
  YP::YPSpur_msg cmd, res;
  cmd.msg_type = YPSPUR_MSG_CMD;
  cmd.type = YP::YPSPUR_ADJUST;
  cmd.data[0] = x;
  cmd.data[1] = y;
  cmd.data[2] = theta;
  cmd.cs = cs;
  YP::ypsc_command(&cmd, &res);
}

void YPSpur_free()
{
  YP::YPSpur_msg cmd, res;
  cmd.msg_type = YPSPUR_MSG_CMD;
  cmd.type = YP::YPSPUR_FREE;
  YP::ypsc_command(&cmd, &res);
}

void YPSpur_set_accel(const double dv)
{
  YP::YPSpur_msg cmd, res;
  cmd.msg_type = YPSPUR_MSG_CMD;
  cmd.type = YP::YPSPUR_SET_ACCEL;
  cmd.data[0] = dv;
  YP::ypsc_command(&cmd, &res);
}

void YPSpur_set_angaccel(const double dw)
{
  YP::YPSpur_msg cmd, res;
  cmd.msg_type = YPSPUR_MSG_CMD;
  cmd.type = YP::YPSPUR_SET_ANGACCEL;
  cmd.data[0] = dw;
  YP::ypsc_command(&cmd, &res);
}

void YPSpur_set_angvel(const double w)
{
  YP::YPSpur_msg cmd, res;
  cmd.msg_type = YPSPUR_MSG_CMD;
  cmd.type = YP::YPSPUR_SET_ANGVEL;
  cmd.data[0] = w;
  YP::ypsc_command(&cmd, &res);
}

void YPSpur_set_vel(const double v)
{
  YP::YPSpur_msg cmd, res;
  cmd.msg_type = YPSPUR_MSG_CMD;
  cmd.type = YP::YPSPUR_SET_VEL;
  cmd.data[0] = v;
  YP::ypsc_command(&cmd, &res);
}

void YPSpur_vel(const double v, const double w)
{
  YP::YPSpur_msg cmd, res;
  cmd.msg_type = YPSPUR_MSG_CMD;
  cmd.type = YP::YPSPUR_VEL;
  cmd.data[0] = v;
  cmd.data[1] = w;
  YP::ypsc_command(&cmd, &res);
}

void YP_openfree()
{
  YP::YPSpur_msg cmd, res;
  cmd.msg_type = YPSPUR_MSG_CMD;
  cmd.type = YP::YPSPUR_OPENFREE;
  YP::ypsc_command(&cmd, &res);
}

void YP_joint_ang(const int id, const double a)
{
  YP::YPSpur_msg cmd, res;
  cmd.msg_type = YPSPUR_MSG_CMD;
  cmd.type = YP::YPSPUR_JOINT_ANG;
  cmd.cs = id;
  cmd.data[0] = a;
  YP::ypsc_command(&cmd, &res);
}

void YP_joint_ang_vel(const int id, const double a, const double v)
{
  YP::YPSpur_msg cmd, res;
  cmd.msg_type = YPSPUR_MSG_CMD;
  cmd.type = YP::YPSPUR_JOINT_ANG_VEL;
  cmd.cs = id;
  cmd.data[0] = a;
  cmd.data[1] = v;
  YP::ypsc_command(&cmd, &res);
}

void YP_joint_vel(const int id, const double v)
{
  YP::YPSpur_msg cmd, res;
  cmd.msg_type = YPSPUR_MSG_CMD;
  cmd.type = YP::YPSPUR_JOINT_VEL;
  cmd.cs = id;
  cmd.data[0] = v;
  YP::ypsc_command(&cmd, &res);
}

void YP_set_joint_accel(const int id, const double a)
{
  YP::YPSpur_msg cmd, res;
  cmd.msg_type = YPSPUR_MSG_CMD;
  cmd.type = YP::YPSPUR_SET_JOINT_ACCEL;
  cmd.cs = id;
  cmd.data[0] = a;
  YP::ypsc_command(&cmd, &res);
}

void YP_set_joint_vel(const int id, const double v)
{
  YP::YPSpur_msg cmd, res;
  cmd.msg_type = YPSPUR_MSG_CMD;
  cmd.type = YP::YPSPUR_SET_JOINT_VEL;
  cmd.cs = id;
  cmd.data[0] = v;
  YP::ypsc_command(&cmd, &res);
}

void YP_set_io_data(const uint8_t data)
{
  YP::YPSpur_msg cmd, res;
  cmd.msg_type = YPSPUR_MSG_CMD;
  cmd.type = YP::YPSPUR_SETIODATA;
  cmd.data[0] = data;
  YP::ypsc_command(&cmd, &res);
}

void YP_set_io_dir(const uint8_t dir)
{
  YP::YPSpur_msg cmd, res;
  cmd.msg_type = YPSPUR_MSG_CMD;
  cmd.type = YP::YPSPUR_SETIODIR;
  cmd.data[0] = dir;
  YP::ypsc_command(&cmd, &res);
}
}  // namespace direct_ypspur
