/*
 * Copyright (c) 2016-2017, the mcl_3dl authors
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

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ypspur_ros/JointPositionControl.h>

#include <map>
#include <string>

class convert
{
private:
  ros::Subscriber sub_joint;
  ros::Subscriber sub_joint_state;
  ros::Publisher pub_joint;

  std::map<std::string, double> state;

  double accel;
  bool skip_same;

  void cbJointState(const sensor_msgs::JointState::ConstPtr& msg)
  {
    for (size_t i = 0; i < msg->name.size(); i++)
    {
      state[msg->name[i]] = msg->position[i];
    }
  }
  ypspur_ros::JointPositionControl cmd_prev;
  void cbJointPosition(const ypspur_ros::JointPositionControl::ConstPtr& msg)
  {
    while (true)
    {
      if (msg->joint_names.size() != cmd_prev.joint_names.size())
        break;
      {
        bool eq = true;
        for (unsigned int i = 0; i < msg->joint_names.size(); i++)
        {
          if (msg->joint_names[i].compare(cmd_prev.joint_names[i]) != 0)
            eq = false;
        }
        if (!eq)
          break;
      }
      if (msg->positions.size() != cmd_prev.positions.size())
        break;
      {
        bool eq = true;
        for (unsigned int i = 0; i < msg->positions.size(); i++)
        {
          if (msg->positions[i] != cmd_prev.positions[i])
            eq = false;
        }
        if (!eq)
          break;
      }
      if (msg->velocities.size() != cmd_prev.velocities.size())
        break;
      {
        bool eq = true;
        for (unsigned int i = 0; i < msg->velocities.size(); i++)
        {
          if (msg->velocities[i] != cmd_prev.velocities[i])
            eq = false;
        }
        if (!eq)
          break;
      }
      if (msg->accelerations.size() != cmd_prev.accelerations.size())
        break;
      {
        bool eq = true;
        for (unsigned int i = 0; i < msg->accelerations.size(); i++)
        {
          if (msg->accelerations[i] != cmd_prev.accelerations[i])
            eq = false;
        }
        if (!eq)
          break;
      }
      return;
    }
    cmd_prev = *msg;

    trajectory_msgs::JointTrajectory cmd;
    cmd.header = msg->header;
    cmd.joint_names = msg->joint_names;
    cmd.points.resize(1);
    cmd.points[0].velocities.resize(msg->positions.size());
    cmd.points[0].positions = msg->positions;
    cmd.points[0].accelerations.resize(msg->positions.size());

    float t_max = 0;
    int i = 0;
    for (auto& p : msg->positions)
    {
      float t = fabs(p - state[msg->joint_names[i]]) / msg->velocities[i];
      if (t_max < t)
        t_max = t;

      i++;
    }
    cmd.points[0].time_from_start = ros::Duration(t_max);

    pub_joint.publish(cmd);
  }

public:
  convert()
  {
    ros::NodeHandle nh("~");
    sub_joint = nh.subscribe(std::string("joint_position"), 5,
                             &convert::cbJointPosition, this);
    sub_joint_state = nh.subscribe(std::string("joint"), 5,
                                   &convert::cbJointState, this);
    pub_joint = nh.advertise<trajectory_msgs::JointTrajectory>(
        std::string("joint_trajectory"), 5, false);

    nh.param("accel", accel, 0.3);
    nh.param("skip_same", skip_same, true);
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "joint_position_to_joint_trajectory");

  convert conv;
  ros::spin();

  return 0;
}
