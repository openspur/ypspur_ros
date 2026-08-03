/*
 * Copyright (c) 2026, the ypspur_ros authors
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

#include <string>

#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <gtest/gtest.h>

TEST(JointIndexCheck, ErrorOnIndexSignalLoss)
{
  ros::WallDuration wait(0.05);

  ros::NodeHandle nh;
  ros::Publisher pub_cmd =
      nh.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 1, true);

  sensor_msgs::JointState::ConstPtr joint_states;
  const boost::function<void(const sensor_msgs::JointState::ConstPtr&)> cb_joint =
      [&joint_states](const sensor_msgs::JointState::ConstPtr& msg) -> void
  {
    joint_states = msg;
  };
  ros::Subscriber sub_joint_states =
      nh.subscribe("joint_states", 100, cb_joint);

  diagnostic_msgs::DiagnosticStatus::Ptr diag;
  const boost::function<void(const diagnostic_msgs::DiagnosticArray::ConstPtr&)> cb_diag =
      [&diag](const diagnostic_msgs::DiagnosticArray::ConstPtr& msg) -> void
  {
    for (const diagnostic_msgs::DiagnosticStatus& status : msg->status)
    {
      if (status.name == "YP-Spur Motor Controller")
      {
        diag.reset(new diagnostic_msgs::DiagnosticStatus(status));
      }
    }
  };
  ros::Subscriber sub_diag =
      nh.subscribe("/diagnostics", 100, cb_diag);

  // Wait until ypspur_ros
  for (int i = 0; i < 20 * 30; ++i)
  {
    wait.sleep();
    ros::spinOnce();
    if (joint_states && diag)
      break;
  }
  ASSERT_TRUE(static_cast<bool>(joint_states));
  ASSERT_TRUE(static_cast<bool>(diag));

  // Index signal error must not be reported while the joint is stopped
  for (int i = 0; i < 40; ++i)
  {
    wait.sleep();
    ros::spinOnce();
    ASSERT_NE(diag->level, diagnostic_msgs::DiagnosticStatus::ERROR)
        << "Index signal error must not be reported while stopped: "
        << diag->message;
  }

  // Rotate the joint continuously
  // (the joint keeps vel_end_ after the trajectory)
  trajectory_msgs::JointTrajectory cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.joint_names.resize(1);
  cmd.joint_names[0] = "joint0";
  cmd.points.resize(1);
  cmd.points[0].time_from_start = ros::Duration(1);
  cmd.points[0].positions.resize(1);
  cmd.points[0].positions[0] = 2.0;
  cmd.points[0].velocities.resize(1);
  cmd.points[0].velocities[0] = 2.0;
  pub_cmd.publish(cmd);

  // joint0_index_check_travel is expected to be exceeded in about 2 seconds
  bool error_detected = false;
  for (int i = 0; i < 20 * 10; ++i)
  {
    wait.sleep();
    ros::spinOnce();
    if (diag->level == diagnostic_msgs::DiagnosticStatus::ERROR)
    {
      error_detected = true;
      break;
    }
  }
  ASSERT_TRUE(error_detected)
      << "Index signal loss must be reported as diagnostic error";
  ASSERT_NE(diag->message.find("Joint index signal is not detected"), std::string::npos)
      << "Unexpected message: " << diag->message;

  bool key_found = false;
  for (const diagnostic_msgs::KeyValue& value : diag->values)
  {
    if (value.key == "joint_index_signal_error")
    {
      key_found = true;
      ASSERT_EQ(value.value, "joint0");
    }
  }
  ASSERT_TRUE(key_found);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_joint_index_check");

  return RUN_ALL_TESTS();
}
