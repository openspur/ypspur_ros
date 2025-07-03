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

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>

#include <gtest/gtest.h>

TEST(CmdVel, Control)
{
  ros::WallDuration wait(0.05);
  ros::NodeHandle nh;
  ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

  nav_msgs::Odometry::ConstPtr odom;
  const boost::function<void(const nav_msgs::Odometry::ConstPtr&)> cb_odom =
      [&odom](const nav_msgs::Odometry::ConstPtr& msg) -> void
  {
    odom = msg;
  };
  ros::Subscriber sub_odom = nh.subscribe("odom", 1, cb_odom);

  // Wait ypspur_ros
  for (int i = 0; i < 20 * 30; ++i)
  {
    wait.sleep();
    ros::spinOnce();
    if (odom)
      break;
  }
  ASSERT_TRUE(static_cast<bool>(odom));

  ASSERT_NEAR(odom->twist.twist.linear.x, 0.0, 1e-6);
  ASSERT_NEAR(odom->twist.twist.angular.z, 0.0, 1e-6);
  ASSERT_NEAR(odom->pose.pose.position.x, 0.0, 1e-6);
  ASSERT_NEAR(odom->pose.pose.position.y, 0.0, 1e-6);
  ASSERT_NEAR(tf2::getYaw(odom->pose.pose.orientation), 0.0, 1e-6);

  // Go forward
  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.5;
  pub_cmd.publish(cmd);

  ros::Duration(1).sleep();
  ros::spinOnce();

  ASSERT_NEAR(odom->twist.twist.linear.x, 0.5, 1e-6);
  ASSERT_NEAR(odom->twist.twist.angular.z, 0.0, 1e-6);

  // Stop
  cmd.linear.x = 0;
  pub_cmd.publish(cmd);

  ros::Duration(1).sleep();
  ros::spinOnce();

  ASSERT_NEAR(odom->twist.twist.linear.x, 0.0, 1e-6);
  ASSERT_NEAR(odom->twist.twist.angular.z, 0.0, 1e-6);
  ASSERT_NEAR(odom->pose.pose.position.x, 0.5, 1e-2);
  ASSERT_NEAR(odom->pose.pose.position.y, 0.0, 1e-2);
  ASSERT_NEAR(tf2::getYaw(odom->pose.pose.orientation), 0.0, 1e-2);

  // Rotate
  cmd.angular.z = 0.5;
  pub_cmd.publish(cmd);

  ros::Duration(1).sleep();
  ros::spinOnce();

  ASSERT_NEAR(odom->twist.twist.linear.x, 0.0, 1e-6);
  ASSERT_NEAR(odom->twist.twist.angular.z, 0.5, 1e-6);

  // Stop
  cmd.angular.z = 0;
  pub_cmd.publish(cmd);

  ros::Duration(1).sleep();
  ros::spinOnce();

  ASSERT_NEAR(odom->twist.twist.linear.x, 0.0, 1e-6);
  ASSERT_NEAR(odom->twist.twist.angular.z, 0.0, 1e-6);
  ASSERT_NEAR(odom->pose.pose.position.x, 0.5, 1e-2);
  ASSERT_NEAR(odom->pose.pose.position.y, 0.0, 1e-2);
  ASSERT_NEAR(tf2::getYaw(odom->pose.pose.orientation), 0.5, 1e-2);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_joint_trajectory");

  return RUN_ALL_TESTS();
}
