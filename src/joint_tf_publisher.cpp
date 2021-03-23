/*
 * Copyright (c) 2015-2017, the ypspur_ros authors
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

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <map>
#include <string>

#include <compatibility.h>

class JointTfPublisherNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::map<std::string, ros::Subscriber> subs_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2::Vector3 z_axis_;

  void cbJoint(const sensor_msgs::JointState::ConstPtr& msg)
  {
    for (size_t i = 0; i < msg->name.size(); i++)
    {
      geometry_msgs::TransformStamped trans;
      trans.header = msg->header;
      trans.header.frame_id = msg->name[i] + "_in";
      trans.child_frame_id = msg->name[i] + "_out";

      trans.transform.rotation = tf2::toMsg(tf2::Quaternion(z_axis_, msg->position[i]));
      tf_broadcaster_.sendTransform(trans);
    }
  }

public:
  JointTfPublisherNode()
    : nh_()
    , pnh_("~")
    , z_axis_(0, 0, 1)
  {
    subs_["joint"] = compat::subscribe(
        nh_, "joint_states",
        pnh_, "joint", 1, &JointTfPublisherNode::cbJoint, this);
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "joint_tf_publisher");

  JointTfPublisherNode jp;
  ros::spin();

  return 0;
}
