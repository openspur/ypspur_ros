#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/transform_broadcaster.h>

#include <sys/types.h>
#include <signal.h>
#include <sys/wait.h>


class joint_tf_publisher_node
{
private:
	ros::NodeHandle nh;
	std::map<std::string, ros::Publisher> pubs;
	std::map<std::string, ros::Subscriber> subs;
	tf::TransformBroadcaster tf_broadcaster;

	void cbJoint(const sensor_msgs::JointState::ConstPtr& msg)
	{
		for(size_t i = 0; i < msg->name.size(); i ++)
		{
			geometry_msgs::TransformStamped trans;
			trans.header = msg->header;
			trans.header.frame_id = msg->name[i] + "_in";
			trans.child_frame_id = msg->name[i] + "_out";
			trans.transform.rotation = tf::createQuaternionMsgFromYaw(msg->position[i]);
			tf_broadcaster.sendTransform(trans);
		}
	}

public:
	joint_tf_publisher_node():
		nh("~")
	{
		subs["joint"] = nh.subscribe("joint", 1, &joint_tf_publisher_node::cbJoint, this);
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "joint_tf_publisher");

	joint_tf_publisher_node jp;
	ros::spin();

	return 0;
}

