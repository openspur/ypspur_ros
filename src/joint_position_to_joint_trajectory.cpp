#include <ros/ros.h>

#include <ypspur_ros/JointPositionControl.h>
#include <trajectory_msgs/JointTrajectory.h>

class convert
{
private:
	ros::Subscriber sub_joint;
	ros::Publisher pub_joint;

	double accel;

	void cbJointPosition(const ypspur_ros::JointPositionControl::ConstPtr& msg)
	{
		trajectory_msgs::JointTrajectory cmd;
		cmd.header = msg->header;
		cmd.joint_names = msg->joint_names;
		cmd.points.resize(1);
		cmd.points[0].velocities.resize(msg->positions.size());
		cmd.points[0].positions = msg->positions;
		cmd.points[0].accelerations.resize(msg->positions.size());

		float t_max = 0;
		int i = 0;
		for(auto &p: msg->positions)
		{
			float t_acc = msg->velocities[i] / accel;
			float t = fabs(p) / msg->velocities[i] + t_acc; // Approx.
			if(t_max < t) t_max = t;

			i ++;
		}
		cmd.points[0].time_from_start = ros::Duration(t_max);

		pub_joint.publish(cmd);
	}

public:
	convert()
	{
		ros::NodeHandle nh("~");
		sub_joint = nh.subscribe( std::string("joint_position"), 5, 
				&convert::cbJointPosition, this);
		pub_joint = nh.advertise<trajectory_msgs::JointTrajectory>(
				std::string("joint_trajectory"), 5, false);

		nh.param("accel", accel, 0.3);
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "joint_position_to_joint_trajectory");

	convert conv;
	ros::spin();

	return 0;
}


