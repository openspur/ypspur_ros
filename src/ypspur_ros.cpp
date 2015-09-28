#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Float32MultiArray.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sys/types.h>
#include <signal.h>
#include <sys/wait.h>


namespace YP
{
#include <ypspur.h>
}


bool g_shutdown = false;
void sigint_handler(int sig)
{
	g_shutdown = true;
}

class ypspur_ros
{
private:
	ros::NodeHandle nh;
	std::map<std::string, ros::Publisher> pubs;
	std::map<std::string, ros::Subscriber> subs;
	tf::TransformListener tf_listener;
	tf::TransformBroadcaster tf_broadcaster;

	std::string port;
	std::string param_file;
	std::string ypspur_bin;
	std::map<std::string, std::string> frames;
	std::map<std::string, double> params;
	int key;
	bool simulate;

	pid_t pid;

	enum
	{
		DIFF,
		JOINT
	} mode;
	class ad_params
	{
	public:
		bool enable;
		double gain;
		double offset;
	};
	bool digital_input_enable;
	std::vector<ad_params> ads;
	const int ad_num = 8;

	void cbCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
	{
		YP::YPSpur_vel(msg->linear.x, msg->angular.z);
	}
	void cbJoint(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg)
	{
		if(msg->positions.size() == 2 && 
				msg->velocities.size() == 2 &&
				msg->accelerations.size() == 2)
		{
			YP::YP_set_wheel_vel(msg->velocities[0], msg->velocities[1]);
			YP::YP_set_wheel_accel(msg->accelerations[0], msg->accelerations[1]);
			YP::YP_wheel_ang(msg->positions[0], msg->positions[1]);
		}
		else if(msg->positions.size() == 1 && 
				msg->velocities.size() == 1 &&
				msg->accelerations.size() == 1)
		{
			YP::YP_set_wheel_vel(msg->velocities[0], 0.0);
			YP::YP_set_wheel_accel(msg->accelerations[0], 0.0);
			YP::YP_wheel_ang(msg->positions[0], 0.0);
		}
		else
		{
			ROS_ERROR("Joint number mismatch");
		}
	}

public:
	ypspur_ros():
		nh("~")
	{
		nh.param("port", port, std::string("/dev/ttyACM0"));
		nh.param("ipc_key", key, 28741);
		nh.param("simulate", simulate, false);
		nh.param("ypspur_bin", ypspur_bin, std::string("/usr/local/bin/ypspur-coordinator"));
		nh.param("param_file", param_file, std::string(""));
		std::string ad_mask("");
		ads.resize(ad_num);
		for(int i = 0; i < ad_num; i ++)
		{
			nh.param(std::string("ad_enable") + std::to_string(i), ads[i].enable, false);
			nh.param(std::string("ad_gain") + std::to_string(i), ads[i].gain, 1.0);
			nh.param(std::string("ad_offset") + std::to_string(i), ads[i].offset, 0.0);
			ad_mask = (ads[i].enable ? std::string("1") : std::string("0")) + ad_mask;
		}
		nh.param(std::string("digital_input_enable"), digital_input_enable, false);
		nh.param("odom_id", frames["odom"], std::string("odom"));
		nh.param("base_link_id", frames["base_link"], std::string("base_link"));
		nh.param("origin_id", frames["origin"], std::string(""));
		nh.param("joint_id_base", frames["joint"], std::string("joint"));
		nh.param("vel", params["vel"], 0.2);
		nh.param("angvel", params["angvel"], 0.4);
		nh.param("acc", params["acc"], 0.4);
		nh.param("angacc", params["angacc"], 0.8);
		nh.param("hz", params["hz"], 200.0);

		std::string mode_name;
		nh.param("mode", mode_name, std::string("diff"));
		if(mode_name.compare("diff") == 0)
		{
			mode = DIFF;
			pubs["wrench"] = nh.advertise<geometry_msgs::WrenchStamped>("wrench", 1);
			pubs["odom"] = nh.advertise<nav_msgs::Odometry>("odom", 1);
			subs["cmd_vel"] = nh.subscribe("cmd_vel", 1, &ypspur_ros::cbCmdVel, this);
		}
		else if(mode_name.compare("joint") == 0)
		{
			mode = JOINT;
			pubs["joint"] = nh.advertise<sensor_msgs::JointState>("joint", 1);
			subs["joint"] = nh.subscribe("cmd_joint", 1, &ypspur_ros::cbJoint, this);
		}
		else
		{
			ROS_ERROR("unknown mode '%s'", mode_name.c_str());
			throw(std::string("unknown mode '") + mode_name + std::string("'"));
		}

		pubs["ad"] = nh.advertise<std_msgs::Float32MultiArray>("ad", 1);
		
		pid = 0;
		if(YP::YPSpur_initex(key) < 0)
		{
			pid = fork();
			if(pid == 0)
			{
				setsid();
				std::vector<std::string> args;
				args.push_back(ypspur_bin);
				args.push_back(std::string("-d"));
				args.push_back(port);
				args.push_back(std::string("--admask"));
				args.push_back(ad_mask);
				args.push_back(std::string("--msq-key"));
				args.push_back(std::to_string(key));
				if(digital_input_enable) 
					args.push_back(std::string("--enable-get-digital-io"));
				if(simulate) 
					args.push_back(std::string("--without-device"));
				if(param_file.size() > 0)
				{
					args.push_back(std::string("-p"));
					args.push_back(param_file);
				}

				const char **argv = new const char * [args.size() + 1];
				for(int i = 0; i < args.size(); i ++) argv[i] = args[i].c_str();
				argv[args.size()] = nullptr;

				execv(ypspur_bin.c_str(), (char**)argv);
				ROS_ERROR("failed to start ypspur-coordinator");
				throw(std::string("failed to start ypspur-coordinator"));
			}
			sleep(2);
			if(YP::YPSpur_initex(key) < 0)
			{
				ROS_ERROR( "failed to init libypspur" );
				throw(std::string("failed to init libypspur"));
			}
		}
		ROS_INFO("ypspur-coordinator conneceted");
		signal(SIGINT, sigint_handler);

		YP::YPSpur_set_vel(params["vel"]);
		YP::YPSpur_set_accel(params["acc"]);
		YP::YPSpur_set_angvel(params["angvel"]);
		YP::YPSpur_set_angaccel(params["angacc"]);
	}
	~ypspur_ros()
	{
	}
	void spin()
	{
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.frame_id = frames["odom"];
		odom_trans.child_frame_id = frames["base_link"];

		nav_msgs::Odometry odom;
		geometry_msgs::WrenchStamped wrench;
		odom.header.frame_id = frames["odom"];
		odom.child_frame_id = frames["base_link"];
		wrench.header.frame_id = frames["base_link"];

		geometry_msgs::TransformStamped joint_trans[2];
		sensor_msgs::JointState joint;
		joint.header.frame_id = std::string("");
		joint.name.resize(2);
		joint.velocity.resize(2);
		joint.position.resize(2);
		joint.effort.resize(2);
		joint.name[0] = frames["joint"] + std::string("0");
		joint.name[1] = frames["joint"] + std::string("1");
		joint_trans[0].header.frame_id = frames["joint"] + std::string("0_in");
		joint_trans[0].child_frame_id = frames["joint"] + std::string("0_out");
		joint_trans[1].header.frame_id = frames["joint"] + std::string("1_in");
		joint_trans[1].child_frame_id = frames["joint"] + std::string("1_out");

		std_msgs::Float32MultiArray ad;
		ad.layout.dim.resize(1);
		ad.layout.dim[0].label = std::string("channel");
		ad.layout.dim[0].stride = 1;
		ad.layout.data_offset = 0;

		ROS_INFO("ypspur_ros main loop started");
		ros::Rate loop(params["hz"]);
		while(!g_shutdown)
		{
			double t;

			if(mode == DIFF)
			{
				double x, y, yaw, v, w;

				t = YP::YPSpur_get_pos(YP::CS_BS, &x, &y, &yaw);
				if(t == 0.0) t = ros::Time::now().toSec();
				YP::YPSpur_get_vel(&v, &w);

				odom.header.stamp = ros::Time(t);
				odom.pose.pose.position.x = x;
				odom.pose.pose.position.y = y;
				odom.pose.pose.position.z = 0;
				odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
				odom.twist.twist.linear.x = v;
				odom.twist.twist.linear.y = 0;
				odom.twist.twist.angular.z = w;
				pubs["odom"].publish(odom);

				odom_trans.header.stamp = ros::Time(t);
				odom_trans.transform.translation.x = x;
				odom_trans.transform.translation.y = y;
				odom_trans.transform.translation.z = 0;
				odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(yaw);
				tf_broadcaster.sendTransform(odom_trans);

				t = YP::YPSpur_get_force(&wrench.wrench.force.x, &wrench.wrench.torque.z);
				if(t == 0.0) t = ros::Time::now().toSec();
				wrench.header.stamp = ros::Time(t);
				wrench.wrench.force.y = 0;
				wrench.wrench.force.z = 0;
				wrench.wrench.torque.x = 0;
				wrench.wrench.torque.y = 0;
				pubs["wrench"].publish(wrench);
				
				if(frames["origin"].length() > 0)
				{
					try{
						tf::StampedTransform transform;
						tf_listener.lookupTransform(
								frames["origin"], frames["base_link"], 
								ros::Time(0), transform);

						tfScalar yaw, pitch, roll;
						transform.getBasis().getEulerYPR(yaw, pitch, roll);
						YP::YPSpur_adjust_pos(YP::CS_GL, transform.getOrigin().x(),
								transform.getOrigin().y(),
								yaw);
					}
					catch (tf::TransformException ex)
					{
						ROS_ERROR("Failed to feedback localization result to YP-Spur (%s)",ex.what());
					}
				}
			}
			if(mode == JOINT)
			{
				t = YP::YP_get_wheel_ang(&joint.position[0], &joint.position[1]);
				if(t == 0.0) t = ros::Time::now().toSec();
				YP::YP_get_wheel_vel(&joint.velocity[0], &joint.velocity[1]);
				YP::YP_get_wheel_torque(&joint.effort[0], &joint.effort[1]);
				joint.header.stamp = ros::Time(t);
				pubs["joint"].publish(joint);

				joint_trans[0].transform.rotation =
				   	tf::createQuaternionMsgFromYaw(joint.position[0]);
				joint_trans[0].header.stamp = ros::Time(t);
				tf_broadcaster.sendTransform(joint_trans[0]);

				joint_trans[1].transform.rotation =
				   	tf::createQuaternionMsgFromYaw(joint.position[1]);
				joint_trans[1].header.stamp = ros::Time(t);
				tf_broadcaster.sendTransform(joint_trans[1]);
			}

			ad.data.clear();
			for(int i = 0; i < 8; i ++)
			{
				if(ads[i].enable)
				{
					float val;
					val = YP::YP_get_ad_value(i) * ads[i].gain + ads[i].offset;
					ad.data.push_back(val);
				}
			}
			ad.layout.dim[0].size = ad.data.size();
			pubs["ad"].publish(ad);

			if(YP::YP_get_error_state())
			{
				ROS_ERROR("ypspur-coordinator is not active");
				break;
			}
			ros::spinOnce();
			loop.sleep();
		}
		ROS_INFO("ypspur_ros main loop terminated");
		if(pid > 0)
		{
			ROS_INFO("killing ypspur-coordinator (%d)", (int)pid);
			kill(pid, SIGINT);
			sleep(2);
		}
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "ypspur_ros");

	try
	{
		ypspur_ros yr;
		yr.spin();
	}
	catch(std::string e)
	{
	}

	return 0;
}

