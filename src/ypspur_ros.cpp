#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <ypspur_ros/DigitalOutput.h>
#include <ypspur_ros/JointPositionControl.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sys/types.h>
#include <signal.h>
#include <sys/wait.h>

#include <future>
#include <chrono>

namespace YP
{
#include <ypspur.h>
}


bool g_shutdown = false;
void sigint_handler(int sig)
{
	g_shutdown = true;
}

class ypspur_ros_node
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
	bool simulate_control;

	pid_t pid;

	enum
	{
		DIFF,
		NONE
	} mode;
	class joint_params
	{
	public:
		int id;
		std::string name;
		double accel;
		double vel;
		double angle_ref;
		double vel_ref;
		enum
		{
			STOP,
			VELOCITY,
			POSITION
		} control;
	};
	std::vector<joint_params> joints;
	std::map<std::string, int> joint_name_to_num;
	
	class ad_params
	{
	public:
		bool enable;
		std::string name;
		double gain;
		double offset;
	};
	bool digital_input_enable;
	std::vector<ad_params> ads;
	const int ad_num = 8;
	unsigned int dio_output;
	unsigned int dio_dir;
	unsigned int dio_output_default;
	unsigned int dio_dir_default;
	const int dio_num = 8;
	std::map<int, ros::Time> dio_revert;

	trajectory_msgs::JointTrajectory cmd_joint;
	geometry_msgs::Twist cmd_vel;

	void cbCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
	{
		cmd_vel = *msg;
		YP::YPSpur_vel(msg->linear.x, msg->angular.z);
	}
	void cbJoint(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
	{
		cmd_joint = *msg;
		for(auto &cmd: cmd_joint.points)
		{
			if(cmd.velocities.size() < cmd_joint.points.size())
				cmd.velocities.resize(cmd_joint.points.size());
			if(cmd.accelerations.size() < cmd_joint.points.size())
				cmd.accelerations.resize(cmd_joint.points.size());
		}
		for(auto &j: joints)
		{
			j.control = j.POSITION;
		}
	}
	void cbSetVel(const std_msgs::Float32::ConstPtr& msg, int num)
	{
		joints[num].vel = msg->data;
		//printf("set_vel %d %d %f\n", num, joints[num].id, msg->data);
#if !(YPSPUR_JOINT_SUPPORT)
		YP::YP_set_wheel_vel(joints[0].vel, joints[1].vel);
#else
		YP::YP_set_joint_vel(joints[num].id, joints[num].vel);
#endif
	}
	void cbSetAccel(const std_msgs::Float32::ConstPtr& msg, int num)
	{
		joints[num].accel = msg->data;
		//printf("set_accel %d %d %f\n", num, joints[num].id, msg->data);
#if !(YPSPUR_JOINT_SUPPORT)
		YP::YP_set_wheel_accel(joints[0].accel, joints[1].accel);
#else
		YP::YP_set_joint_accel(joints[num].id, joints[num].accel);
#endif
	}
	void cbVel(const std_msgs::Float32::ConstPtr& msg, int num)
	{
		joints[num].vel_ref = msg->data;
		joints[num].control = joint_params::VELOCITY;
		//printf("vel %d %d %f\n", num, joints[num].id, msg->data);
#if !(YPSPUR_JOINT_SUPPORT)
		YP::YP_wheel_vel(joints[0].vel_ref, joints[1].vel_ref);
#else
		YP::YP_joint_vel(joints[num].id, joints[num].vel_ref);
#endif
	}
	void cbAngle(const std_msgs::Float32::ConstPtr& msg, int num)
	{
		joints[num].angle_ref = msg->data;
		joints[num].control = joint_params::POSITION;
#if !(YPSPUR_JOINT_SUPPORT)
		YP::YP_wheel_ang(joints[0].angle_ref, joints[1].angle_ref);
#else
		YP::YP_joint_ang(joints[num].id, joints[num].angle_ref);
#endif
	}
	void cbJointPosition(const ypspur_ros::JointPositionControl::ConstPtr& msg)
	{
		int i = 0;
		for(auto &name: msg->joint_names)
		{
			if(joint_name_to_num.find(name) == joint_name_to_num.end())
			{
				ROS_ERROR("Unknown joint name '%s'", name.c_str());
				continue;
			}
			int num = joint_name_to_num[name];
			joints[num].vel = msg->velocities[i];
			joints[num].accel = msg->accelerations[i];
			joints[num].angle_ref = msg->positions[i];
			joints[num].control = joint_params::POSITION;

			//printf("%s %d %d  %f", name.c_str(), num, joints[num].id, joints[num].angle_ref);
#if (YPSPUR_JOINT_SUPPORT)
			YP::YP_set_joint_vel(joints[num].id, joints[num].vel);
			YP::YP_set_joint_accel(joints[num].id, joints[num].accel);
			YP::YP_joint_ang(joints[num].id, joints[num].angle_ref);
#endif
			i++;
		}
#if !(YPSPUR_JOINT_SUPPORT)
		YP::YP_set_wheel_vel(joints[0].vel, joints[1].vel);
		YP::YP_set_wheel_accel(joints[0].accel, joints[1].accel);
		YP::YP_wheel_ang(joints[0].angle_ref, joints[1].angle_ref);
#endif
	}

	void cbDigitalOutput(const ypspur_ros::DigitalOutput::ConstPtr& msg, int id)
	{
		const auto dio_output_prev = dio_output;
		const auto dio_dir_prev = dio_dir;
		const unsigned int mask = 1 << id;

		switch(msg->output)
		{
		case ypspur_ros::DigitalOutput::HIGH_IMPEDANCE:
			dio_output &= ~mask;
			dio_dir &= ~mask;
			break;
		case ypspur_ros::DigitalOutput::LOW:
			dio_output &= ~mask;
			dio_dir |= mask;
			break;
		case ypspur_ros::DigitalOutput::HIGH:
			dio_output |= mask;
			dio_dir |= mask;
			break;
		case ypspur_ros::DigitalOutput::PULL_UP:
			dio_output |= mask;
			dio_dir &= ~mask;
			break;
		case ypspur_ros::DigitalOutput::PULL_DOWN:
			ROS_ERROR("Digital IO pull down is not supported on this system");
			break;
		}
		if(dio_output != dio_output_prev)
			YP::YP_set_io_data(dio_output);
		if(dio_dir != dio_dir_prev)
			YP::YP_set_io_dir(dio_dir);

		if(msg->toggle_time <= ros::Duration(0))
		{
			dio_revert[id] = ros::Time::now() + msg->toggle_time;
		}
	}
	void revertDigitalOutput(int id)
	{
		const auto dio_output_prev = dio_output;
		const auto dio_dir_prev = dio_dir;
		const unsigned int mask = 1 << id;

		dio_output &= ~mask;
		dio_output |= dio_output_default & mask;
		dio_dir &= ~mask;
		dio_dir |= dio_output_default & mask;

		if(dio_output != dio_output_prev)
			YP::YP_set_io_data(dio_output);
		if(dio_dir != dio_dir_prev)
			YP::YP_set_io_dir(dio_dir);

		dio_revert[id] = ros::Time(0);
	}

public:
	ypspur_ros_node():
		nh("~")
	{
		nh.param("port", port, std::string("/dev/ttyACM0"));
		nh.param("ipc_key", key, 28741);
		nh.param("simulate", simulate, false);
		nh.param("simulate_control", simulate_control, false);
		if(simulate_control) simulate = true;
		nh.param("ypspur_bin", ypspur_bin, std::string("/usr/local/bin/ypspur-coordinator"));
		nh.param("param_file", param_file, std::string(""));
		std::string ad_mask("");
		ads.resize(ad_num);
		for(int i = 0; i < ad_num; i ++)
		{
			nh.param(std::string("ad") + std::to_string(i) + std::string("_enable"),
				   	ads[i].enable, false);
			nh.param(std::string("ad") + std::to_string(i) + std::string("_name"),
				   	ads[i].name, std::string("ad") + std::to_string(i));
			nh.param(std::string("ad") + std::to_string(i) + std::string("_gain"),
				   	ads[i].gain, 1.0);
			nh.param(std::string("ad") + std::to_string(i) + std::string("_offset"),
				   	ads[i].offset, 0.0);
			ad_mask = (ads[i].enable ? std::string("1") : std::string("0")) + ad_mask;
			pubs["ad/" + ads[i].name] = nh.advertise<std_msgs::Float32>("ad/" + ads[i].name, 1);
		}
		nh.param(std::string("digital_input_enable"), digital_input_enable, false);
		dio_output_default = 0;
		dio_dir_default = 0;
		for(int i = 0; i < dio_num; i ++)
		{
			bool output;
			std::string name;
			nh.param(std::string("dio") + std::to_string(i) + std::string("_enable"),
				   	output, false);
			if(output)
			{
				nh.param(std::string("dio") + std::to_string(i) + std::string("_name"),
						name, std::string(std::string("dio") + std::to_string(i)));
				subs[name] = 
					nh.subscribe<ypspur_ros::DigitalOutput>(name, 1, 
							boost::bind(&ypspur_ros_node::cbDigitalOutput, this, _1, i));

				std::string output_default;
				nh.param(std::string("dio") + std::to_string(i) + std::string("_default"),
						output_default, std::string(std::string("dio") + std::to_string(i)));
				if(output_default.compare("HIGH_IMPEDANCE"))
				{
				}
				else if(output_default.compare("LOW"))
				{
					dio_dir_default |= 1 << i;
				}
				else if(output_default.compare("HIGH"))
				{
					dio_dir_default |= 1 << i;
					dio_output_default |= 1 << i;
				}
				else if(output_default.compare("PULL_UP"))
				{
					dio_output_default |= 1 << i;
				}
				else if(output_default.compare("PULL_DOWN"))
				{
					ROS_ERROR("Digital IO pull down is not supported on this system");
				}
			}
		}
		dio_output = dio_output_default;
		dio_dir = dio_dir_default;

		nh.param("odom_id", frames["odom"], std::string("odom"));
		nh.param("base_link_id", frames["base_link"], std::string("base_link"));
		nh.param("origin_id", frames["origin"], std::string(""));
		nh.param("vel", params["vel"], 0.2);
		nh.param("angvel", params["angvel"], 0.4);
		nh.param("acc", params["acc"], 0.4);
		nh.param("angacc", params["angacc"], 0.8);
		nh.param("hz", params["hz"], 200.0);

		std::string mode_name;
		nh.param("odometry_mode", mode_name, std::string("diff"));
		if(mode_name.compare("diff") == 0)
		{
			mode = DIFF;
			pubs["wrench"] = nh.advertise<geometry_msgs::WrenchStamped>("wrench", 1);
			pubs["odom"] = nh.advertise<nav_msgs::Odometry>("odom", 1);
			subs["cmd_vel"] = nh.subscribe("cmd_vel", 1, &ypspur_ros_node::cbCmdVel, this);
		}
		else if(mode_name.compare("none") == 0)
		{
		}
		else
		{
			ROS_ERROR("unknown mode '%s'", mode_name.c_str());
			throw(std::string("unknown mode '") + mode_name + std::string("'"));
		}

		int max_joint_id;
		bool separated_joint;
		nh.param("max_joint_id", max_joint_id, 32);
		nh.param("separated_joint_control", separated_joint, false);
		int num = 0;
		for(int i = 0; i < max_joint_id; i ++)
		{
			std::string name;
			name = std::string("joint") + std::to_string(i);
			if(nh.hasParam(name + std::string("_enable")))
			{
				bool en;
				nh.param(name + std::string("_enable"), en, false);
				if(en)
				{
					joint_params jp;
					jp.id = i;
					nh.param(name + std::string("_name"), jp.name, name);
					nh.param(name + std::string("_accel"), jp.accel, 3.14);
					joint_name_to_num[jp.name] = num;
					joints.push_back(jp);
					//printf("%s %d %d", jp.name.c_str(), jp.id, joint_name_to_num[jp.name]);
					if(separated_joint)
					{
						subs[jp.name + std::string("_setVel")] = 
							nh.subscribe<std_msgs::Float32>(jp.name + std::string("_setVel"), 1, 
									boost::bind(&ypspur_ros_node::cbSetVel, this, _1, num));
						subs[jp.name + std::string("_setAccel")] = 
							nh.subscribe<std_msgs::Float32>(jp.name + std::string("_setAccel"), 1, 
									boost::bind(&ypspur_ros_node::cbSetAccel, this, _1, num));
						subs[jp.name + std::string("_vel")] = 
							nh.subscribe<std_msgs::Float32>(jp.name + std::string("_vel"), 1, 
									boost::bind(&ypspur_ros_node::cbVel, this, _1, num));
						subs[jp.name + std::string("_pos")] = 
							nh.subscribe<std_msgs::Float32>(jp.name + std::string("_pos"), 1, 
									boost::bind(&ypspur_ros_node::cbAngle, this, _1, num));
					}
					subs[std::string("joint_position")] = 
						nh.subscribe<ypspur_ros::JointPositionControl>(
								std::string("joint_position"), 1, 
								&ypspur_ros_node::cbJointPosition, this);
					num ++;
				}
			}
		}
#if !(YPSPUR_JOINT_SUPPORT)
		if(joints.size() != 0)
		{
			if(!(joints.size() == 2 && joints[0].id == 0 && joints[1].id == 1))
			{
				ROS_ERROR("This version of yp-spur only supports [joint0_enable: true, joint1_enable: true]");
				throw(std::string("joint configuration error"));
			}
		}
#endif
		if(joints.size() > 0)
		{
			pubs["joint"] = nh.advertise<sensor_msgs::JointState>("joint", 1);
			subs["joint"] = nh.subscribe("cmd_joint", 1, &ypspur_ros_node::cbJoint, this);
		}

		pid = 0;
		for(int i = 0; i < 2; i ++)
		{
			if(i > 0 || YP::YPSpur_initex(key) < 0)
			{
				ROS_WARN("launching ypspur-coordinator");
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
					for(unsigned int i = 0; i < args.size(); i ++) argv[i] = args[i].c_str();
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
			double test_v, test_w;
			double ret;
			std::atomic<bool> done(false);
			std::thread spur_test = 
				std::thread([&]{
							ret = YP::YPSpur_get_vel(&test_v, &test_w);
							done = true;
						});
			std::chrono::milliseconds span(100);
			std::this_thread::sleep_for(span);
			if(!done)
			{
				// There is no way to kill thread safely in C++11
				// So, just leave it detached.
				spur_test.detach();
				ROS_WARN("ypspur-coordinator seems to be down - launching");
				continue;
			}
			spur_test.join();
			if(ret < 0)
			{
				ROS_WARN("ypspur-coordinator returns error - launching");
				continue;
			}
			ROS_WARN("ypspur-coordinator launched");
			break;
		}

		ROS_INFO("ypspur-coordinator conneceted");
		signal(SIGINT, sigint_handler);

		YP::YPSpur_set_vel(params["vel"]);
		YP::YPSpur_set_accel(params["acc"]);
		YP::YPSpur_set_angvel(params["angvel"]);
		YP::YPSpur_set_angaccel(params["angacc"]);
		YP::YP_set_io_data(dio_output);
		YP::YP_set_io_dir(dio_dir);
	}
	~ypspur_ros_node()
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
		
		odom.pose.pose.position.x = 0;
		odom.pose.pose.position.y = 0;
		odom.pose.pose.position.z = 0;
		odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
		odom.twist.twist.linear.x = 0;
		odom.twist.twist.linear.y = 0;
		odom.twist.twist.angular.z = 0;

		std::map<int, geometry_msgs::TransformStamped> joint_trans;
		sensor_msgs::JointState joint;
		if(joints.size() > 0)
		{
			joint.header.frame_id = std::string("");
			joint.velocity.resize(joints.size());
			joint.position.resize(joints.size());
			joint.effort.resize(joints.size());
			for(auto &j: joints) joint.name.push_back(j.name);

			for(unsigned int i = 0; i < joints.size(); i ++)
			{
				joint_trans[i].header.frame_id = joints[i].name + std::string("_in");
				joint_trans[i].child_frame_id = joints[i].name + std::string("_out");
				joint.velocity[i] = 0;
				joint.position[i] = 0;
				joint.effort[i] = 0;
			}
		}

		ROS_INFO("ypspur_ros main loop started");
		ros::Rate loop(params["hz"]);
		while(!g_shutdown)
		{
			auto now = ros::Time::now();

			if(mode == DIFF)
			{
				double x, y, yaw, v, w;
				double t;

				if(!simulate_control)
				{
					t = YP::YPSpur_get_pos(YP::CS_BS, &x, &y, &yaw);
					if(t == 0.0) t = now.toSec();
					YP::YPSpur_get_vel(&v, &w);
				}
				else
				{
					float dt = 1.0 / params["hz"];
					t = ros::Time::now().toSec();
					v = cmd_vel.linear.x;
					w = cmd_vel.angular.z;
					yaw = tf::getYaw(odom.pose.pose.orientation) + dt * w;
					x = odom.pose.pose.position.x + dt * v * cosf(yaw);
					y = odom.pose.pose.position.y + dt * v * sinf(yaw);
				}

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
				if(t == 0.0) t = now.toSec();
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
			if(joints.size() > 0)
			{
				double t;
				if(!simulate_control)
				{
#if !(YPSPUR_JOINT_SUPPORT)
					double js[2];
					int i;
					t = YP::YP_get_wheel_ang(&js[0], &js[1]);
					i = 0;
					for(auto &j: joints) joint.position[i++] = js[j.id];
					YP::YP_get_wheel_vel(&js[0], &js[1]);
					i = 0;
					for(auto &j: joints) joint.velocity[i++] = js[j.id];
					YP::YP_get_wheel_torque(&js[0], &js[1]);
					i = 0;
					for(auto &j: joints) joint.velocity[i++] = js[j.id];
					if(t == 0.0) t = ros::Time::now().toSec();
#else
					int i = 0;
					t = ros::Time::now().toSec();
					for(auto &j: joints)
					{
						t = YP::YP_get_joint_ang(j.id, &joint.position[i]);
						YP::YP_get_joint_vel(j.id, &joint.velocity[i]);
						//YP::YP_get_joint_torque(j.id, &joint.effort[i]);
						i ++;
					}
#endif
					joint.header.stamp = ros::Time(t);
				}
				else
				{
					float dt = 1.0 / params["hz"];
					t = ros::Time::now().toSec();
					for(unsigned int i = 0; i < joints.size(); i ++)
					{
						auto vel_prev = joint.velocity[i];
						switch(joints[i].control)
						{
						case joint_params::STOP:
							break;
						case joint_params::POSITION:
							{
								float position_err = joints[i].angle_ref - joint.position[i];
								joints[i].vel_ref = sqrtf(2.0 * joints[i].accel * fabs(position_err));
								if(joints[i].vel_ref > joints[i].vel) joints[i].vel_ref = joints[i].vel;
								if(position_err < 0) joints[i].vel_ref = -joints[i].vel_ref;
							}
						case joint_params::VELOCITY:
							joint.velocity[i] = joints[i].vel_ref;
							if(joint.velocity[i] < vel_prev - dt * joints[i].accel)
							{
								joint.velocity[i] = vel_prev - dt * joints[i].accel;
							}
							else if(joint.velocity[i] > vel_prev + dt * joints[i].accel)
							{
								joint.velocity[i] = vel_prev + dt * joints[i].accel;
							}
							joint.position[i] += joint.velocity[i] * dt;
							break;
						}
					}
				}
				pubs["joint"].publish(joint);

				for(unsigned int i = 0; i < joints.size(); i ++)
				{
					joint_trans[i].transform.rotation =
						tf::createQuaternionMsgFromYaw(joint.position[i]);
					joint_trans[i].header.stamp = ros::Time(t);
					tf_broadcaster.sendTransform(joint_trans[i]);
				}

				for(auto &cmd: cmd_joint.points)
				{
					auto t = now - cmd_joint.header.stamp;
					if(cmd.time_from_start < t) continue;

					for(unsigned int i = 0; i < cmd.positions.size(); i ++)
					{
						int jid = joint_name_to_num[cmd_joint.joint_names[i]];
						double ang_err = joint.position[jid] - cmd.positions[i];
						/* Control here */
						ROS_ERROR("Joint trajectory control is not supported yet");
					}
					break;
				}
			}

			for(int i = 0; i < ad_num; i ++)
			{
				if(ads[i].enable)
				{
					std_msgs::Float32 ad;
					ad.data = YP::YP_get_ad_value(i) * ads[i].gain + ads[i].offset;
					pubs["ad/" + ads[i].name].publish(ad);
				}
			}

			for(int i = 0; i < dio_num; i ++)
			{
				if(dio_revert[i] != ros::Time(0))
				{
					if(dio_revert[i] < now)
					{
						revertDigitalOutput(i);
					}
				}
			}

			if(YP::YP_get_error_state())
			{
				ROS_ERROR("ypspur-coordinator is not active");
				break;
			}
			ros::spinOnce();
			loop.sleep();
		}
		ROS_INFO("ypspur_ros main loop terminated");
		ros::shutdown();
		ros::spin();
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
		ypspur_ros_node yr;
		yr.spin();
	}
	catch(std::string e)
	{
	}

	return 0;
}

