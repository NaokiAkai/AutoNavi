#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "common.h"

class TrajectoryFollower
{
private:
	ros::NodeHandle nh;
	ros::Subscriber pose_sub;
	std::string map_frame, base_link_frame;
	std::string path_file_name;
	std::string output_twist_topic_name;
	double look_ahead_dist, max_vel, kv;
	double publish_hz;
	int nearest_path_index, prev_nearest_path_index;

public:
	TrajectoryFollower();
	void initial_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
};

TrajectoryFollower::TrajectoryFollower():
	nh("~"),
	map_frame("/world"),
	base_link_frame("/base_link"),
	path_file_name("/tmp/simple_trajectory.txt"),
	output_twist_topic_name("/twist_cmd"),
	look_ahead_dist(1.0),
	max_vel(1.0),
	kv(0.6),
	publish_hz(20.0),
	prev_nearest_path_index(-1)
{
	// read parameters
	nh.param("/trajectory_follower/map_frame", map_frame, map_frame);
	nh.param("/trajectory_follower/base_link_frame", base_link_frame, base_link_frame);
	nh.param("/trajectory_follower/path_file_name", path_file_name, path_file_name);
	nh.param("/trajectory_follower/output_twist_topic_name", output_twist_topic_name, output_twist_topic_name);
	nh.param("/trajectory_follower/look_ahead_dist", look_ahead_dist, look_ahead_dist);
	nh.param("/trajectory_follower/max_vel", max_vel, max_vel);
	nh.param("/trajectory_follower/kv", kv, kv);
	nh.param("/trajectory_follower/publish_hz", publish_hz, publish_hz);
	// subscriber
	pose_sub = nh.subscribe("/initialpose", 1, &TrajectoryFollower::initial_pose_callback, this);
	// publisher
	ros::Publisher twist_pub = nh.advertise<geometry_msgs::TwistStamped>(output_twist_topic_name, 100);
	ros::Rate loop_rate(publish_hz);
	// load path
	std::vector<point_t> path;
	FILE* fp = fopen(path_file_name.c_str(), "r");
	if (fp == NULL)
	{
		ROS_ERROR("cannot open path file, %s", path_file_name.c_str());
		exit(1);
	}
	point_t p;
	while (fscanf(fp, "%lf %lf", &p.x, &p.y) != EOF)
		path.push_back(p);
	fclose(fp);
	// start trajectory tracking
	tf::TransformListener tf_listener;
	tf::StampedTransform map2base_link;
	double eo = 0.0;
	geometry_msgs::TwistStamped twist_cmd;
	while (ros::ok())
	{
		// read robot pose (base_link_frame) from tf tree in map_frame
		ros::Time now = ros::Time::now();
		try
		{
			tf_listener.waitForTransform(map_frame, base_link_frame, now, ros::Duration(1.0));
			tf_listener.lookupTransform(map_frame, base_link_frame, now, map2base_link);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			continue;
		}
		// x, y, and yaw mean 2d robot pose
		double x = map2base_link.getOrigin().x();
		double y = map2base_link.getOrigin().y();
		tf::Quaternion q(map2base_link.getRotation().x(),
			map2base_link.getRotation().y(),
			map2base_link.getRotation().z(),
			map2base_link.getRotation().w());
		double roll, pitch, yaw;
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);
		// search the nearest path point
		double min_dist;
		int i0, i1;
		if (prev_nearest_path_index < 0)
		{
			i0 = 1;
			i1 = path.size();
		}
		else
		{
			i0 = prev_nearest_path_index - 30;
			i1 = prev_nearest_path_index + 30;
			if (i0 < 1)
				i0 = 1;
			if (i1 >= path.size())
				i1 = path.size();
		}
		for (int i = i0; i < i1; i++)
		{
			double dx = path[i].x - x;
			double dy = path[i].y - y;
			double dl = sqrt(dx * dx + dy * dy);
			if (i == i0)
			{
				nearest_path_index = i;
				min_dist = dl;
			}
			else if (dl < min_dist)
			{
				nearest_path_index = i;
				min_dist = dl;
			}
		}
		prev_nearest_path_index = nearest_path_index;
		// search the target path point
		int target_path_index = -1;
		for (int i = nearest_path_index; i < path.size(); i++)
		{
			double dx = path[i].x - x;
			double dy = path[i].y - y;
			double dl = sqrt(dx * dx + dy * dy);
			if (dl >= look_ahead_dist)
			{
				target_path_index = i;
				break;
			}
		}
		if (target_path_index < 0) // reach at the goal point
			break;
		// compute deviation from the path and determine twist_cmd based on PD control
		double dx = path[target_path_index].x - path[target_path_index - 1].x;
		double dy = path[target_path_index].y - path[target_path_index - 1].y;
		double th = atan2(dy, dx);
		double x2p = dx * cos(th) + dy * sin(th);
		dx = path[target_path_index].x - x;
		dy = path[target_path_index].y - y;
		double xrp = dx * cos(th) + dy * sin(th);
		double yrp = -dx * sin(th) + dy * cos(th);
		double t = atan2(dy, dx);
		double e = t - yaw;
		if (e < -M_PI)	e += 2.0 * M_PI;
		if (e > M_PI)	e -= 2.0 * M_PI;
		twist_cmd.header.stamp = ros::Time::now();
		twist_cmd.twist.linear.x = max_vel - kv * fabs(e);
		twist_cmd.twist.angular.z = 0.4 * e + 0.01 * eo;
		eo = e;
		twist_pub.publish(twist_cmd);
		// ros spin
		ros::spinOnce();
		// print debug message
		printf("robot pose: x = %.3lf [m], y = %.3lf [m], yaw = %.3lf [deg]\n", x, y, yaw * 180.0 / M_PI);
		printf("target path point: x = %.3lf [m], y = %.3lf [m]\n", path[target_path_index].x, path[target_path_index].y);
		printf("target path index = %d, path point num = %d\n", target_path_index, (int)path.size());
		printf("twist command: vel = %.3lf [m/sec], ang_vel = %.3lf [rad/sec]\n", twist_cmd.twist.linear.x, twist_cmd.twist.angular.z);
		printf("\n");
		loop_rate.sleep();
	}
	// publish stop command
	for (int i = 0; i < 50; i++)
	{
		twist_cmd.header.stamp = ros::Time::now();
		twist_cmd.twist.linear.x = 0.0;
		twist_cmd.twist.angular.z = 0.0;
		twist_pub.publish(twist_cmd);
		ros::spinOnce();
		usleep(100000);
	}
}

void TrajectoryFollower::initial_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	prev_nearest_path_index = -1;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "trajectory_follower");
	TrajectoryFollower node;
	return 0;
}
