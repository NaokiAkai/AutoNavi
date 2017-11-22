#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "amcl.h"

AMCL* amcl;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "amcl");
	amcl = new AMCL;
	ros::Rate loop_rate(amcl->pose_publish_hz);
	bool is_first = true;
	double prev_time = 0.0;
	while (ros::ok())
	{
		ros::spinOnce();
		// perform localization
		amcl->update_particle_pose_by_odom();
		if (!amcl->is_map_data || !amcl->is_scan_data || !amcl->is_tf_initialized)
		{
			ROS_ERROR("initialization is not finished yet");
			continue;
		}
		sensor_msgs::LaserScan scan = amcl->curr_scan;
		double curr_time = scan.header.stamp.toSec();;
		amcl->check_scan_points_validity(scan);
		amcl->evaluate_particles(scan);
		amcl->compute_total_weight_and_effective_sample_size();
		amcl->estimate_robot_pose();
		amcl->compute_random_particle_rate();
		double d_time = curr_time - prev_time;
		if (fabs(amcl->delta_dist) >= amcl->update_dist || fabs(amcl->delta_yaw) >= amcl->update_yaw || d_time >= amcl->update_time)
		{
			amcl->resample_particles();
			prev_time = curr_time;
			amcl->delta_dist = amcl->delta_yaw = 0.0;
			printf("x = %.3lf [m], y = %.3lf [m], yaw = %.3lf [deg]\n", amcl->robot_pose.x, amcl->robot_pose.y, amcl->robot_pose.yaw * 180.0 / M_PI);
			printf("particle_num = %d\n", amcl->particle_num);
			printf("effective_sample_size = %lf, total_weight = %lf\n", amcl->effective_sample_size, amcl->total_weight);
			printf("random_particle_rate = %lf\n", amcl->random_particle_rate);
			printf("\n");
		}
		// publish localization result messages
		amcl->broadcast_tf();
		amcl->publish_pose();
		amcl->publish_particles();
		loop_rate.sleep();
	}
	return 0;
}
