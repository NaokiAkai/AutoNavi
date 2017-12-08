#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "amcl.h"
#include "ndt.h"

AMCL* amcl;
NDT* ndt;
tf::TransformListener* tf_listener;
ros::Publisher mean_points_pub, grid_lines_pub, ellipses_pub;

void *map_data_publisher(void* obj)
{
	visualization_msgs::Marker grid_lines = ndt->get_grid_lines_of_ndt_map(amcl->map_frame);
	// main loop
	ros::Rate loop_rate(1.0);
	while (ros::ok())
	{
		sensor_msgs::PointCloud mean_points = ndt->get_mean_points_of_ndt_map(amcl->map_frame, 50.0f, amcl->robot_pose.x, amcl->robot_pose.y);
		visualization_msgs::MarkerArray ellipses = ndt->get_ellipses_of_ndt_map(amcl->map_frame, 50.0f, amcl->robot_pose.x, amcl->robot_pose.y);
		mean_points_pub.publish(mean_points);
		grid_lines_pub.publish(grid_lines);
		ellipses_pub.publish(ellipses);
		loop_rate.sleep();
	}
}

void evaluate_particles_using_ndt_map(sensor_msgs::LaserScan scan)
{
	double z_hit = 0.95;
	double max_dist_prob = 0.043937;
	double z_hit_denom = 0.08;
	double z_rand = 0.05;
	double z_rand_mult = 0.033333;
	double max;
	for (int i = 0; i < amcl->particle_num; i++)
	{
		double w = 0.0;
		double c = cos(amcl->particles[i].pose.yaw);
		double s = sin(amcl->particles[i].pose.yaw);
		double xo = amcl->base_link2laser.x * c - amcl->base_link2laser.y * s + amcl->particles[i].pose.x;
		double yo = amcl->base_link2laser.x * s + amcl->base_link2laser.y * c + amcl->particles[i].pose.y;
		for (int j = 0; j < scan.ranges.size(); j += amcl->scan_step)
		{
			if (!amcl->is_valid_scan_points[j])
			{
				double pz = z_hit * max_dist_prob + z_rand * z_rand_mult;
				if (pz < 0.0)	pz = 0.0;
				if (pz > 1.0)	pz = 1.0;
				w += log(pz);
				continue;
			}
			double angle = scan.angle_min + scan.angle_increment * (double)j;
			double yaw = angle + amcl->particles[i].pose.yaw;
			double r = scan.ranges[j];
			float x = (float)(r * cos(yaw) + xo);
			float y = (float)(r * sin(yaw) + yo);
			double pz = 0.0;
			double z = ndt->compute_probability(x, y);
			if (z > 0.0)
				pz += z_hit * z;
			pz += z_rand * z_rand_mult;
			if (pz < 0.0)	pz = 0.0;
			if (pz > 1.0)	pz = 1.0;
			w += log(pz);
		}
		double weight = exp(w);
		amcl->particles[i].w *= weight;
		if (i == 0)
		{
			amcl->max_particle_likelihood_num = i;
			max = amcl->particles[i].w;
		}
		else
		{
			if (max < amcl->particles[i].w)
			{
				amcl->max_particle_likelihood_num = i;
				max = amcl->particles[i].w;
			}
		}
	}
}

void mapping(sensor_msgs::LaserScan scan)
{
	static bool is_first = true;
	static double prev_xo, prev_yo, prev_yawo;
	static tf::StampedTransform map2laser;
	try
	{
		tf_listener->waitForTransform(amcl->map_frame, scan.header.frame_id, scan.header.stamp, ros::Duration(1.0));
		tf_listener->lookupTransform(amcl->map_frame, scan.header.frame_id, scan.header.stamp, map2laser);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
		return;
	}
	double xo = map2laser.getOrigin().x();
	double yo = map2laser.getOrigin().y();
	tf::Quaternion q(map2laser.getRotation().x(), map2laser.getRotation().y(), map2laser.getRotation().z(), map2laser.getRotation().w());
	double rollo, pitcho, yawo;
	tf::Matrix3x3 m(q);
	m.getRPY(rollo, pitcho, yawo);
	bool do_mapping = false;
	if (is_first)
	{
		do_mapping = true;
		is_first = false;
	}	
	else
	{
		double dx = xo - prev_xo;
		double dy = yo - prev_yo;
		double dl = sqrt(dx * dx + dy * dy);
		double dyaw = yawo - prev_yawo;
		if (dyaw < -M_PI)	dyaw += 2.0 * M_PI;
		if (dyaw > M_PI)	dyaw -= 2.0 * M_PI;
//		if (dl >= mapping_interval_dist || fabs(dyaw) > mapping_interval_angle)
		if (dl >= 0.2f || fabs(dyaw) > 1.0f * M_PI / 180.0f)
			do_mapping = true;
	}
	if (do_mapping)
	{
		// decrease occupancy rate
		for (int i = 0; i < scan.ranges.size(); i++)
		{
			float r = scan.ranges[i];
			if (r < scan.range_min || scan.range_max < r)
				continue;
			float yaw = (float)yawo + scan.angle_min + scan.angle_increment * (float)i;
			float dx = ndt->ndt_map_grid_size * cos(yaw);
			float dy = ndt->ndt_map_grid_size * sin(yaw);
			float x = (float)xo;
			float y = (float)yo;
			float dr = ndt->ndt_map_grid_size;
			for (float l = 0.0f; l < r - dr; l += dr)
			{
				ndt->decrease_occupancy_rate_of_ndt_map(x, y, r - l);
				x += dx;
				y += dy;
			}
		}
		// add points to the ndt map
		for (int i = 0; i < scan.ranges.size(); i++)
		{
			float r = scan.ranges[i];
			if (r < scan.range_min || scan.range_max < r)
				continue;
			float yaw = (float)yawo + scan.angle_min + scan.angle_increment * (float)i;
			float x = r * cos(yaw) + (float)xo;
			float y = r * sin(yaw) + (float)yo;
			ndt->add_point_to_ndt_map(x, y);
		}
		prev_xo = xo;
		prev_yo = yo;
		prev_yawo = yawo;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "amcl");
	ros::NodeHandle nh("~");
	// read parameters
	std::string map_file_name = "/tmp/ndt_map_aaa.txt";
	int min_points_num = 20;
	double occupancy_rate_threshold = 0.65f;
	bool mapping_mode = false;
	nh.param("/amcl/map_file_name", map_file_name, map_file_name);
	nh.param("/amcl/min_points_num", min_points_num, min_points_num);
	nh.param("/amcl/occupancy_rate_threshold", occupancy_rate_threshold, occupancy_rate_threshold);
	nh.param("/amcl/mapping_mode", mapping_mode, mapping_mode);
	// publisher
	mean_points_pub = nh.advertise<sensor_msgs::PointCloud>("/ndt_map_mean_points", 10);
	grid_lines_pub = nh.advertise<visualization_msgs::Marker>("/ndt_map_grid_lines", 10);
	ellipses_pub = nh.advertise<visualization_msgs::MarkerArray>("/ndt_map_ellipses", 10);
	// initialization
	amcl = new AMCL;
	ndt = new NDT;
	tf_listener = new tf::TransformListener;
	bool is_map_data = true;
	if (!ndt->read_ndt_map(map_file_name))
		is_map_data = false;
	if (!is_map_data)
		ndt->init_ndt_map(700.0f, 600.0f, 1.0f, -200.0f, -200.0f);
	ndt->set_min_points_num(min_points_num);
	ndt->set_occupancy_rate_threshold(occupancy_rate_threshold);
	// map data publisher will be parallely executed
	pthread_t tid;
	pthread_create(&tid, NULL, map_data_publisher, NULL);
	// main loop
	ros::Rate loop_rate(amcl->pose_publish_hz);
	double prev_time = 0.0;
	while (ros::ok())
	{
		ros::spinOnce();
		// perform localization
		amcl->update_particle_pose_by_odom();
		if (!amcl->is_scan_data || !amcl->is_tf_initialized)
		{
			ROS_ERROR("initialization is not finished yet");
			continue;
		}
		sensor_msgs::LaserScan scan = amcl->curr_scan;
		double curr_time = scan.header.stamp.toSec();;
		amcl->check_scan_points_validity(scan);
//		amcl->evaluate_particles(scan);
		evaluate_particles_using_ndt_map(scan);
		amcl->compute_total_weight_and_effective_sample_size();
		amcl->estimate_robot_pose();
		amcl->compute_random_particle_rate();
		double d_time = curr_time - prev_time;
		bool do_mapping = false;
		if (fabs(amcl->delta_dist) >= amcl->update_dist || fabs(amcl->delta_yaw) >= amcl->update_yaw || d_time >= amcl->update_time)
		{
			amcl->resample_particles();
			prev_time = curr_time;
			amcl->delta_dist = amcl->delta_yaw = 0.0;
			prev_time = curr_time;
			do_mapping = true;
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
		if (mapping_mode && do_mapping)
			mapping(scan);
		loop_rate.sleep();
	}
	return 0;
}
