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

AMCL::AMCL():
	nh("~"),
	map_frame("/world"),
	laser_frame("/laser"),
	base_link_frame("/base_link"),
	input_map_topic_name("/amcl_map"),
	input_odom_topic_name("/odom"),
	input_scan_topic_name("/scan"),
	min_particle_num(100),
	max_particle_num(1000),
	resample_threshold(0.5),
	scan_step(20),
	alpha_slow(0.0001),
	alpha_fast(0.1),
	delta_dist(0.0),
	delta_yaw(0.0),
	update_dist(0.2),
	update_yaw(2.0),
	update_time(5.0),
	odom_noise_dist_dist(0.6),
	odom_noise_dist_head(0.03),
	odom_noise_head_dist(0.03),
	odom_noise_head_head(0.6),
	start_x(0.0),
	start_y(0.0),
	start_yaw(0.0),
	initial_cov_xx(0.5),
	initial_cov_yy(0.5),
	initial_cov_yawyaw(3.0),
	pose_publish_hz(20.0),
	is_map_data(false),
	is_scan_data(false),
	is_first_time(true),
	is_tf_initialized(false)
{
	// read parameters
	nh.param("/amcl/map_frame", map_frame, map_frame);
	nh.param("/amcl/laser_frame", laser_frame, laser_frame);
	nh.param("/amcl/base_link_frame", base_link_frame, base_link_frame);
	nh.param("/amcl/input_map_topic_name", input_map_topic_name, input_map_topic_name);
	nh.param("/amcl/input_odom_topic_name", input_odom_topic_name, input_odom_topic_name);
	nh.param("/amcl/input_scan_topic_name", input_scan_topic_name, input_scan_topic_name);
	nh.param("/amcl/min_particle_num", min_particle_num, min_particle_num);
	nh.param("/amcl/max_particle_num", max_particle_num, max_particle_num);
	nh.param("/amcl/resample_threshold", resample_threshold, resample_threshold);
	nh.param("/amcl/scan_step", scan_step, scan_step);
	nh.param("/amcl/alpha_slow", alpha_slow, alpha_slow);
	nh.param("/amcl/alpha_fast", alpha_fast, alpha_fast);
	nh.param("/amcl/update_dist", update_dist, update_dist);
	nh.param("/amcl/update_yaw", update_yaw, update_yaw);
	nh.param("/amcl/update_time", update_time, update_time);
	nh.param("/amcl/odom_noise_dist_dist", odom_noise_dist_dist, odom_noise_dist_dist);
	nh.param("/amcl/odom_noise_dist_head", odom_noise_dist_head, odom_noise_dist_head);
	nh.param("/amcl/odom_noise_head_dist", odom_noise_head_dist, odom_noise_head_dist);
	nh.param("/amcl/odom_noise_head_head", odom_noise_head_head, odom_noise_head_head);
	nh.param("/amcl/start_x", start_x, start_x);
	nh.param("/amcl/start_y", start_y, start_y);
	nh.param("/amcl/start_yaw", start_yaw, start_yaw);
	nh.param("/amcl/initial_cov_xx", initial_cov_xx, initial_cov_xx);
	nh.param("/amcl/initial_cov_yy", initial_cov_yy, initial_cov_yy);
	nh.param("/amcl/initial_cov_yawyaw", initial_cov_yawyaw, initial_cov_yawyaw);
	nh.param("/amcl/pose_publish_hz", pose_publish_hz, pose_publish_hz);
	// check values
	if (resample_threshold < 0.0 || 1.0 < resample_threshold)
	{
		ROS_ERROR("resample_threshold must be included from 0 to 1 (0.5 is recommended)");
		exit(1);
	}
	// convert degree to radian
	update_yaw *= M_PI / 180.0;
	start_yaw *= M_PI / 180.0;
	initial_cov_yawyaw *= M_PI / 180.0;
	// set initial state
	robot_pose.x = start_x;
	robot_pose.y = start_y;
	robot_pose.yaw = start_yaw;
	particle_num = min_particle_num;
	// subscriber
	pose_sub = nh.subscribe("/initialpose", 1, &AMCL::initial_pose_callback, this);
	map_sub = nh.subscribe(input_map_topic_name, 1, &AMCL::map_callback, this);
	odom_sub = nh.subscribe(input_odom_topic_name, 100, &AMCL::odom_callback, this);
	scan_sub = nh.subscribe(input_scan_topic_name, 1, &AMCL::scan_callback, this);
	// publisher
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/amcl_pose", 1);
	particles_pub = nh.advertise<geometry_msgs::PoseArray>("/amcl_particles", 1);
	// initialization
	amcl_init();
}

void AMCL::reset_particles(void)
{
	double wo = 1.0 / (double)particle_num;
	for (int i = 0; i < particle_num; i++)
	{
		particles[i].pose.x = robot_pose.x + nrand(initial_cov_xx);
		particles[i].pose.y = robot_pose.y + nrand(initial_cov_yy);
		particles[i].pose.yaw = robot_pose.yaw + nrand(initial_cov_yawyaw);
		particles[i].w = wo;
	}
	w_slow = w_fast = 0.0;
}

void AMCL::amcl_init(void)
{
	// initilizatioin of pf
	particles.resize(max_particle_num);
	reset_particles();
	// initilizatioin of tf
	tf::TransformListener tf_listener;
	tf::StampedTransform tf_base_link2laser;
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		try
		{
			ros::Time now = ros::Time::now();
			tf_listener.waitForTransform(base_link_frame, laser_frame, now, ros::Duration(1));
			tf_listener.lookupTransform(base_link_frame, laser_frame, now, tf_base_link2laser);
			break;
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			loop_rate.sleep();
		}
	}
	tf::Point zero_point(0.0, 0.0, 0.0);
	tf::Point laser_point_in_base_link_frame = tf_base_link2laser * zero_point;
	tf::Quaternion q(tf_base_link2laser.getRotation().x(),
		tf_base_link2laser.getRotation().y(),
		tf_base_link2laser.getRotation().z(),
		tf_base_link2laser.getRotation().w());
	double roll, pitch, yaw;
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);
	base_link2laser.x = laser_point_in_base_link_frame.getX();
	base_link2laser.y = laser_point_in_base_link_frame.getY();
	base_link2laser.yaw = yaw;
	is_tf_initialized = true;
}

void AMCL::initial_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	tf::Quaternion q(msg->pose.pose.orientation.x, 
		msg->pose.pose.orientation.y, 
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);
	robot_pose.x = msg->pose.pose.position.x;
	robot_pose.y = msg->pose.pose.position.y;
	robot_pose.yaw = yaw;
	reset_particles();
	is_first_time = true;
}

void AMCL::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	if (!is_map_data)
	{
		map = *msg;
		cv::Mat occ_map(map.info.height, map.info.width, CV_8UC1);
		for (int v = 0; v < occ_map.rows; v++)
		{
			for (int u = 0; u < occ_map.cols; u++)
			{
				int node = v * map.info.width + u;
				int val = map.data[node];
				if (val == 100)
					occ_map.at<uchar>(v, u) = 0;
				else
					occ_map.at<uchar>(v, u) = 1;
			}
		}
		cv::Mat dist(map.info.height, map.info.width, CV_32FC1);
		cv::distanceTransform(occ_map, dist, CV_DIST_L2, 5);
		for (int v = 0; v < occ_map.rows; v++)
		{
			for (int u = 0; u < occ_map.cols; u++)
			{
				float d = dist.at<float>(v, u) * map.info.resolution;
				dist.at<float>(v, u) = d;
			}
		}
		dist_map = dist;
		is_map_data = true;
	}
}

void AMCL::broadcast_tf(void)
{
	tf::Transform tf;
	tf::Quaternion q;
	static tf::TransformBroadcaster br;
	tf.setOrigin(tf::Vector3(robot_pose.x, robot_pose.y, 0.0));
	q.setRPY(0.0, 0.0, robot_pose.yaw);
	tf.setRotation(q);
	br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), map_frame, "/amcl_frame"));
}

void AMCL::publish_pose(void)
{
	geometry_msgs::PoseStamped pose;
	tf::Transform tf;
	tf::Quaternion q;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = map_frame;
	tf.setOrigin(tf::Vector3(robot_pose.x, robot_pose.y, 0.0));
	q.setRPY(0.0, 0.0, robot_pose.yaw);
	tf.setRotation(q);
	pose.pose.position.x = tf.getOrigin().x();
	pose.pose.position.y = tf.getOrigin().y();
	pose.pose.position.z = tf.getOrigin().z();
	pose.pose.orientation.x = tf.getRotation().x();
	pose.pose.orientation.y = tf.getRotation().y();
	pose.pose.orientation.z = tf.getRotation().z();
	pose.pose.orientation.w = tf.getRotation().w();
	pose_pub.publish(pose);
}

void AMCL::publish_particles(void)
{
	geometry_msgs::Pose pose;
	geometry_msgs::PoseArray poses;
	tf::Transform tf;
	tf::Quaternion q;
	poses.header.stamp = ros::Time::now();
	poses.header.frame_id = map_frame;
	for (int i = 0; i < particle_num; i++)
	{
		tf.setOrigin(tf::Vector3(particles[i].pose.x, particles[i].pose.y, 0.0));
		q.setRPY(0.0, 0.0, particles[i].pose.yaw);
		tf.setRotation(q);
		pose.position.x = tf.getOrigin().x();
		pose.position.y = tf.getOrigin().y();
		pose.position.z = tf.getOrigin().z();
		pose.orientation.x = tf.getRotation().x();
		pose.orientation.y = tf.getRotation().y();
		pose.orientation.z = tf.getRotation().z();
		pose.orientation.w = tf.getRotation().w();
		poses.poses.push_back(pose);
	}
	particles_pub.publish(poses);
}

void AMCL::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	curr_odom = *msg;
}

void AMCL::update_particle_pose_by_odom(void)
{
	// update robot and particle pose based on odometry
	static double prev_time;
	nav_msgs::Odometry odom = curr_odom;
	if (is_first_time)
	{
		prev_time = odom.header.stamp.toSec();
		is_first_time = false;
		return;
	}
	double curr_time = odom.header.stamp.toSec();
	double d_time = curr_time - prev_time;
	if (d_time > 1.0)
	{
		prev_time = curr_time;
		return;
	}
	double d_dist = odom.twist.twist.linear.x * d_time;
	double d_yaw = odom.twist.twist.angular.z * d_time;
	delta_dist += d_dist;
	delta_yaw += d_yaw;
	robot_pose.x += d_dist * cos(robot_pose.yaw);
	robot_pose.y += d_dist * sin(robot_pose.yaw);
	robot_pose.yaw += d_yaw;
	if (robot_pose.yaw < -M_PI)	robot_pose.yaw += 2.0 * M_PI;
	if (robot_pose.yaw > M_PI)	robot_pose.yaw -= 2.0 * M_PI;
	for (int i = 0; i < particle_num; i++)
	{
		double dd = d_dist + nrand(d_dist * odom_noise_dist_dist + d_yaw * odom_noise_head_dist);
		double dy = d_yaw + nrand(d_dist * odom_noise_dist_head + d_yaw * odom_noise_head_head);
		particles[i].pose.x += dd * cos(particles[i].pose.yaw);
		particles[i].pose.y += dd * sin(particles[i].pose.yaw);
		particles[i].pose.yaw += dy;
		while (particles[i].pose.yaw < -M_PI)	particles[i].pose.yaw += 2.0 * M_PI;
		while (particles[i].pose.yaw > M_PI)	particles[i].pose.yaw -= 2.0 * M_PI;
	}
	prev_time = curr_time;
}

void AMCL::check_scan_points_validity(sensor_msgs::LaserScan scan)
{
	for (int i = 0; i < scan.ranges.size(); i++)
	{
		double r = scan.ranges[i];
		if (r < scan.range_min || scan.range_max < r)
			is_valid_scan_points[i] = false;
		else
			is_valid_scan_points[i] = true;
	}
}

void AMCL::evaluate_particles(sensor_msgs::LaserScan scan)
{
	double z_hit = 0.95;
	double max_dist_prob = 0.043937;
	double z_hit_denom = 0.08;
	double z_rand = 0.05;
	double z_rand_mult = 0.033333;
	double max;
	for (int i = 0; i < particle_num; i++)
	{
		double w = 0.0;
		double c = cos(particles[i].pose.yaw);
		double s = sin(particles[i].pose.yaw);
		double xo = base_link2laser.x * c - base_link2laser.y * s + particles[i].pose.x;
		double yo = base_link2laser.x * s + base_link2laser.y * c + particles[i].pose.y;
		for (int j = 0; j < scan.ranges.size(); j += scan_step)
		{
			if (!is_valid_scan_points[j])
				continue;
			double angle = scan.angle_min + scan.angle_increment * (double)j;
			double yaw = angle + particles[i].pose.yaw;
			double r = scan.ranges[j];
			double x = r * cos(yaw) + xo;
			double y = r * sin(yaw) + yo;
			int u, v;
			xy2uv(x, y, &u, &v);
			double pz = 0.0;
			if (0 <= u && u < map.info.width && 0 <= v && v < map.info.height)
			{
				int node = v * map.info.width + u;
				double z = dist_map.at<float>(v, u);
				double zz = exp(-(z * z) / z_hit_denom);
				pz += z_hit * zz;
			}
			else
			{
				pz += z_hit * max_dist_prob;
			}
			pz += z_rand * z_rand_mult;
			if (pz < 0.0)	pz = 0.0;
			if (pz > 1.0)	pz = 1.0;
			w += log(pz);
		}
		double weight = exp(w);
		particles[i].w *= weight;
		if (i == 0)
		{
			max_particle_likelihood_num = i;
			max = particles[i].w;
		}
		else
		{
			if (max < particles[i].w)
			{
				max_particle_likelihood_num = i;
				max = particles[i].w;
			}
		}
	}
}

void AMCL::compute_total_weight_and_effective_sample_size(void)
{
	double wo = 1.0 / (double)particle_num;
	total_weight = 0.0;
	for (int i = 0; i < particle_num; i++)
		total_weight += particles[i].w;
	double sum = 0.0;
	for (int i = 0; i < particle_num; i++)
	{
		double w = particles[i].w / total_weight;
		if (isnan(w) != 0)	w = wo;
		particles[i].w = w;
		sum += w * w;
	}
	effective_sample_size = 1.0 / sum;
}

void AMCL::compute_random_particle_rate(void)
{
	w_avg = total_weight / (double)particle_num;
	if (w_slow == 0.0)
		w_slow = w_avg;
	else
		w_slow = w_slow + alpha_slow * (w_avg - w_slow);
	if (w_fast == 0.0)
		w_fast = w_avg;
	else
		w_fast = w_fast + alpha_fast * (w_avg - w_fast);
	double r = 1.0 - w_fast / w_slow;
	// reset weight history to avoid spiraling off into complete randomness
	if (r > 0.0)
		w_fast = w_slow = 0.0;
	random_particle_rate = r;
}

void AMCL::estimate_robot_pose(void)
{
	double tmp_yaw = robot_pose.yaw;
	pose_t pose;
	pose.x = pose.y = pose.yaw = 0.0;
	for (int i = 0; i < particle_num; i++)
	{
		pose.x += particles[i].pose.x * particles[i].w;
		pose.y += particles[i].pose.y * particles[i].w;
		double dyaw = tmp_yaw - particles[i].pose.yaw;
		while (dyaw < -M_PI)	dyaw += 2.0 * M_PI;
		while (dyaw > M_PI)		dyaw -= 2.0 * M_PI;
		pose.yaw += dyaw * particles[i].w;
	}
	pose.yaw = tmp_yaw - pose.yaw;
	while (pose.yaw < -M_PI)	pose.yaw += 2.0 * M_PI;
	while (pose.yaw > M_PI)		pose.yaw -= 2.0 * M_PI;
	robot_pose = pose;
}

void AMCL::resample_particles(void)
{
	if (effective_sample_size > (double)particle_num * resample_threshold)
		return;
	double darts, wo, wb[particle_num];
	wo = 1.0 / (double)particle_num;
	wb[0] = particles[0].w;
	for (int i = 1; i < particle_num; i++)
		wb[i] = particles[i].w + wb[i - 1];
	for (int i = 0; i < particle_num; i++)
	{
		darts = (double)rand() / ((double)RAND_MAX + 1.0);
		for (int j = 0; j < particle_num; j++)
		{
			if (darts < wb[j])
			{
				particles[i] = particles[j];
				particles[i].w = wo;
				break;
			}
		}
	}
}

void AMCL::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	curr_scan = *msg;
	if (!is_scan_data)
	{
		is_valid_scan_points.resize(msg->ranges.size());
		is_scan_data = true;
	}
}
