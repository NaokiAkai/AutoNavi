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
	max_dist_to_obstacle(0.5),
	alpha_slow(0.0001),
	alpha_fast(0.1),
	delta_dist(0.0),
	delta_yaw(0.0),
	update_dist(0.2),
	update_yaw(2.0),
	update_time(5.0),
	odom_noise_dist_dist(1.0),
	odom_noise_dist_head(0.7),
	odom_noise_head_dist(0.7),
	odom_noise_head_head(1.0),
	start_x(0.0),
	start_y(0.0),
	start_yaw(0.0),
	initial_cov_xx(0.5),
	initial_cov_yy(0.5),
	initial_cov_yawyaw(3.0),
	pose_publish_hz(20.0),
	use_kld_sampling(false),
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
	nh.param("/amcl/max_dist_to_obstacle", max_dist_to_obstacle, max_dist_to_obstacle);
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
	nh.param("/amcl/use_kld_sampling", use_kld_sampling, use_kld_sampling);
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

double AMCL::nrand(double n)
{
	return (n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX));
}

void AMCL::xy2uv(double x, double y, int* u, int* v)
{
	double dx = x - map.info.origin.position.x;
	double dy = y - map.info.origin.position.y;
	*u = (int)(dx / map.info.resolution);
	*v = (int)(dy / map.info.resolution);
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
	// initilizatioin of tf to know relative position of base link and laser
	tf::TransformListener tf_listener;
	tf::StampedTransform tf_base_link2laser;
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		try
		{
			ros::Time now = ros::Time::now();
			tf_listener.waitForTransform(base_link_frame, laser_frame, now, ros::Duration(1.0));
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
		// build distance map from a target point to the nearest obstacle (occupied cell)
		int dr = (int)(max_dist_to_obstacle / map.info.resolution);
		dist_map.resize(map.info.width);
		for (int u = 0; u < map.info.width; u++)
			dist_map[u].resize(map.info.height);
		for (int u = 0; u < map.info.width; u++)
		{
			for (int v = 0; v < map.info.height; v++)
			{
				float min_d;
				bool is_first = true;
				for (int uu = u - dr; uu <= u + dr; uu++)
				{
					for (int vv = v - dr; vv <= v + dr; vv++)
					{
						if (0 <= uu && uu < map.info.width && 0 <= vv && vv < map.info.height)
						{
							int node = vv * map.info.width + uu;
							if (map.data[node] == 100)
							{
								if (is_first)
								{
									float du = (float)(uu - u);
									float dv = (float)(vv - v);
									min_d = sqrt(du * du + dv * dv);
									is_first = false;
								}
								else
								{
									float du = (float)(uu - u);
									float dv = (float)(vv - v);
									float d = sqrt(du * du + dv * dv);
									if (d < min_d)
										min_d = d;
								}
							}
						}
					}
				}
				if (!is_first && min_d < max_dist_to_obstacle)
					dist_map[u][v] = min_d * map.info.resolution;
				else
					dist_map[u][v] = max_dist_to_obstacle;
			}
		}
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
	double d_dist2 = d_dist * d_dist;
	double d_yaw2 = d_yaw * d_yaw;
	delta_dist += d_dist;
	delta_yaw += d_yaw;
	robot_pose.x += d_dist * cos(robot_pose.yaw);
	robot_pose.y += d_dist * sin(robot_pose.yaw);
	robot_pose.yaw += d_yaw;
	if (robot_pose.yaw < -M_PI)	robot_pose.yaw += 2.0 * M_PI;
	if (robot_pose.yaw > M_PI)	robot_pose.yaw -= 2.0 * M_PI;
	for (int i = 0; i < particle_num; i++)
	{
		double dd = d_dist + nrand(d_dist2 * odom_noise_dist_dist + d_yaw2 * odom_noise_head_dist);
		double dy = d_yaw + nrand(d_dist2 * odom_noise_dist_head + d_yaw2 * odom_noise_head_head);
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
			{
				double pz = z_hit * max_dist_prob + z_rand * z_rand_mult;
				if (pz < 0.0)	pz = 0.0;
				if (pz > 1.0)	pz = 1.0;
				w += log(pz);
				continue;
			}
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
				double z = (double)dist_map[u][v];
				double p = exp(-(z * z) / z_hit_denom);
				pz += z_hit * p;
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
	std::vector<particle_t> tmp_particles = particles;
	if (use_kld_sampling)
	{
		printf("kld sampling\n");
		double pose_reso = 0.1, angle_reso = 0.2 * M_PI / 180.0;
		double pose_range = 2.0, angle_range = 5.0 * M_PI / 180.0;
		int pose_size = (int)(pose_range / pose_reso);
		int angle_size = (int)(angle_range / angle_reso);
		std::vector<bool> is_empty;
		is_empty.resize(pose_size * pose_size * angle_size, true);
		int k = 0, m = 0;
		double m_chi = 0.0, epsilon = 0.05, z = 0.83891;
		// determined value of z from this pdf -> http://math.arizona.edu/~rsims/ma464/standardnormaltable.pdf
		do
		{
			darts = (double)rand() / ((double)RAND_MAX + 1.0);
			for (int i = 0; i < particle_num; i++)
			{
				if (darts < wb[i])
				{
					particles[m] = tmp_particles[i];
					particles[m].w = wo;
					double dx = particles[m].pose.x - robot_pose.x;
					double dy = particles[m].pose.y - robot_pose.y;
					double dyaw = particles[m].pose.yaw - robot_pose.yaw;
					if (dyaw < -M_PI)	dyaw += 2.0 * M_PI;
					if (dyaw > M_PI)	dyaw -= 2.0 * M_PI;
					int u = (int)(dx / pose_reso) + pose_size / 2;
					int v = (int)(dy / pose_reso) + pose_size / 2;
					int w = (int)(dyaw / angle_reso) + angle_size / 2;
					int node = w * pose_size * pose_size + v * pose_size + u;
					bool fall_empty = true;
					if (0 <= u && u < pose_size && 0 <= v && v < pose_size && 0 <= w && w < angle_size)
					{
						if (!is_empty[node])
							fall_empty = false;
					}
					if (fall_empty)
					{
						k++;
						if (0 <= u && u < pose_size && 0 <= v && v < pose_size && 0 <= w && w < angle_size)
							is_empty[node] = false;
						if (k > 1)
						{
							double m1 = (double)(k - 1) / (2.0 * epsilon);
							double m2 = 1.0 - 2.0 / (9.0 * (double)(k - 1)) + sqrt(2.0 / (9.0 * (double)(k - 1))) * z;
							m_chi = m1 * pow(m2, 3.0);
						}
					}
					m++;
					i = particle_num;
				}
			}
			if (m >= max_particle_num)
				break;
		}
		while (m < (int)m_chi || m < min_particle_num);
		particle_num = m;
	}
	else
	{
		for (int i = 0; i < particle_num; i++)
		{
			darts = (double)rand() / ((double)RAND_MAX + 1.0);
			for (int j = 0; j < particle_num; j++)
			{
				if (darts < wb[j])
				{
					particles[i] = tmp_particles[j];
					particles[i].w = wo;
					break;
				}
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
