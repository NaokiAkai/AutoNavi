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

#include "common.h"

typedef struct
{
	pose_t pose;
	double w;
} particle_t;

class AMCL
{
private:
	ros::NodeHandle nh;
	std::string map_frame, laser_frame, base_link_frame;
	std::string input_map_topic_name, input_odom_topic_name, input_scan_topic_name;
	int particle_num, min_particle_num, max_particle_num;
	double resample_threshold;
	int scan_step;
	double alpha_slow, alpha_fast;
	double delta_dist, delta_yaw;
	double update_dist, update_yaw, update_time;
	double odom_noise_dist_dist, odom_noise_dist_head, odom_noise_head_dist, odom_noise_head_head;
	double start_x, start_y, start_yaw;
	double initial_cov_xx, initial_cov_yy, initial_cov_yawyaw;
	pose_t robot_pose, base_link2laser;
	std::vector<particle_t> particles;
	nav_msgs::OccupancyGrid map;
	cv::Mat dist_map;
	double effective_sample_size, total_weight, random_particle_rate, w_avg, w_slow, w_fast;
	bool is_map_data, is_first_time, is_tf_initialized;
	ros::Publisher pose_pub, particles_pub;

public:
	AMCL();
	void reset_particles(double xo, double yo, double yawo);
	void amcl_init(void);
	void initial_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void publish_pose(void);
	void publish_particles(void);
	void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
	void evaluate_particles(const sensor_msgs::LaserScan::ConstPtr& scan);
	void compute_total_weight_and_effective_sample_size(void);
	void compute_random_particle_rate(void);
	void estimate_robot_pose(void);
	void resample_particles(void);
	void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

	inline double nrand(double n)
	{
		return (n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX));
	}

	// world frame point, xy, to grid map index, uv
	inline void xy2uv(double x, double y, int* u, int* v)
	{
		double dx = x - map.info.origin.position.x;
		double dy = y - map.info.origin.position.y;
		*u = (int)(dx / map.info.resolution);
		*v = (int)(dy / map.info.resolution);
	}
};

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
	is_map_data(false),
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
	particle_num = min_particle_num;
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
	// subscriber
	ros::Subscriber initial_pose_sub = nh.subscribe("/initialpose", 1, &AMCL::initial_pose_callback, this);
	ros::Subscriber map_sub = nh.subscribe(input_map_topic_name, 1, &AMCL::map_callback, this);
	ros::Subscriber odom_sub = nh.subscribe(input_odom_topic_name, 100, &AMCL::odom_callback, this);
	ros::Subscriber scan_sub = nh.subscribe(input_scan_topic_name, 1, &AMCL::scan_callback, this);
	// publisher
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/amcl_pose", 1);
	particles_pub = nh.advertise<geometry_msgs::PoseArray>("/amcl_particles", 1);
	// initialization
	amcl_init();
	// wait for odometry and scan topics
	ros::spin();
}

void AMCL::reset_particles(double xo, double yo, double yawo)
{
	double wo = 1.0 / (double)particle_num;
	robot_pose.x = xo;
	robot_pose.y = yo;
	robot_pose.yaw = yawo;
	for (int i = 0; i < particle_num; i++)
	{
		particles[i].pose.x = xo + nrand(initial_cov_xx);
		particles[i].pose.y = yo + nrand(initial_cov_yy);
		particles[i].pose.yaw = yawo + nrand(initial_cov_yawyaw);
		particles[i].w = wo;
	}
	w_slow = w_fast = 0.0;
}

void AMCL::amcl_init(void)
{
	// initilizatioin of pf
	particles.resize(max_particle_num);
	reset_particles(start_x, start_y, start_yaw);
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
	reset_particles(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
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
	// update robot and particle pose by odometry
	static double prev_time;
	if (is_first_time)
	{
		prev_time = msg->header.stamp.toSec();
		is_first_time = false;
		return;
	}
	double curr_time = msg->header.stamp.toSec();
	double d_time = curr_time - prev_time;
	double d_dist = msg->twist.twist.linear.x * d_time;
	double d_yaw = msg->twist.twist.angular.z * d_time;
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
	// publish tf
	tf::Transform tf;
	tf::Quaternion q;
	static tf::TransformBroadcaster br;
	tf.setOrigin(tf::Vector3(robot_pose.x, robot_pose.y, 0.0));
	q.setRPY(0.0, 0.0, robot_pose.yaw);
	tf.setRotation(q);
	br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), map_frame, "/amcl_frame"));
	// publish pose and particle cloud
	publish_pose();
	publish_particles();
}

void AMCL::evaluate_particles(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	double z_hit = 0.95;
	double max_dist_prob = 0.043937;
	double z_hit_denom = 0.08;
	double z_rand = 0.05;
	double z_rand_mult = 0.033333;
	for (int i = 0; i < particle_num; i++)
	{
		double w = 0.0;
		double c = cos(particles[i].pose.yaw);
		double s = sin(particles[i].pose.yaw);
		double xo = base_link2laser.x * c - base_link2laser.y * s + particles[i].pose.x;
		double yo = base_link2laser.x * s + base_link2laser.y * c + particles[i].pose.y;
		for (int j = 0; j < scan->ranges.size(); j += scan_step)
		{
			double angle = scan->angle_min + scan->angle_increment * (double)j;
			double yaw = angle + particles[i].pose.yaw;
			double r = scan->ranges[j];
			if (r < scan->range_min || scan->range_max < r)
				continue;
			double x = r * cos(yaw) + xo;
			double y = r * sin(yaw) + yo;
			int u, v;
			xy2uv(x, y, &u, &v);
			double pz = 0.0;
			if (0 <= u && u < map.info.width && 0 <= v && v < map.info.height)
			{
				int node = v * map.info.width + u;
				double z = dist_map.at<float>(v, u);
				pz += z_hit * exp(-(z * z) / z_hit_denom);
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
	if (!is_map_data)
	{
		ROS_ERROR("no map data");
		return;
	}
	if (!is_tf_initialized)
	{
		ROS_ERROR("tf is not initialized");
		return;
	}
	static bool is_first = true;
	static double prev_time;
	double curr_time = msg->header.stamp.toSec();;
	if (!is_first)
	{
		double d_time = curr_time - prev_time;
		if (fabs(delta_dist) < update_dist && fabs(delta_yaw) < update_yaw && d_time < update_time)
			return;
	}
	else
	{
		is_first = false;
	}
	evaluate_particles(msg);
	compute_total_weight_and_effective_sample_size();
	estimate_robot_pose();
	compute_random_particle_rate();
	resample_particles();
	prev_time = curr_time;
	delta_dist = delta_yaw = 0.0;
	printf("x = %.3lf [m], y = %.3lf [m], yaw = %.3lf [deg]\n", robot_pose.x, robot_pose.y, robot_pose.yaw * 180.0 / M_PI);
	printf("particle_num = %d\n", particle_num);
	printf("effective_sample_size = %lf, total_weight = %lf\n", effective_sample_size, total_weight);
	printf("random_particle_rate = %lf\n", random_particle_rate);
	printf("\n");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "amcl");
	AMCL node;
	return 0;
}
