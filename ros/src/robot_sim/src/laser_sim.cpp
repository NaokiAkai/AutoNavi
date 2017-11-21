#include <ros/ros.h>
#include <time.h>
#include <std_msgs/Time.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "common.h"

typedef struct
{
	bool is_active;
	pose_t pose;
	double v;
	double size;
} moving_object_t;

class LaserSim
{
private:
	ros::NodeHandle nh;
	ros::Subscriber map_sub;
	ros::Publisher scan_pub, points_pub, map_pub;
	std::string input_map_topic_name;
	std::string output_scan_topic_name, output_scan_points_topic_name;
	std::string map_frame, laser_frame, base_link_frame;
	int moving_object_num;
	double moving_object_mean_x, moving_object_mean_y, moving_object_mean_v, moving_object_mean_size;
	double moving_object_std_x, moving_object_std_y, moving_object_std_v, moving_object_std_size;
	std::vector<moving_object_t> moving_objects;
	std::string sensor_type;
	nav_msgs::OccupancyGrid map;
	sensor_msgs::LaserScan scan;
	bool is_tf_initialized, is_map_data;
	pose_t robot_pose, base_link2laser;
	tf::TransformListener tf_listener;

public:
	LaserSim();
	void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void laser_sim_init(void);
	bool get_ground_truth_robot_pose(void);
	void publish_scan(void);
	nav_msgs::OccupancyGrid make_virtual_environmental_map(void);

	inline double nrand(double n)
	{
		return (n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX));
	}

	inline void mod_yaw(double *yaw)
	{
		while (*yaw < -M_PI)	*yaw += 2.0 * M_PI;
		while (*yaw > M_PI)		*yaw -= 2.0 * M_PI;
	}

	inline void xy2node(double x, double y, int* node)
	{
		double dx = x - map.info.origin.position.x;
		double dy = y - map.info.origin.position.y;
		int u = (int)(dx / map.info.resolution);
		int v = (int)(dy / map.info.resolution);
		if (u < 0 || map.info.width <= u || v < 0 || map.info.height <= v)
			*node = -1;
		else
			*node = v * map.info.width + u;
	}
};

LaserSim::LaserSim():
	nh("~"),
	input_map_topic_name("/laser_sim_map_source"),
	output_scan_topic_name("/scan_robot_sim"),
	output_scan_points_topic_name("/scan_points_robot_sim"),
	map_frame("/world"),
	laser_frame("/laser"),
	base_link_frame("/base_link"),
	moving_object_num(50),
	moving_object_mean_x(0.0),
	moving_object_mean_y(0.0),
	moving_object_mean_v(1.0),
	moving_object_mean_size(0.5),
	moving_object_std_x(0.0),
	moving_object_std_y(0.0),
	moving_object_std_v(1.0),
	moving_object_std_size(0.5),
	sensor_type("top_urg"),
	tf_listener(),
	is_tf_initialized(false),
	is_map_data(false)
{
	// read parameters
	nh.param("/laser_sim/input_map_topic_name", input_map_topic_name, input_map_topic_name);
	nh.param("/laser_sim/output_scan_topic_name", output_scan_topic_name, output_scan_topic_name);
	nh.param("/laser_sim/output_scan_points_topic_name", output_scan_points_topic_name, output_scan_points_topic_name);
	nh.param("/laser_sim/laser_frame", laser_frame, laser_frame);
	nh.param("/laser_sim/moving_object_num", moving_object_num, moving_object_num);
	nh.param("/laser_sim/moving_object_mean_x", moving_object_mean_x, moving_object_mean_x);
	nh.param("/laser_sim/moving_object_mean_y", moving_object_mean_y, moving_object_mean_y);
	nh.param("/laser_sim/moving_object_mean_v", moving_object_mean_v, moving_object_mean_v);
	nh.param("/laser_sim/moving_object_mean_size", moving_object_mean_size, moving_object_mean_size);
	nh.param("/laser_sim/moving_object_std_x", moving_object_std_x, moving_object_std_x);
	nh.param("/laser_sim/moving_object_std_y", moving_object_std_y, moving_object_std_y);
	nh.param("/laser_sim/moving_object_std_v", moving_object_std_v, moving_object_std_v);
	nh.param("/laser_sim/moving_object_std_size", moving_object_std_size, moving_object_std_size);
	nh.param("/laser_sim/sensor_type", sensor_type, sensor_type);
	if (sensor_type != "top_urg" && sensor_type != "classic_urg" && sensor_type != "tough_urg")
	{
		ROS_ERROR("unsupported sensor type is selected. top_urg, classic_urg, or tough_urg must be selected.");
		exit(1);
	}
	// Subscriber
	map_sub = nh.subscribe(input_map_topic_name, 1, &LaserSim::map_callback, this);
	// Publisher
	scan_pub = nh.advertise<sensor_msgs::LaserScan>(output_scan_topic_name, 100);
	points_pub = nh.advertise<sensor_msgs::PointCloud>(output_scan_points_topic_name, 100);
	map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/laser_sim_map", 10);
	// initialization
	laser_sim_init();
	// main loop
	ros::Rate loop_rate(1.0 / scan.scan_time);
	while (ros::ok())
	{
		ros::spinOnce();
		if (get_ground_truth_robot_pose())
		{
			publish_scan();
			printf("x = %.3lf [m], y = %.3lf [m], yaw = %.3lf [deg]\n", robot_pose.x, robot_pose.y, robot_pose.yaw * 180.0 / M_PI);
		}
		loop_rate.sleep();
	}
}

void LaserSim::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	if (!is_map_data) {
		map = *msg;
		is_map_data = true;
	}
}

void LaserSim::laser_sim_init(void)
{
	srand((unsigned)time(NULL));
	// initilizatioin of simulated scan
	if (sensor_type == "top_urg")
	{
		scan.angle_min = -135.0 * M_PI / 180.0;
		scan.angle_max = 135.0 * M_PI / 180.0;
		scan.angle_increment = 0.25 * M_PI / 180.0;
		scan.time_increment = 1.0 / 40.0;
		scan.scan_time = scan.time_increment;
		scan.range_min = 0.05;
		scan.range_max = 30.0;
	}
	else if (sensor_type == "classic_urg")
	{
		scan.angle_min = -135.0 * M_PI / 180.0;
		scan.angle_max = 135.0 * M_PI / 180.0;
		scan.angle_increment = 0.5 * M_PI / 180.0;
		scan.time_increment = 1.0 / (double)40.0;
		scan.scan_time = scan.time_increment;
		scan.range_min = 0.05;
		scan.range_max = 6.0;
	}
	else if (sensor_type == "tough_urg")
	{
		scan.angle_min = -95.0 * M_PI / 180.0;
		scan.angle_max = 95.0 * M_PI / 180.0;
		scan.angle_increment = 0.125 * M_PI / 180.0;
		scan.time_increment = 1.0 / (double)40.0;
		scan.scan_time = scan.time_increment;
		scan.range_min = 0.05;
		scan.range_max = 80.0;
	}
	scan.header.frame_id = laser_frame;
	int scan_num = (int)((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1;
	scan.ranges.resize(scan_num);
	scan.intensities.resize(scan_num);
	// initilizatioin of moving objects
	moving_objects.resize(moving_object_num);
	for (int i = 0; i < moving_objects.size(); i++) {
		moving_objects[i].pose.x = moving_object_mean_x + nrand(moving_object_std_x);
		moving_objects[i].pose.y = moving_object_mean_y + nrand(moving_object_std_y);
		moving_objects[i].pose.yaw = nrand(M_PI);
		mod_yaw(&moving_objects[i].pose.yaw);
		moving_objects[i].v = moving_object_mean_v + fabs(nrand(moving_object_std_v));
		moving_objects[i].size = moving_object_mean_size + fabs(nrand(moving_object_std_size));
		int node;
		xy2node(moving_objects[i].pose.x, moving_objects[i].pose.y, &node);
		if (node >= 0)
			moving_objects[i].is_active = true;
		else
			moving_objects[i].is_active = false;
	}
	// initilizatioin of tf
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

bool LaserSim::get_ground_truth_robot_pose(void)
{
	tf::StampedTransform map2base_link;
	ros::Time now = ros::Time::now();
	try
	{
		tf_listener.waitForTransform(map_frame, "ground_truth", now, ros::Duration(0.1));
		tf_listener.lookupTransform(map_frame, "ground_truth", now, map2base_link);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
		return false;
	}
	robot_pose.x = map2base_link.getOrigin().x();
	robot_pose.y = map2base_link.getOrigin().y();
	tf::Quaternion q(map2base_link.getRotation().x(),
		map2base_link.getRotation().y(),
		map2base_link.getRotation().z(),
		map2base_link.getRotation().w());
	double roll, pitch;
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, robot_pose.yaw);
	return true;
}

void LaserSim::publish_scan(void)
{
	static unsigned int cnt;
	if (!is_map_data || !is_tf_initialized)
	{
		ROS_ERROR("map or tf data is invalid");
		return;
	}
	// make scan data
	int i = 0;
	double xo = base_link2laser.x * cos(robot_pose.yaw) - base_link2laser.y * sin(robot_pose.yaw) + robot_pose.x;
	double yo = base_link2laser.x * sin(robot_pose.yaw) + base_link2laser.y * cos(robot_pose.yaw) + robot_pose.y;
	double yawo = robot_pose.yaw + base_link2laser.yaw;
	std_msgs::Time curr_time;
	curr_time.data = ros::Time::now();
	sensor_msgs::PointCloud points;
	points.channels.resize(1);
	points.channels[0].name = "intensity";
	nav_msgs::OccupancyGrid env_map = make_virtual_environmental_map();
	for (double yaw = scan.angle_min; yaw <= scan.angle_max; yaw += scan.angle_increment)
	{
		double t = yawo + yaw;
		double dx = map.info.resolution * cos(t);
		double dy = map.info.resolution * sin(t);
		double x = xo;
		double y = yo;
		scan.ranges[i] = 0.0;
		scan.intensities[i] = 0.0;
		for (double l = 0.0; l <= scan.range_max; l += map.info.resolution)
		{
			int node;
			xy2node(x, y, &node);
			if (node < 0)	// out of map
			{
				scan.ranges[i] = 0.0;
				scan.intensities[i] = 0.0;
				break;
			}
			else if (env_map.data[node] == 100)	// hit to obstacle
			{
				scan.ranges[i] = l;
				scan.intensities[i] = 100;
				geometry_msgs::Point32 p;
				p.x = l * cos(yaw);
				p.y = l * sin(yaw);
				p.z = 0.0;
				points.points.push_back(p);
				points.channels[0].values.push_back(100);
				break;
			}
			else if (l > scan.range_max)	// out of measurement range
			{
				scan.ranges[i] = 0.0;
				scan.intensities[i] = 0.0;
				break;
			}
			x += dx;	// lay casting
			y += dy;
		}
		i++;
	}
	scan.header.stamp = curr_time.data;
	scan.header.frame_id = laser_frame;
	points.header.stamp = curr_time.data;
	points.header.frame_id = laser_frame;
	scan_pub.publish(scan);
	points_pub.publish(points);
	if (cnt >= (int)((1.0 / scan.scan_time) / 10.0)) {
		map_pub.publish(env_map);
		cnt = 0;
	}
	cnt++;
}

nav_msgs::OccupancyGrid LaserSim::make_virtual_environmental_map(void)
{
	nav_msgs::OccupancyGrid env_map = map;
	double dt = 1.0 / (1.0 / scan.scan_time);
	for (int i = 0; i < moving_objects.size(); i++)
	{
		if (moving_objects[i].is_active)
		{
			moving_objects[i].pose.x += moving_objects[i].v * cos(moving_objects[i].pose.yaw) * dt;
			moving_objects[i].pose.y += moving_objects[i].v * sin(moving_objects[i].pose.yaw) * dt;
			int node;
			xy2node(moving_objects[i].pose.x, moving_objects[i].pose.y, &node);
			if (node >= 0)
			{
				double sh = moving_objects[i].size / 2.0;
				for (double x = moving_objects[i].pose.x - sh; x <= moving_objects[i].pose.x + sh; x += map.info.resolution)
				{
					for (double y = moving_objects[i].pose.y - sh; y <= moving_objects[i].pose.y + sh; y += map.info.resolution)
					{
						xy2node(x, y, &node);
						if (node >= 0)
							env_map.data[node] = 100;
					}
				}
			}
			else
			{
				moving_objects[i].is_active = false;
			}
		}
		else
		{
			moving_objects[i].pose.x = moving_object_mean_x + nrand(moving_object_std_x);
			moving_objects[i].pose.y = moving_object_mean_y + nrand(moving_object_std_y);
			moving_objects[i].pose.yaw = nrand(M_PI);
			mod_yaw(&moving_objects[i].pose.yaw);
			moving_objects[i].v = moving_object_mean_v + fabs(nrand(moving_object_std_v));
			moving_objects[i].size = moving_object_mean_size + fabs(nrand(moving_object_std_size));
			int node;
			xy2node(moving_objects[i].pose.x, moving_objects[i].pose.y, &node);
			if (node >= 0)
				moving_objects[i].is_active = true;
		}
	}
	return env_map;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_sim");
	LaserSim node;
	return 0;
}
