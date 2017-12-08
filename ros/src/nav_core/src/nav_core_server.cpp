#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <yaml-cpp/yaml.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>

class NavCoreServer
{
private:
	ros::NodeHandle nh;
	std::string map_frame, base_link_frame, root_dir_name;
	ros::Publisher map_pub, path_pub, pose_pub;
	std::string output_map_topic_name, output_path_topic_name;
	nav_msgs::OccupancyGrid map;
	nav_msgs::Path path;
	double last_pose_x, last_pose_y, last_pose_yaw;
	geometry_msgs::PoseWithCovarianceStamped initial_pose;
	tf::TransformListener tf_listener;
	int count;

public:
	NavCoreServer();
	void spin();
	void read_map_yaml_file(void);
	void read_map_pgm_file(void);
	void read_path_file(void);
	void read_last_pose_file(void);
	void set_next_initial_pose(void);
};

NavCoreServer::NavCoreServer():
	nh("~"),
	map_frame("/map"),
	base_link_frame("/base_link"),
	root_dir_name("/home/akai/Dropbox/work/AutoNavi/ros/data/sample/"),
	output_map_topic_name("/amcl_map"),
	output_path_topic_name("/target_path"),
	tf_listener(),
	count(0)
{
	// read parameters
	nh.param("/nav_core_server/map_frame", map_frame, map_frame);
	nh.param("/nav_core_server/base_link_frame", base_link_frame, base_link_frame);
	nh.param("/nav_core_server/root_dir_name", root_dir_name, root_dir_name);
	nh.param("/nav_core_server/output_map_topic_name", output_map_topic_name, output_map_topic_name);
	nh.param("/nav_core_server/output_path_topic_name", output_path_topic_name, output_path_topic_name);
	// publisher
	map_pub = nh.advertise<nav_msgs::OccupancyGrid>(output_map_topic_name, 1, true);
	path_pub = nh.advertise<nav_msgs::Path>(output_path_topic_name, 1, true);
	pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
}

void NavCoreServer::spin(void)
{
	read_map_yaml_file();
	read_map_pgm_file();
	read_path_file();
	map_pub.publish(map);
	path_pub.publish(path);
	ros::Rate loop_rate(10.0);
	while (ros::ok())
	{
		ros::spinOnce();
		bool publish_next_data = false;
		bool request_new_map;
		nh.getParam("/nav_params/request_new_map", request_new_map);
		if (request_new_map)
		{
			nh.setParam("/nav_params/request_new_map", false);
			read_last_pose_file();
			set_next_initial_pose();
			count++;
			read_map_pgm_file();
			read_path_file();
			map_pub.publish(map);
			path_pub.publish(path);
			pose_pub.publish(initial_pose);
			printf("published new data\n");
		}
		loop_rate.sleep();
	}
}

void NavCoreServer::read_map_yaml_file(void)
{
	char fname[1024];
	sprintf(fname, "%sogm_%d.yaml", root_dir_name.c_str(), count);
	YAML::Node lconf = YAML::LoadFile(fname);
	map.info.resolution = lconf["resolution"].as<float>();
	std::vector<double> origin = lconf["origin"].as<std::vector<double> >();
	map.info.origin.position.x = origin[0];
	map.info.origin.position.y = origin[1];
}

void NavCoreServer::read_map_pgm_file(void)
{
	char fname[1024];
	sprintf(fname, "%sogm_%d.pgm", root_dir_name.c_str(), count);
	cv::Mat map_img = cv::imread(fname, 0);
	map.info.width = map_img.cols;
	map.info.height = map_img.rows;
	map.data.resize(map_img.cols * map_img.rows);
	for (int u = 0; u < map.info.width; u++)
	{
		for (int v = 0; v < map.info.height; v++)
		{
			int node = v * map.info.width + u;
			int val = map_img.at<uchar>(map.info.height - 1 - v, u);
			if (val <= 10)
				val = 100;
			else if (val >= 245)
				val = 0;
			else
				val = -1;
			map.data[node] = val;
		}
	}
}

void NavCoreServer::read_path_file(void)
{
	char fname[1024];
	sprintf(fname, "%spath_%d.txt", root_dir_name.c_str(), count);
	FILE* fp = fopen(fname, "r");
	geometry_msgs::PoseStamped pose;
	path.header.frame_id = pose.header.frame_id = map_frame;
	path.header.stamp = pose.header.stamp = ros::Time::now();
	path.poses.clear();
	while (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf", &pose.pose.position.x, &pose.pose.position.y, &pose.pose.position.z,
			&pose.pose.orientation.x, &pose.pose.orientation.y, &pose.pose.orientation.z, &pose.pose.orientation.w) != EOF)
		path.poses.push_back(pose);
	fclose(fp);
}

void NavCoreServer::read_last_pose_file(void)
{
	char fname[1024];
	sprintf(fname, "%slast_pose_%d.txt", root_dir_name.c_str(), count);
	FILE* fp = fopen(fname, "r");
	int val = fscanf(fp, "%lf %lf %lf", &last_pose_x, &last_pose_y, &last_pose_yaw);
	fclose(fp);
}

void NavCoreServer::set_next_initial_pose(void)
{
	tf::StampedTransform map2base_link;
	while (ros::ok())
	{
		ros::Time now = ros::Time::now();
		try
		{
			tf_listener.waitForTransform(map_frame, base_link_frame, now, ros::Duration(0.2));
			tf_listener.lookupTransform(map_frame, base_link_frame, now, map2base_link);
			break;
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
		}
	}
	double xo = map2base_link.getOrigin().x();
	double yo = map2base_link.getOrigin().y();
	tf::Quaternion q(map2base_link.getRotation().x(), map2base_link.getRotation().y(), map2base_link.getRotation().z(), map2base_link.getRotation().w());
	double rollo, pitcho, yawo;
	tf::Matrix3x3 m(q);
	m.getRPY(rollo, pitcho, yawo);
	double x = xo - last_pose_x;
	double y = yo - last_pose_y;
	double ini_x = x * cos(last_pose_yaw) + y * sin(last_pose_yaw);
	double ini_y = -x * sin(last_pose_yaw) + y * cos(last_pose_yaw);
	double ini_yaw = yawo - last_pose_yaw;
	if (ini_yaw < -M_PI)
		ini_yaw += 2.0 * M_PI;
	if (ini_yaw > M_PI)
		ini_yaw -= 2.0 * M_PI;
	tf::Transform tf;
	tf.setOrigin(tf::Vector3(ini_x, ini_y, 0.0));
	q.setRPY(0.0, 0.0, ini_yaw);
	tf.setRotation(q);
	initial_pose.header.frame_id = map_frame;
	initial_pose.header.stamp = ros::Time::now();
	initial_pose.pose.pose.position.x = tf.getOrigin().x();
	initial_pose.pose.pose.position.y = tf.getOrigin().y();
	initial_pose.pose.pose.position.z = tf.getOrigin().z();
	initial_pose.pose.pose.orientation.x = tf.getRotation().x();
	initial_pose.pose.pose.orientation.y = tf.getRotation().y();
	initial_pose.pose.pose.orientation.z = tf.getRotation().z();
	initial_pose.pose.pose.orientation.w = tf.getRotation().w();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nav_core_server");
	NavCoreServer node;
	node.spin();
	return 0;
}
