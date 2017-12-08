#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

class NavCoreServer
{
private:
	ros::NodeHandle nh;
	std::string root_dir_name;
	ros::Publisher map_pub, path_pub;
	std::string output_map_topic_name, output_path_topic_name;
	nav_msgs::OccupancyGrid map;
	nav_msgs::Path path;
	int count;

public:
	NavCoreServer();
	void spin();
	void read_map_yaml_file(void);
	void read_map_pgm_file(void);
};

NavCoreServer::NavCoreServer():
	nh("~"),
	root_dir_name("/home/akai/Dropbox/work/AutoNavi/ros/data/sample/"),
	output_map_topic_name("/amcl_map"),
	output_path_topic_name("/target_path"),
	count(0)
{
	// read parameters
	nh.param("/nav_core_server/root_dir_name", root_dir_name, root_dir_name);
	nh.param("/nav_core_server/output_map_topic_name", output_map_topic_name, output_map_topic_name);
	nh.param("/nav_core_server/output_path_topic_name", output_path_topic_name, output_path_topic_name);
	// subscriber

	// publisher
	map_pub = nh.advertise<nav_msgs::OccupancyGrid>(output_map_topic_name, 1);
	path_pub = nh.advertise<nav_msgs::Path>(output_path_topic_name, 1);
	// initialization
	read_map_yaml_file();
}

void NavCoreServer::spin(void)
{
	ros::Rate loop_rate(1.0);
	while (ros::ok())
	{
		ros::spinOnce();
		bool publish_next_data = false;
		nh.getParam("/nav_core_server/publish_next_data", publish_next_data);
		if (publish_next_data)
		{
			read_map_pgm_file();
			map_pub.publish(map);
			nh.setParam("/nav_core_server/publish_next_data", false);
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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nav_core_server");
	NavCoreServer node;
	node.spin();
	return 0;
}
