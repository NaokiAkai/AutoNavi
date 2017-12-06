#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

class TrajectoryDrawer
{
private:
	ros::NodeHandle nh;
	std::string map_frame;
	std::string path_file_name;
	ros::Publisher path_pub;
	double draw_interval;
	visualization_msgs::Marker path;

public:
	TrajectoryDrawer();
	TrajectoryDrawer(std::string fname);
	void init(void);
};

TrajectoryDrawer::TrajectoryDrawer():
	nh("~"),
	map_frame("/world"),
	path_file_name("/tmp/simple_trajectory.txt"),
	draw_interval(0.5)
{
	// read parameters
	nh.param("/trajectory_drawer/map_frame", map_frame, map_frame);
	nh.param("/trajectory_drawer/path_file_name", path_file_name, path_file_name);
	nh.param("/trajectory_drawer/draw_interval", draw_interval, draw_interval);
	// publisher
	path_pub = nh.advertise<visualization_msgs::Marker>("/target_trajectory", 1);
	// initialization
	init();
	// main loop
	ros::Rate loop_rate(2.0);
	while (ros::ok())
	{
		path_pub.publish(path);
		ros::spinOnce();
		loop_rate.sleep();
	}
}

TrajectoryDrawer::TrajectoryDrawer(std::string fname):
	nh("~"),
	map_frame("/world"),
	draw_interval(0.5)
{
	// read parameters
	nh.param("/trajectory_drawer/map_frame", map_frame, map_frame);
	nh.param("/trajectory_drawer/draw_interval", draw_interval, draw_interval);
	// publisher
	path_pub = nh.advertise<visualization_msgs::Marker>("/target_trajectory", 1);
	// initialization
	path_file_name = fname;
	init();
	// main loop
	ros::Rate loop_rate(2.0);
	while (ros::ok())
	{
		path_pub.publish(path);
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void TrajectoryDrawer::init(void)
{
	// set path configuration
	path.header.frame_id = map_frame;
	path.header.stamp = ros::Time::now();
	path.ns = "target_path";
	path.action = visualization_msgs::Marker::ADD;
	path.pose.orientation.w = 1.0;
	path.lifetime = ros::Duration(2);
	path.frame_locked = true;
	path.id = 1;
	path.type = visualization_msgs::Marker::LINE_STRIP;
	path.scale.x = 0.15;
	path.scale.y = 1.0;
	path.scale.z = 1.0;
	path.color.r = 0.0;
	path.color.g = 1.0;
	path.color.b = 0.0;
	path.color.a = 1.0;
	// load path
	FILE* fp = fopen(path_file_name.c_str(), "r");
	if (fp == NULL)
	{
		ROS_ERROR("cannot open path file -> %s", path_file_name.c_str());
		exit(1);
	}
	int c = 0;
	double xo, yo;
	int val = fscanf(fp, "%lf %lf", &xo, &yo);
	geometry_msgs::Point p;
	p.x = xo;
	p.y = yo;
	p.z = 0.0;
	path.points.push_back(p);
	path.points.push_back(p);
	double x, y;
	while ((fscanf(fp, "%lf %lf", &x, &y)) != EOF)
	{
		double dx = x - xo;
		double dy = y - yo;
		double dl = sqrt(dx * dx + dy * dy);
		if (dl > draw_interval)
		{
			p.x = x;
			p.y = y;
			path.points.push_back(p);
			path.points.push_back(p);
			xo = x;
			yo = y;
		}
	}
	fclose(fp);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "trajectory_drawer");
	if (argv[1] == NULL)
		TrajectoryDrawer node;
	else
		TrajectoryDrawer node((std::string)argv[1]);
	return 0;
}
