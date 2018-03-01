#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <path_follower/GetPath.h>

class PathServer
{
private:
	ros::NodeHandle nh;
	std::string map_frame;
	std::string output_path_topic_name, service_path_topic_name;
	path_follower::GetPath::Response path_res;
	ros::ServiceServer path_srv;
	ros::Publisher path_pub;

public:
	PathServer(std::string fname);
	~PathServer() {};
	bool path_callback(path_follower::GetPath::Request& req, path_follower::GetPath::Response& res);
};

PathServer::PathServer(std::string fname):
	nh("~"),
	map_frame("/map"),
	output_path_topic_name("/target_path"),
	service_path_topic_name("/static_target_path")
{
	// read parameters
	nh.param("/path_server/map_frame", map_frame, map_frame);
	// server
	path_srv = nh.advertiseService(service_path_topic_name, &PathServer::path_callback, this);
	// publisher
	path_pub = nh.advertise<nav_msgs::Path>(output_path_topic_name, 1, true);
	// read path data
	FILE* fp = fopen(fname.c_str(), "r");
	if (fp == NULL)
	{
		ROS_ERROR("path_server could not open the give file -> %s", fname.c_str());
		exit(-1);
	}
	geometry_msgs::PoseStamped pose;
	path_res.path.header.frame_id = pose.header.frame_id = map_frame;
	path_res.path.header.stamp = pose.header.stamp = ros::Time::now();
	while (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf", &pose.pose.position.x, &pose.pose.position.y, &pose.pose.position.z,
			&pose.pose.orientation.x, &pose.pose.orientation.y, &pose.pose.orientation.z, &pose.pose.orientation.w) != EOF)
	{
		printf("x = %.3lf, y = %.3lf\n", pose.pose.position.x, pose.pose.position.y);
		path_res.path.poses.push_back(pose);
	}
	fclose(fp);
	// latched publisher for path
	path_pub.publish(path_res.path);
}

bool PathServer::path_callback(path_follower::GetPath::Request& req, path_follower::GetPath::Response& res)
{
	res = path_res;
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_server");
	if (argv[1] == NULL)
	{
		ROS_ERROR("path_server requires a path data file as argv[1]");
		exit(-1);
	}
	std::string fname(argv[1]);
	PathServer node(fname);
	ros::spin();
	return 0;
}
