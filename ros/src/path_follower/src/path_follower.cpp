#include <ros/ros.h>
#include <nav_msgs/Path.h>

class PathFollower
{
private:
	ros::NodeHandle nh;
	std::string input_path_topic_name;
	ros::Subscriber path_sub;
	nav_msgs::Path path;

public:
	PathFollower();
	void path_callback(const nav_msgs::Path::ConstPtr& msg);
	void spin(void);
};

PathFollower::PathFollower():
	nh("~"),
	input_path_topic_name("/target_path")
{
	// read parameters
	nh.param("/path_follower/input_path_topic_name", input_path_topic_name, input_path_topic_name);
	// subscriber
	path_sub = nh.subscribe(input_path_topic_name, 1, &PathFollower::path_callback, this);
}

void PathFollower::path_callback(const nav_msgs::Path::ConstPtr& msg)
{
	path = *msg;
printf("recv\n");
}

void PathFollower::spin(void)
{
	ros::spin();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_follower");
	PathFollower node;
	node.spin();
	return 0;
}
