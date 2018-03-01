#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class FG
{
private:
	ros::NodeHandle nh;

public:
	FG();
};

FG::FG():
	nh("~")
{

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "g2o_file_generator");
	FG node;
	return 0;
}
