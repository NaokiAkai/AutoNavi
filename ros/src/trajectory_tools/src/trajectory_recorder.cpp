#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class TrajectoryRecorder
{
private:
	ros::NodeHandle nh;
	std::string map_frame, base_link_frame;
	std::string path_file_name;
	double dist_interval, angle_interval;

public:
	TrajectoryRecorder();
};

TrajectoryRecorder::TrajectoryRecorder():
	nh("~"),
	map_frame("/world"),
	base_link_frame("/base_link"),
	path_file_name("/tmp/simple_trajectory.txt"),
	dist_interval(0.5),
	angle_interval(2.0)
{
	// read parameters
	nh.param("/trajectory_recorder/map_frame", map_frame, map_frame);
	nh.param("/trajectory_recorder/base_link_frame", base_link_frame, base_link_frame);
	nh.param("/trajectory_recorder/path_file_name", path_file_name, path_file_name);
	nh.param("/trajectory_recorder/dist_interval", dist_interval, dist_interval);
	nh.param("/trajectory_recorder/angle_interval", angle_interval, angle_interval);
	// convert degree to radian
	angle_interval *= M_PI / 180.0;
	FILE* fp = fopen(path_file_name.c_str(), "w");
	if (fp == NULL)
	{
		ROS_ERROR("cannot open path file, %s", path_file_name.c_str());
		exit(1);
	}
	// trajectory record based on localization result
	tf::TransformListener tf_listener;
	tf::StampedTransform map2base_link;
	bool is_first = true;
	double xo, yo, yawo;
	bool do_record = false;
	ros::Rate loop_rate(40);
	while (ros::ok())
	{
		ros::Time now = ros::Time::now();
		try
		{
			tf_listener.waitForTransform(map_frame, base_link_frame, now, ros::Duration(1.0));
			tf_listener.lookupTransform(map_frame, base_link_frame, now, map2base_link);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			continue;
		}
		double x = map2base_link.getOrigin().x();
		double y = map2base_link.getOrigin().y();
		tf::Quaternion q(map2base_link.getRotation().x(),
			map2base_link.getRotation().y(),
			map2base_link.getRotation().z(),
			map2base_link.getRotation().w());
		double roll, pitch, yaw;
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);
		if (is_first)
		{
			do_record = true;
			is_first = false;
		}
		else
		{
			double dx = x - xo;
			double dy = y - yo;
			double dl = sqrt(dx * dx + dy * dy);
			double dyaw = yaw - yawo;
			if (dyaw < -M_PI)	dyaw += 2.0 * M_PI;
			if (dyaw > M_PI)	dyaw -= 2.0 * M_PI;
			if (dl > dist_interval || fabs(dyaw) > angle_interval)
				do_record = true;
		}
		if (do_record)
		{
			fprintf(fp, "%lf %lf\n", x, y);
			printf("x = %.3lf [m], y = %.3lf [m]\n", x, y);
			xo = x;
			yo = y;
			yawo = yaw;
			do_record = false;
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	fclose(fp);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "trajectory_recorder");
	TrajectoryRecorder node;
	return 0;
}
