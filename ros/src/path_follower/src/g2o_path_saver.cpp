#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>

class PathSaver
{
private:
	ros::NodeHandle nh;
	std::string input_path_topic_name, path_file_name;
	ros::Subscriber path_sub;
	double a1, a2, a3, a4, a5, a6, a7, a8, a9;

public:
	PathSaver(std::string fname);
	void path_callback(const nav_msgs::Path::ConstPtr& msg);
};

PathSaver::PathSaver(std::string fname):
	nh("~"),
	input_path_topic_name("/recorded_path"),
	a1(0.01),
	a2(0.01),
	a3(0.01),
	a4(0.01),
	a5(0.01),
	a6(0.01),
	a7(0.01),
	a8(0.01),
	a9(0.01)
{
	// read parameters
	nh.param("/g2o_path_saver/input_path_topic_name", input_path_topic_name, input_path_topic_name);
	// subscriber
	path_sub = nh.subscribe(input_path_topic_name, 1, &PathSaver::path_callback, this);
	// wait for path
	path_file_name = fname;
	ros::spin();
}

void PathSaver::path_callback(const nav_msgs::Path::ConstPtr& msg)
{
	FILE* fp = fopen(path_file_name.c_str(), "w");
	if (fp == NULL)
	{
		ROS_ERROR("g2o path saver could not open the give file -> %s", path_file_name.c_str());
		exit(-1);
	}
	std::vector<double> yaws;
	// write node (robot trajectory)
	for (int i = 0; i < msg->poses.size(); i++)
	{
		tf::Quaternion q(msg->poses[i].pose.orientation.x,
			msg->poses[i].pose.orientation.y,
			msg->poses[i].pose.orientation.z,
			msg->poses[i].pose.orientation.w);
		double roll, pitch, yaw;
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);
		fprintf(fp, "VERTEX_SE2 %d %lf %lf %lf\n", i, msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, yaw);
		yaws.push_back(yaw);
	}
	// write edge between robot trajectory
	for (int i = 0; i < msg->poses.size() - 1; i++)
	{
		double dx = msg->poses[i + 1].pose.position.x - msg->poses[i].pose.position.x;
		double dy = msg->poses[i + 1].pose.position.y - msg->poses[i].pose.position.y;
		double ex = dx * cos(-yaws[i]) - dy * sin(-yaws[i]);
		double ey = dx * sin(-yaws[i]) + dy * cos(-yaws[i]);
		double eyaw = yaws[i + 1] - yaws[i];
		if (eyaw < -M_PI)
			eyaw += 2.0 * M_PI;
		if (eyaw > M_PI)
			eyaw -= 2.0 * M_PI;
		double d_rot1 = atan2(dy, dx) - yaws[i];
		if (d_rot1 < -M_PI)
			d_rot1 += 2.0 * M_PI;
		if (d_rot1 > M_PI)
			d_rot1 -= 2.0 * M_PI;
		double d_trans = sqrt(dx * dx + dy * dy);
		double d_rot2 = eyaw - d_rot1;
		if (d_rot2 < -M_PI)
			d_rot2 += 2.0 * M_PI;
		if (d_rot2 > M_PI)
			d_rot2 -= 2.0 * M_PI;
		double c = cos(yaws[i] + d_rot1);
		double s = sin(yaws[i] + d_rot1);
		double d_trans2 = d_trans * d_trans;
		double d_rot12 = d_rot1 * d_rot1;
		double d_rot22 = d_rot2 * d_rot2;
		Eigen::Matrix3d G, V, M, S0, S1, IM;
		G << 1.0, 0.0, -d_trans * s, 0.0, 1.0, d_trans * c, 0.0, 0.0, 1.0;
		V << c, -d_trans * s, 0.0, s, d_trans * c, 0.0, 0.0, 1.0, 1.0;
		M << a1 * d_trans2 + a2 * d_rot12 + a3 * d_rot22, 0.0, 0.0, 0.0, a4 * d_trans2 + a5 * d_rot12 + a6 * d_rot22, 0.0, 0.0, 0.0, a7 * d_trans2 + a8 * d_rot12 + a9 * d_rot22;
		S0 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		S1 = G * S0 * G.transpose() + V * M * V.transpose();
		IM = S1.inverse();
		fprintf(fp, "EDGE_SE2 %d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", i, i + 1, ex, ey, eyaw, IM(0, 0), IM(0, 1), IM(0, 2), IM(1, 1), IM(1, 2), IM(2, 2));
	}
	// write loop edge (assume that start and goal pose is the same pose)
	fprintf(fp, "EDGE_SE2 0 %d 0.0 0.0 0.0 1.0 0.0 0.0 1.0 0.0 1.0\n", (int)msg->poses.size() - 1);
	fclose(fp);
	printf("g2o path saver saved path data\n");
	exit(0);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "g2o_path_saver");
	if (argv[1] == NULL)
	{
		ROS_ERROR("g2o path saver requires file name as argv[1]");
		exit(-1);
	}
	std::string fname(argv[1]);
	PathSaver node(fname);
	return 0;
}
