// Copyright © 2018 Naoki Akai. All rights reserved.

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>

typedef struct
{
	double x, y, yaw;
} pose_t;

class ML2D
{
private:
	ros::NodeHandle nh;
	ros::Subscriber scan_sub;
	std::string data_dir;
	std::string map_frame, laser_frame;
	std::string input_scan_topic_name;
	double dist_interval, angle_interval;
	tf::TransformListener tf_listener;
	std::vector<sensor_msgs::LaserScan> scans, scan_buffer;
	std::vector<pose_t> poses;
	FILE* fp_scans, *fp_poses;
	double a1, a2, a3, a4, a5, a6, a7, a8, a9;

public:
	ML2D();
	void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void spin(void);
	void write_scan(double pose_time);
	void terminate(void);
};

ML2D::ML2D():
	nh("~"),
	data_dir("/tmp/"),
	map_frame("/map"),
	laser_frame("/laser"),
	input_scan_topic_name("/scan"),
	dist_interval(0.5),
	angle_interval(10.0),
	tf_listener(),
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
	nh.param("/mapping_logger_2d/data_dir", data_dir, data_dir);
	nh.param("/mapping_logger_2d/map_frame", map_frame, map_frame);
	nh.param("/mapping_logger_2d/laser_frame", laser_frame, laser_frame);
	nh.param("/mapping_logger_2d/input_scan_topic_name", input_scan_topic_name, input_scan_topic_name);
	nh.param("/mapping_logger_2d/dist_interval", dist_interval, dist_interval);
	nh.param("/mapping_logger_2d/angle_interval", angle_interval, angle_interval);
	// convert degree to radian
	angle_interval *= M_PI / 180.0;
	// subscriber
	scan_sub = nh.subscribe(input_scan_topic_name, 10, &ML2D::scan_callback, this);
	// open file to write logs
	std::string fname;
	fname = data_dir + "scans.log";
	fp_scans = fopen(fname.c_str(), "w");
	if (fp_scans == NULL)
	{
		ROS_ERROR("could not open file to write scan data -> %s", fname.c_str());
		exit(1);
	}
	fname = data_dir + "initial_poses.g2o";
	fp_poses = fopen(fname.c_str(), "w");
	if (fp_poses == NULL)
	{
		ROS_ERROR("could not open file to write g2o data");
		exit(1);
	}
}

void ML2D::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	scans.insert(scans.begin(), *msg);
	if (scans.size() > 40)
		scans.resize(40);
}

void ML2D::spin(void)
{
	bool is_first = true;
	pose_t prev_pose;
	bool do_record = false;
	ros::Rate loop_rate(40);
	while (ros::ok())
	{
		ros::spinOnce();
		tf::StampedTransform map2laser;
		ros::Time now = ros::Time::now();
		try
		{
			tf_listener.waitForTransform(map_frame, laser_frame, now, ros::Duration(1.0));
			tf_listener.lookupTransform(map_frame, laser_frame, now, map2laser);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			continue;
		}
		pose_t pose;
		pose.x = map2laser.getOrigin().x();
		pose.y = map2laser.getOrigin().y();
		tf::Quaternion q(map2laser.getRotation().x(), map2laser.getRotation().y(), map2laser.getRotation().z(), map2laser.getRotation().w());
		double roll, pitch;
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, pose.yaw);
		if (is_first)
		{
			do_record = true;
			is_first = false;
		}
		else
		{
			pose_t d;
			d.x = pose.x - prev_pose.x;
			d.y = pose.y - prev_pose.y;
			double dl = sqrt(d.x * d.x + d.y * d.y);
			d.yaw = pose.yaw - prev_pose.yaw;
			if (d.yaw < -M_PI)
				d.yaw += 2.0 * M_PI;
			if (d.yaw > M_PI)
				d.yaw -= 2.0 * M_PI;
			if (dl > dist_interval || fabs(d.yaw) > angle_interval)
				do_record = true;
		}
		if (do_record)
		{
			poses.push_back(pose);
			write_scan(now.toSec());
			printf("x = %.3lf [m], y = %.3lf [m], yaw = %lf [deg]\n", pose.x, pose.y, pose.yaw * 180.0 / M_PI);
			prev_pose = pose;
			do_record = false;
		}
		loop_rate.sleep();
	}
	terminate();
}

void ML2D::write_scan(double pose_time)
{
	static bool is_first = true;
	int index = -1;
	for (int i = 0; i < scans.size(); i++)
	{
		double scan_time = scans[i].header.stamp.toSec();
		if (scan_time < pose_time)
		{
			index = i;
			break;
		}
	}
	if (index < 0)
	{
		ROS_ERROR("no synchronized scan data");
		return;
	}
	sensor_msgs::LaserScan scan = scans[index];
	if (is_first)
	{
		fwrite(&scan.angle_min, sizeof(float), 1, fp_scans);
		fwrite(&scan.angle_max, sizeof(float), 1, fp_scans);
		fwrite(&scan.angle_increment, sizeof(float), 1, fp_scans);
		fwrite(&scan.time_increment, sizeof(float), 1, fp_scans);
		fwrite(&scan.scan_time, sizeof(float), 1, fp_scans);
		fwrite(&scan.range_min, sizeof(float), 1, fp_scans);
		fwrite(&scan.range_max, sizeof(float), 1, fp_scans);
		int scan_range_size = (int)scan.ranges.size();
		fwrite(&scan_range_size, sizeof(int), 1, fp_scans);
		is_first = false;
	}
	for (int i = 0; i < scan.ranges.size(); i++)
	{
		fwrite(&scan.ranges[i], sizeof(float), 1, fp_scans);
		fwrite(&scan.intensities[i], sizeof(float), 1, fp_scans);
	}
}

void ML2D::terminate(void)
{
	// save scan logs
	fclose(fp_scans);
	// create g2o data
	std::vector<double> yaws;
	// write node (robot trajectory)
	for (int i = 0; i < poses.size(); i++)
		fprintf(fp_poses, "VERTEX_SE2 %d %lf %lf %lf\n", i, poses[i].x, poses[i].y, poses[i].yaw);
	// write edge between robot trajectory (odometry constraint)
	for (int i = 0; i < poses.size() - 1; i++)
	{
		double dx = poses[i + 1].x - poses[i].x;
		double dy = poses[i + 1].y - poses[i].y;
		double ex = dx * cos(-poses[i].yaw) - dy * sin(-poses[i].yaw);
		double ey = dx * sin(-poses[i].yaw) + dy * cos(-poses[i].yaw);
		double eyaw = poses[i + 1].yaw - poses[i].yaw;
		if (eyaw < -M_PI)
			eyaw += 2.0 * M_PI;
		if (eyaw > M_PI)
			eyaw -= 2.0 * M_PI;
		double d_rot1 = atan2(dy, dx) - poses[i].yaw;
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
		double c = cos(poses[i].yaw + d_rot1);
		double s = sin(poses[i].yaw + d_rot1);
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
		fprintf(fp_poses, "EDGE_SE2 %d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", i, i + 1, ex, ey, eyaw, IM(0, 0), IM(0, 1), IM(0, 2), IM(1, 1), IM(1, 2), IM(2, 2));
	}
	// write loop edge (assume that start and goal pose is the same pose)
	fprintf(fp_poses, "EDGE_SE2 0 %d 0.0 0.0 0.0 100.0 0.0 0.0 100.0 0.0 100.0\n", (int)poses.size() - 1);
	fclose(fp_poses);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mapping_logger_2d");
	ML2D node;
	node.spin();
	return 0;
}
