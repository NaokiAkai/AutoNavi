// Copyright © 2018 Naoki Akai. All rights reserved.

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>

class Mapping3d
{
private:
	ros::NodeHandle nh;
	std::string sensor_trajectory_file, sensor_points_file;
	pcl::PointCloud<pcl::PointXYZ>::Ptr raw_points, filtered_points;

public:
	Mapping3d();
};

Mapping3d::Mapping3d():
	nh("~"),
	sensor_trajectory_file("/tmp/sensor_trajectory.g2o_out.txt"),
	sensor_points_file("/tmp/sensor_points.log"),
	raw_points(new pcl::PointCloud<pcl::PointXYZ>),
	filtered_points(new pcl::PointCloud<pcl::PointXYZ>)
{
	// read parameters
	// mapping
	double x, y, z, roll, pitch, yaw;
	FILE* fp_trajectory = fopen(sensor_trajectory_file.c_str(), "r");
	FILE* fp_points = fopen(sensor_points_file.c_str(), "r");
	while (fscanf(fp_trajectory, "%lf %lf %lf %lf %lf %lf\n", &x, &y, &z, &roll, &pitch, &yaw) != EOF)
	{
		Eigen::Translation3f m2s_trans(x, y, z);
		Eigen::AngleAxisf m2s_rot_x(roll, Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf m2s_rot_y(pitch, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf m2s_rot_z(yaw, Eigen::Vector3f::UnitZ());
		Eigen::Matrix4f tf_map2sensor = (m2s_trans * m2s_rot_z * m2s_rot_y * m2s_rot_x).matrix();
		size_t ret_val;
		int points_num;
		ret_val = fread(&points_num, sizeof(int), 1, fp_points);
		for (int i = 0; i < points_num; i++)
		{
			float px, py, pz;
			ret_val = fread(&px, sizeof(float), 1, fp_points);
			ret_val = fread(&py, sizeof(float), 1, fp_points);
			ret_val = fread(&pz, sizeof(float), 1, fp_points);
//			if (px < 0.0 || px * px + py * py + pz * pz < 9.0f)
			if (px * px + py * py + pz * pz < 9.0f)
				continue;
			Eigen::Vector4f pl(px, py, pz, 1.0f);
			Eigen::Vector4f pm = tf_map2sensor * pl;
			pcl::PointXYZ p;
			p.x = pm(0);
			p.y = pm(1);
			p.z = pm(2);
if (p.z < -2.0 || 10.0 < p.z)
	continue;
			raw_points->points.push_back(p);
		}
	}
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize(0.5, 0.5, 0.5);
	approximate_voxel_filter.setInputCloud(raw_points);
	approximate_voxel_filter.filter(*filtered_points);
//	pcl::io::savePCDFileASCII("/tmp/map_points.pcd", *filtered_points);
	pcl::io::savePCDFileBinary("/tmp/map_points.pcd", *filtered_points);
	fclose(fp_trajectory);
	fclose(fp_points);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mapping_3d");
	Mapping3d node;
	return 0;
}
