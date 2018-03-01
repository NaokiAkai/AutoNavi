#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <chrono>

#include "p2o.h"

typedef struct
{
	double x, y, yaw;
} pose_t;

class Mapping2d
{
private:
	ros::NodeHandle nh;
	std::string data_dir;
	ros::Publisher map_pub;
	int max_iter, min_iter;
	double robust_thre;
	std::vector<pose_t> poses, raw_poses;
	nav_msgs::OccupancyGrid map, raw_map;
	std::string map_frame;
	double map_size_x, map_size_y, map_resolution, map_origin_x, map_origin_y;
	sensor_msgs::LaserScan scan;

public:
	Mapping2d();
	void update_cell(int u, int v, double dr);
};

Mapping2d::Mapping2d():
	nh("~"),
	data_dir("/tmp/"),
	max_iter(20),
	min_iter(3),
	robust_thre(1.0),
	map_frame("/map"),
	map_size_x(100.0),
	map_size_y(100.0),
	map_resolution(0.1),
	map_origin_x(-50.0),
	map_origin_y(-50.0)
{
	// read parameters
	nh.param("/mapping_2d/data_dir", data_dir, data_dir);
	nh.param("/mapping_2d/max_iter", max_iter, max_iter);
	nh.param("/mapping_2d/min_iter", min_iter, min_iter);
	nh.param("/mapping_2d/robust_thre", robust_thre, robust_thre);
	nh.param("/mapping_2d/map_frame", map_frame, map_frame);
	// publisher
	map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);



	// optimization
	std::string filename = data_dir + "initial_poses.g2o";
	std::string fname_in = data_dir + "initial_poses.txt";
	std::string fname_out = data_dir + "optimized_poses.txt";
	std::ofstream ofs(fname_in);
	std::ofstream ofs2(fname_out);
	p2o::Pose2DVec nodes;
	p2o::Con2DVec con;
	p2o::Optimizer2D optimizer;
	optimizer.setLambda(1e-6);
	optimizer.setVerbose(true);
	if (!optimizer.loadFile(filename.c_str(), nodes, con))
	{
		std::cout << "can't open file: " << filename << std::endl;
		return;
	}
	optimizer.setRobustThreshold(robust_thre);
	auto t0 = std::chrono::high_resolution_clock::now();
	p2o::Pose2DVec result = optimizer.optimizePath(nodes, con, max_iter, min_iter);
	auto t1 = std::chrono::high_resolution_clock::now();
	auto elapsed = std::chrono::duration_cast< std::chrono::microseconds> (t1-t0);
	std::cout << filename << ": " << elapsed.count()*1e-6 << "s" << std::endl;
	for(int i = 0; i<result.size(); i++)
	{
		ofs << nodes[i].x << " " << nodes[i].y << " " << nodes[i].th << std::endl;
		ofs2 << result[i].x << " " << result[i].y << " " << result[i].th << std::endl;
		pose_t p;
		p.x = nodes[i].x;
		p.y = nodes[i].y;
		p.yaw = nodes[i].th;
		raw_poses.push_back(p);
		p.x = result[i].x;
		p.y = result[i].y;
		p.yaw = result[i].th;
		poses.push_back(p);
	}



	map.info.resolution = map_resolution;
	map.info.width = (int)(map_size_x / map_resolution);
	map.info.height = (int)(map_size_y / map_resolution);
	map.info.origin.position.x = map_origin_x;
	map.info.origin.position.y = map_origin_y;
	map.data.resize(map.info.width * map.info.height, -1);
	raw_map = map;

	std::string fname = data_dir + "scans.log";
	FILE* fp = fopen(fname.c_str(), "r");
	if (fp == NULL)
	{
		ROS_ERROR("could not open scan log file -> %s", fname.c_str());
		exit(1);
	}
	size_t ret_val;
	ret_val = fread(&scan.angle_min, sizeof(float), 1, fp);
	ret_val = fread(&scan.angle_max, sizeof(float), 1, fp);
	ret_val = fread(&scan.angle_increment, sizeof(float), 1, fp);
	ret_val = fread(&scan.time_increment, sizeof(float), 1, fp);
	ret_val = fread(&scan.scan_time, sizeof(float), 1, fp);
	ret_val = fread(&scan.range_min, sizeof(float), 1, fp);
	ret_val = fread(&scan.range_max, sizeof(float), 1, fp);
	int scan_range_size;
	ret_val = fread(&scan_range_size, sizeof(int), 1, fp);
	scan.ranges.resize(scan_range_size);
	scan.intensities.resize(scan_range_size);

	for (int i = 0; i < poses.size(); i++)
	{
		for (int j = 0; j < scan.ranges.size(); j++)
		{
			ret_val = fread(&scan.ranges[j], sizeof(float), 1, fp);
			ret_val = fread(&scan.intensities[j], sizeof(float), 1, fp);
		}
		double xo = poses[i].x;
		double yo = poses[i].y;
		double yawo = poses[i].yaw;
/*
		double xo = raw_poses[i].x;
		double yo = raw_poses[i].y;
		double yawo = raw_poses[i].yaw;
 */
		for (int j = 0; j < scan.ranges.size(); j++)
		{
			double range = scan.ranges[j];
			if (range < scan.range_min || scan.range_max < range)
				continue;
			double yaw = yawo + scan.angle_min + scan.angle_increment * (double)j;
			double dx = map.info.resolution * cos(yaw);
			double dy = map.info.resolution * sin(yaw);
			double x = xo;
			double y = yo;
			for (double r = 0.0; r <= range; r += map.info.resolution)
			{
				double dx_ = x - map.info.origin.position.x;
				double dy_ = y - map.info.origin.position.y;
				int u = (int)(dx_ / map.info.resolution);
				int v = (int)(dy_ / map.info.resolution);
				if (0 <= u && u < map.info.width && 0 <= v && v < map.info.height)
					update_cell(u, v, range - r);
				x += dx;
				y += dy;
			}
/*
			x = range * cos(yaw) + xo;
			y = range * sin(yaw) + yo;
			double dx_ = x - map.info.origin.position.x;
			double dy_ = y - map.info.origin.position.y;
			int u = (int)(dx_ / map.info.resolution);
			int v = (int)(dy_ / map.info.resolution);
			if (1 <= u && u < map.info.width - 1 && 1 <= v && v < map.info.height - 1)
			{
				update_cell(u, v, 0.0);
				update_cell(u - 1, v, 0.0);
				update_cell(u + 1, v, 0.0);
				update_cell(u, v - 1, 0.0);
				update_cell(u, v + 1, 0.0);
			}
 */
		}
	}
	fclose(fp);




printf("start spin\n");

	map_pub.publish(map);
	ros::spin();
}

void Mapping2d::update_cell(int u, int v, double dr)
{
	double z_hit = 0.90;
	double z_max = 0.05;
	double max_dist_prob = 0.043937;
	double z_hit_denom = 0.08;
	double z_rand = 0.05;
	double z_rand_mult = 0.033333;
	int node = v * map.info.width + u;
	int val = map.data[node];
	double po;
	if (val == -1)
		po = 0.5;
	else
		po = (double)val / 100.0;
	double z = dr;
	double pz = z_hit * exp(-(z * z) / z_hit_denom) + z_hit * max_dist_prob + z_rand * z_rand_mult;
	if (pz > 1.0)
		pz = 1.0;
	double l = log(po / (1.0 - po)) + log(pz / (1.0 - pz));
	double p = 1.0 / (1.0 + exp(-l));
	if (p < 0.01)
		p = 0.01;
	if (val > 0.99)
		val = 0.99;
	val = (int)(p * 100.0);
	map.data[node] = val;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mapping_2d");
	Mapping2d node;
	return 0;
}
