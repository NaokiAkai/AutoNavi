#include <stdio.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <localizer/ndt.h>

NDT::NDT():
	ndt_map_min_points_num(20),
	occupancy_rate_threshold(0.65)
{

}

double NDT::nrand(double n)
{
	return (n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX));
}

bool NDT::save_ndt_map(std::string map_file_name)
{
	FILE* fp = fopen(map_file_name.c_str(), "w");
	if (fp == NULL)
	{
		fprintf(stderr, "cannot open file -> %s\n", map_file_name.c_str());
		return false;
	}
	fprintf(fp, "map_size_x,map_size_y,grid_size,origin_x,origin_y\n");
	fprintf(fp, "%f %f %f %f %f\n", ndt_map_size_x, ndt_map_size_y, ndt_map_grid_size, ndt_map_origin_x, ndt_map_origin_y);
	fprintf(fp, "node,points_num,mean_x,mean_y,var_xx,var_xy,var_yy,occupancy_rate\n");
	for (int i = 0; i < ndt_map_grid_num; i++)
	{
		if (ndt_map[i].points_num >= ndt_map_min_points_num && ndt_map[i].occupancy_rate >= occupancy_rate_threshold)
		{
			fprintf(fp, "%d %d %f %f %f %f %f %lf\n", i, ndt_map[i].points_num,
				ndt_map[i].mean_x, ndt_map[i].mean_y, ndt_map[i].var_xx, ndt_map[i].var_xy, ndt_map[i].var_yy, ndt_map[i].occupancy_rate);
		}
	}
	fclose(fp);
	return true;
}

bool NDT::read_ndt_map(std::string map_file_name)
{
	FILE* fp = fopen(map_file_name.c_str(), "r");
	if (fp == NULL)
	{
		fprintf(stderr, "cannot open file -> %s\n", map_file_name.c_str());
		return false;
	}
	int val;
	char string[1024];
	val = fscanf(fp, "%s", string);
	val = fscanf(fp, "%f %f %f %f %f\n", &ndt_map_size_x, &ndt_map_size_y, &ndt_map_grid_size, &ndt_map_origin_x, &ndt_map_origin_y);
	init_ndt_map(ndt_map_size_x, ndt_map_size_y, ndt_map_grid_size, ndt_map_origin_x, ndt_map_origin_y);
	val = fscanf(fp, "%s", string);
	int node;
	ndt_t ndt;
	while (fscanf(fp, "%d %d %f %f %f %f %f %lf", &node, &ndt.points_num, &ndt.mean_x, &ndt.mean_y, &ndt.var_xx, &ndt.var_xy, &ndt.var_yy, &ndt.occupancy_rate) != EOF)
		ndt_map[node] = ndt;
	fclose(fp);
	return true;
}

void NDT::init_ndt_map(float map_size_x, float map_size_y, float grid_size, float origin_x, float origin_y)
{
	ndt_map_size_x = map_size_x;
	ndt_map_size_y = map_size_y;
	ndt_map_width = (int)(map_size_x / grid_size);
	ndt_map_height = (int)(map_size_y / grid_size);
	ndt_map_grid_size = grid_size;
	ndt_map_grid_num = ndt_map_width * ndt_map_height * 2;
	ndt_map_origin_x = origin_x;
	ndt_map_origin_y = origin_y;
	ndt_t ndt;
	ndt.mean_x = ndt.mean_y = 0.0f;
	ndt.var_xx = ndt.var_xy = ndt.var_yy = 0.0f;
	ndt.occupancy_rate = -1.0;
	ndt_map.resize(ndt_map_grid_num, ndt);
}

void NDT::set_min_points_num(int min_points_num)
{
	ndt_map_min_points_num = min_points_num;
}

void NDT::set_occupancy_rate_threshold(double threshold)
{
	occupancy_rate_threshold = threshold;
}

void NDT::xy2uv_layer1(float x, float y, int* u, int* v)
{
	float dx = x - ndt_map_origin_x;
	float dy = y - ndt_map_origin_y;
	*u = (int)(dx / ndt_map_grid_size);
	*v = (int)(dy / ndt_map_grid_size);
}

void NDT::xy2uv_layer2(float x, float y, int* u, int* v)
{
	float dx = x - (ndt_map_origin_x + ndt_map_grid_size / 2.0);
	float dy = y - (ndt_map_origin_y + ndt_map_grid_size / 2.0);
	*u = (int)(dx / ndt_map_grid_size);
	*v = (int)(dy / ndt_map_grid_size);
}

int NDT::uv2node_layer1(int u, int v)
{
	return (v * ndt_map_width + u);
}

int NDT::uv2node_layer2(int u, int v)
{
	return ((v * ndt_map_width + u) + (ndt_map_width * ndt_map_height));
}

float NDT::get_dist_from_point_to_mean(float x, float y, int node)
{
	float dx = x - ndt_map[node].mean_x;
	float dy = y - ndt_map[node].mean_y;
	return sqrt(dx * dx + dy * dy);
}

void NDT::update_occupancy_rate_of_ndt_map(int node, float dr)
{
	double z_hit = 0.95;
//	double z_hit_denom = 0.08;
	double z_hit_denom = ndt_map_grid_size;
	double z_rand = 0.05;
	double z_rand_mult = 0.033333;
	double po = ndt_map[node].occupancy_rate;
	double z = (double)dr;
	double pz = 0.0;
	pz += z_hit * exp(-(z * z) / z_hit_denom);
	pz += z_rand * z_rand_mult;
	if (pz < 0.001)	pz = 0.001;
	if (pz > 0.999)	pz = 0.999;
	double l = log(po / (1.0 - po)) + log(pz / (1.0 - pz));
	double p = 1.0 / (1.0 + exp(-l));
	if (p < 0.001)	p = 0.001;
	if (p > 0.999)	p = 0.999;
	ndt_map[node].occupancy_rate = p;
}

void NDT::decrease_occupancy_rate_of_ndt_map(float x, float y, float dr)
{
	int u, v;
	xy2uv_layer1(x, y, &u, &v);
	if (0 <= u && u < ndt_map_width && 0 <= v && v < ndt_map_height)
	{
		int node = uv2node_layer1(u, v);
		float dl = get_dist_from_point_to_mean(x, y, node);
		if (dl < ndt_map_grid_size / 4.0f)
		{
			if (ndt_map[node].occupancy_rate < 0.0)
				ndt_map[node].occupancy_rate = 0.5;
			else
				update_occupancy_rate_of_ndt_map(node, dr);
		}
	}
	xy2uv_layer2(x, y, &u, &v);
	if (0 <= u && u < ndt_map_width && 0 <= v && v < ndt_map_height)
	{
		int node = uv2node_layer2(u, v);
		float dl = get_dist_from_point_to_mean(x, y, node);
		if (dl < ndt_map_grid_size / 4.0f)
		{
			if (ndt_map[node].occupancy_rate < 0.0)
				ndt_map[node].occupancy_rate = 0.5;
			else
				update_occupancy_rate_of_ndt_map(node, dr);
		}
	}
}

void NDT::update_ndt_grid(float x, float y, int node)
{
	if (ndt_map[node].points_num == 0)
	{
		ndt_map[node].mean_x = x;
		ndt_map[node].mean_y = y;
		ndt_map[node].points_num++;
		if (ndt_map[node].occupancy_rate < 0.0)
			ndt_map[node].occupancy_rate = 0.5;
		else
			update_occupancy_rate_of_ndt_map(node, 0.0f);
	}
	else
	{
		// sequential update of mean and variance
		// https://mathwords.net/tikujiheikin
		int n = ndt_map[node].points_num;
		float a = 1.0f / ((float)n + 1.0f);
		float mean_x = ndt_map[node].mean_x;
		float mean_y = ndt_map[node].mean_y;
		float var_xx = ndt_map[node].var_xx;
		float var_xy = ndt_map[node].var_xy;
		float var_yy = ndt_map[node].var_yy;
		ndt_map[node].mean_x = (1.0f - a) * mean_x + a * x;
		ndt_map[node].mean_y = (1.0f - a) * mean_y + a * y;
		ndt_map[node].var_xx = (1.0f - a) * var_xx + a * (1.0f - a) * (x - mean_x) * (x - mean_x);
		ndt_map[node].var_xy = (1.0f - a) * var_xy + a * (1.0f - a) * (x - mean_x) * (y - mean_y);
		ndt_map[node].var_yy = (1.0f - a) * var_yy + a * (1.0f - a) * (y - mean_y) * (y - mean_y);
		update_occupancy_rate_of_ndt_map(node, 0.0f);
		ndt_map[node].points_num++;
	}
}

void NDT::add_point_to_ndt_map(float x, float y)
{
	int u, v;
	xy2uv_layer1(x, y, &u, &v);
	if (0 <= u && u < ndt_map_width && 0 <= v && v < ndt_map_height)
	{
		int node = uv2node_layer1(u, v);
		update_ndt_grid(x, y, node);
	}
	xy2uv_layer2(x, y, &u, &v);
	if (0 <= u && u < ndt_map_width && 0 <= v && v < ndt_map_height)
	{
		int node = uv2node_layer2(u, v);
		update_ndt_grid(x, y, node);
	}
}

sensor_msgs::PointCloud NDT::get_mean_points_of_ndt_map(std::string frame_id, float dist, float xo, float yo)
{
	sensor_msgs::PointCloud points;
	points.header.frame_id = frame_id;
	for (int i = 0; i < ndt_map_grid_num; i++)
	{
		if (ndt_map[i].points_num >= ndt_map_min_points_num && ndt_map[i].occupancy_rate >= occupancy_rate_threshold)
		{
			if (dist < 0.0)
			{
				geometry_msgs::Point32 p;
				p.x = ndt_map[i].mean_x;
				p.y = ndt_map[i].mean_y;
				p.z = 0.0;
				points.points.push_back(p);
			}
			else
			{
				float dx = xo - ndt_map[i].mean_x;
				float dy = yo - ndt_map[i].mean_y;
				float dl = sqrt(dx * dx + dy * dy);
				if (dl <= dist)
				{
					geometry_msgs::Point32 p;
					p.x = ndt_map[i].mean_x;
					p.y = ndt_map[i].mean_y;
					p.z = 0.0;
					points.points.push_back(p);
				}
			}
		}
	}
	points.header.stamp = ros::Time::now();
	return points;
}

visualization_msgs::Marker NDT::get_grid_lines_of_ndt_map(std::string frame_id)
{
	visualization_msgs::Marker lines;
	lines.header.stamp = ros::Time::now();
	lines.header.frame_id = frame_id;
	lines.ns = "ndt_map_grid_lines";
	lines.id = 0;
	lines.type = visualization_msgs::Marker::LINE_LIST;
	lines.action = visualization_msgs::Marker::ADD;
	lines.pose.orientation.w = 1.0;
	lines.scale.x = 0.01;
	lines.color.r = 1.0;
	lines.color.g = 1.0;
	lines.color.b = 1.0;
	lines.color.a = 1.0;
	for (float x = ndt_map_origin_x; x <= ndt_map_origin_x + ndt_map_size_x; x += ndt_map_grid_size)
	{
		geometry_msgs::Point p;
		p.x = x;
		p.y = ndt_map_origin_y;
		p.z = 0.0;
		lines.points.push_back(p);
		p.y = ndt_map_origin_y + ndt_map_size_y;
		lines.points.push_back(p);
	}
	for (float y = ndt_map_origin_y; y <= ndt_map_origin_y + ndt_map_size_y; y += ndt_map_grid_size)
	{
		geometry_msgs::Point p;
		p.x = ndt_map_origin_x;
		p.y = y;
		p.z = 0.0;
		lines.points.push_back(p);
		p.x = ndt_map_origin_x + ndt_map_size_x;
		lines.points.push_back(p);
	}
	return lines;
}

visualization_msgs::MarkerArray NDT::get_ellipses_of_ndt_map(std::string frame_id, float dist, float xo, float yo)
{
	visualization_msgs::MarkerArray ellipses;
	int id = 0;
	for (int i = 0; i < ndt_map_grid_num; i++)
	{
		if (ndt_map[i].points_num >= ndt_map_min_points_num && ndt_map[i].occupancy_rate >= occupancy_rate_threshold)
		{
			if (dist >= 0.0f)
			{
				float dx = xo - ndt_map[i].mean_x;
				float dy = yo - ndt_map[i].mean_y;
				float dl = sqrt(dx * dx + dy * dy);
				if (dl > dist)
					continue;
			}
			visualization_msgs::Marker ellipse;
			ellipse.header.stamp = ros::Time::now();
			ellipse.header.frame_id = frame_id;
			ellipse.ns = "ndt_map_ellipses";
			ellipse.id = id;
			ellipse.type = visualization_msgs::Marker::CYLINDER;
			ellipse.action = visualization_msgs::Marker::ADD;
			Eigen::Matrix2d A;
			A << ndt_map[i].var_xx, ndt_map[i].var_xy, ndt_map[i].var_xy, ndt_map[i].var_yy;
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(A);
			float ra, rb, t;
			if (es.info() == 0) // eigen values could not be comupted when es.info() is not 0
			{
				float ev1, ev2, chi2 = 9.21034;
				ev1 = es.eigenvalues()(0);
				ev2 = es.eigenvalues()(1);
				if (ev1 >= 0.0 && ev2 >= 0.0) // check whether eigen values is imaginary
				{
					ra = sqrt(chi2 * ev1);
					rb = sqrt(chi2 * ev2);
					t = atan2(es.eigenvectors()(1, 0), es.eigenvectors()(0, 0));
				}
				else
				{
					continue;
				}
			}
			else
			{
				continue;
			}
			ellipse.pose.orientation = tf::createQuaternionMsgFromYaw(t);
			ellipse.scale.x = ra;
			ellipse.scale.y = rb;
			ellipse.scale.z = 0.1;
			ellipse.color.r = 0.0;
			ellipse.color.g = 0.74902;
			ellipse.color.b = 1.0;
			ellipse.color.a = (0.8 / (1.0 - occupancy_rate_threshold)) * (ndt_map[i].occupancy_rate - occupancy_rate_threshold);
			ellipse.pose.position.x = ndt_map[i].mean_x;
			ellipse.pose.position.y = ndt_map[i].mean_y;
			ellipse.pose.position.z = 0.0;
			id++;
			ellipses.markers.push_back(ellipse);
		}
	}
	return ellipses;
}

double NDT::get_mahalanobis_distance(double dx, double dy, int node)
{
	double var_xx = ndt_map[node].var_xx;
	double var_xy = ndt_map[node].var_xy;
	double var_yy = ndt_map[node].var_yy;
	double det = var_xx * var_yy - var_xy * var_xy;
	if (fabs(det) < 0.00000000000000001)
		return 100000.0;
	double d1 = var_yy * dx - var_xy * dy;
	double d2 = -var_xy * dx + var_xx * dy;
	double d = (d1 * dx + d2 * dy) / det;
	return d;
}

double NDT::compute_probability(float x, float y)
{
	int u, v, n1 = 0, n2 = 0, node1, node2;
	double dx1, dy1, dx2, dy2;
	xy2uv_layer1(x, y, &u, &v);
	if (0 <= u && u < ndt_map_width && 0 <= v && v < ndt_map_height)
	{
		node1 = uv2node_layer1(u, v);
		if (ndt_map[node1].occupancy_rate < occupancy_rate_threshold)
		{
			n1 = 0;
		}
		else
		{
			n1 = ndt_map[node1].points_num;
			dx1 = x - ndt_map[node1].mean_x;
			dy1 = y - ndt_map[node1].mean_y;
		}
	}
	xy2uv_layer2(x, y, &u, &v);
	if (0 <= u && u < ndt_map_width && 0 <= v && v < ndt_map_height)
	{
		node2 = uv2node_layer2(u, v);
		if (ndt_map[node2].occupancy_rate < occupancy_rate_threshold)
		{
			n2 = 0;
		}
		else
		{
			n2 = ndt_map[node2].points_num;
			dx2 = x - ndt_map[node2].mean_x;
			dy2 = y - ndt_map[node2].mean_y;
		}
	}
	if (n1 < ndt_map_min_points_num && n2 < ndt_map_min_points_num)
	{
		return -1.0;
	}
	else if (n1 >= ndt_map_min_points_num && n2 < ndt_map_min_points_num)
	{
		double d = get_mahalanobis_distance(dx1, dy1, node1);
		return exp(-d / 2.0);
	}
	else if (n1 < ndt_map_min_points_num && n2 >= ndt_map_min_points_num)
	{
		double d = get_mahalanobis_distance(dx2, dy2, node2);
		return exp(-d / 2.0);
	}
	else
	{
		double dl1 = sqrt(dx1 * dx1 + dy1 * dy1);
		double dl2 = sqrt(dx2 * dx2 + dy2 * dy2);
		if (dl1 <= dl2)
		{
			double d = get_mahalanobis_distance(dx1, dy1, node1);
			return exp(-d / 2.0);
		}
		else
		{
			double d = get_mahalanobis_distance(dx2, dy2, node2);
			return exp(-d / 2.0);
		}
	}
}
