#ifndef __NDT_H__
#define __NDT_H__

#include <vector>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>

typedef struct
{
	int points_num;
	float mean_x, mean_y;
	float var_xx, var_xy, var_yy;
	double occupancy_rate;
} ndt_t;

class NDT
{
private:
	float get_dist_from_point_to_mean(float x, float y, int node);
	void update_occupancy_rate_of_ndt_map(int node, float dr);
	void update_ndt_grid(float x, float y, int node);
	double get_mahalanobis_distance(double dx, double dy, int node);

public:
	int ndt_map_width, ndt_map_height;
	float ndt_map_size_x, ndt_map_size_y, ndt_map_grid_size;
	int ndt_map_grid_num;
	float ndt_map_origin_x, ndt_map_origin_y;
	int ndt_map_min_points_num;
	double occupancy_rate_threshold;
	std::vector<ndt_t> ndt_map;

	NDT();
	~NDT() {};
	double nrand(double n);
	bool save_ndt_map(std::string map_file_name);
	bool read_ndt_map(std::string map_file_name);
	void set_min_points_num(int min_points_num);
	void set_occupancy_rate_threshold(double threshold);
	void init_ndt_map(float map_size_x, float map_size_y, float grid_size, float origin_x, float origin_y);
	void xy2uv_layer1(float x, float y, int* u, int* v);
	void xy2uv_layer2(float x, float y, int* u, int* v);
	int uv2node_layer1(int u, int v);
	int uv2node_layer2(int u, int v);
	void decrease_occupancy_rate_of_ndt_map(float x, float y, float dr);
	void add_point_to_ndt_map(float x, float y);
	sensor_msgs::PointCloud get_mean_points_of_ndt_map(std::string frame_id, float dist, float xo, float yo);
	visualization_msgs::Marker get_grid_lines_of_ndt_map(std::string frame_id);
	visualization_msgs::MarkerArray get_ellipses_of_ndt_map(std::string frame_id, float dist, float xo, float yo);
	double compute_probability(float x, float y);
};

#endif /* __NDT_H__ */
