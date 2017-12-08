#ifndef __AMCL_H__
#define __AMCL_H__

#include <vector>

typedef struct
{
	double x, y, yaw;
} pose_t;

typedef struct
{
	pose_t pose;
	double w;
} particle_t;

class AMCL
{
private:
	ros::NodeHandle nh;

public:
	std::string map_frame, laser_frame, base_link_frame;
	std::string input_map_topic_name, input_odom_topic_name, input_scan_topic_name;
	int particle_num, min_particle_num, max_particle_num;
	double resample_threshold;
	int scan_step;
	double max_dist_to_obstacle;
	double alpha_slow, alpha_fast;
	double delta_dist, delta_yaw;
	double update_dist, update_yaw, update_time;
	double odom_noise_dist_dist, odom_noise_dist_head, odom_noise_head_dist, odom_noise_head_head;
	double start_x, start_y, start_yaw;
	double initial_cov_xx, initial_cov_yy, initial_cov_yawyaw;
	double pose_publish_hz;
	pose_t robot_pose, base_link2laser;
	std::vector<particle_t> particles;
	std::vector<bool> is_valid_scan_points;
	int max_particle_likelihood_num;
	sensor_msgs::PointCloud dynamic_scan_points;
	nav_msgs::OccupancyGrid map;
	std::vector<std::vector<float> > dist_map;
	double effective_sample_size, total_weight, random_particle_rate, w_avg, w_slow, w_fast;
	bool use_kld_sampling, use_test_range_measurement;
	double dynamic_scan_point_threshold;
	bool is_map_data, is_scan_data, is_first_time, is_tf_initialized;
	ros::Publisher pose_pub, particles_pub, upoints_pub, dpoints_pub;
	ros::Subscriber pose_sub, map_sub, odom_sub, scan_sub;
	nav_msgs::Odometry curr_odom;
	sensor_msgs::LaserScan curr_scan;

	AMCL();
	double nrand(double n);
	void xy2uv(double x, double y, int* u, int* v);
	void reset_particles(void);
	void amcl_init(void);
	void initial_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void broadcast_tf(void);
	void publish_pose(void);
	void publish_particles(void);
	void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
	void update_particle_pose_by_odom(void);
	void check_scan_points_validity(sensor_msgs::LaserScan scan);
	void evaluate_particles(sensor_msgs::LaserScan scan);
	void compute_total_weight_and_effective_sample_size(void);
	void compute_random_particle_rate(void);
	void estimate_robot_pose(void);
	void resample_particles(void);
	void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
};

#endif /* __AMCL_H__ */
