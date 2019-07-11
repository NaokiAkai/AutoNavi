/****************************************************************************
 * Copyright (C) 2018 Naoki Akai.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 ****************************************************************************/

#ifndef __AMCL_H__
#define __AMCL_H__

#include <vector>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

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

public:
    ros::NodeHandle nh;
    double timer_start, timer_end;
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
    bool use_omni_odom;
    double start_x, start_y, start_yaw;
    double initial_cov_xx, initial_cov_yy, initial_cov_yawyaw;
    double pose_publish_hz;
    pose_t robot_pose, base_link2laser;
    std::vector<particle_t> particles;
    std::vector<bool> is_valid_scan_points;
    int max_particle_likelihood_num;
    sensor_msgs::PointCloud dynamic_scan_points;
    nav_msgs::OccupancyGrid map;
//    std::vector<std::vector<float> > dist_map;
    cv::Mat dist_map;
    double effective_sample_size, total_weight, random_particle_rate, w_avg, w_slow, w_fast;
    bool use_kld_sampling, use_test_range_measurement, add_random_particle;
    double add_random_particle_rate;
    double dynamic_scan_point_threshold;
    bool is_map_data, is_scan_data, is_first_time, is_tf_initialized, is_initial_pose;
    ros::Publisher pose_pub, particles_pub, uscan_pub, dscan_pub, upoints_pub, dpoints_pub, likelihood_dist_map_pub, matching_error_pub, expected_distances_pub;
    ros::Subscriber pose_sub, map_sub, odom_sub, scan_sub;
    nav_msgs::Odometry curr_odom;
    sensor_msgs::LaserScan curr_scan;
    double z_hit, z_short, z_max, z_rand;
    double z_hit_var, lambda_short, max_dist_prob, z_rand_mult;
    double norm_const_hit;
    bool use_beam_model, use_class_conditional_observation_model;
    std::vector<double> unknown_probs;
    std::vector<std::vector<double> > measurement_classes;
    tf::TransformListener tf_listener;

    AMCL();
    ~AMCL() {};
    double nrand(double n);
    void start_timer(void);
    void stop_timer(void);
    double get_timer_data(void);
    void xy2uv(double x, double y, int* u, int* v);
    void uv2xy(int u, int v, double* x, double* y);
    void reset_particles(void);
    void amcl_init(void);
    bool get_pose_from_tf(std::string source_frame, std::string target_frame, ros::Time target_time, double duration, pose_t* pose, ros::Time *stamp);
    void initial_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void save_map_as_txt_file(std::string fname);
    void broadcast_tf(void);
    void publish_pose(void);
    void publish_particles(void);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void update_particle_pose_by_odom(void);
    void check_scan_points_validity(sensor_msgs::LaserScan scan);
    double compute_weight_using_likelihood_field_model(pose_t pose, sensor_msgs::LaserScan scan);
    void evaluate_particles_using_likelihood_field_model(sensor_msgs::LaserScan scan);
    double compute_weight_using_beam_model(pose_t pose, sensor_msgs::LaserScan scan);
    void evaluate_particles_using_beam_model(sensor_msgs::LaserScan scan);
    double compute_weight_using_class_conditional_observation_model(pose_t pose, sensor_msgs::LaserScan scan, bool use_all_scan);
    void evaluate_particles_using_class_conditional_observation_model(sensor_msgs::LaserScan scan);
    void compute_total_weight_and_effective_sample_size(void);
    void compute_random_particle_rate(void);
    void estimate_robot_pose(void);
    void resample_particles(void);
    void reset_moving_amounts(void);
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void plot_likelihood_distribution(pose_t pose, sensor_msgs::LaserScan scan);
    void publish_likelihood_distribution_map_vis(double range, double reso, sensor_msgs::LaserScan scan);
    void compute_scan_fractions(pose_t pose, sensor_msgs::LaserScan scan, double* valid_scan, double* matched_scan);
    void publish_residual_errors_as_laser_scan(pose_t pose, sensor_msgs::LaserScan scan);
    std::vector<double> get_residual_errors_as_std_vector(pose_t pose, sensor_msgs::LaserScan scan, int scan_step);
    void publish_expected_map_distances_as_laser_scan(pose_t pose);
};

#endif /* __AMCL_H__ */
