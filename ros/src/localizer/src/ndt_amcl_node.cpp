// Copyright © 2018 Naoki Akai. All rights reserved.

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <localizer/amcl.h>
#include <localizer/ndt.h>

AMCL* amcl;
NDT* ndt;
std::vector<std::vector<double> > dsp_probs;
tf::TransformListener* tf_listener;
ros::Publisher mean_points_pub, grid_lines_pub, ellipses_pub;

void *map_data_publisher(void* obj)
{
    visualization_msgs::Marker grid_lines = ndt->get_grid_lines_of_ndt_map(amcl->map_frame);
    // main loop
    ros::Rate loop_rate(1.0);
    while (ros::ok())
    {
        sensor_msgs::PointCloud mean_points = ndt->get_mean_points_of_ndt_map(amcl->map_frame, 50.0f, amcl->robot_pose.x, amcl->robot_pose.y);
        visualization_msgs::MarkerArray ellipses = ndt->get_ellipses_of_ndt_map(amcl->map_frame, 50.0f, amcl->robot_pose.x, amcl->robot_pose.y);
        mean_points_pub.publish(mean_points);
        grid_lines_pub.publish(grid_lines);
        ellipses_pub.publish(ellipses);
        loop_rate.sleep();
    }
}

void evaluate_particles_using_ndt_map(sensor_msgs::LaserScan scan)
{
    double z_hit = 0.90;
    double max_dist_prob = 0.043937;
    double z_max = 0.05;
    double z_hit_denom = 0.08;
    double z_rand = 0.05;
    double z_rand_mult = 0.033333;
    double max;
    for (int i = 0; i < amcl->particle_num; i++)
    {
        double w = 0.0;
        double c = cos(amcl->particles[i].pose.yaw);
        double s = sin(amcl->particles[i].pose.yaw);
        double xo = amcl->base_link2laser.x * c - amcl->base_link2laser.y * s + amcl->particles[i].pose.x;
        double yo = amcl->base_link2laser.x * s + amcl->base_link2laser.y * c + amcl->particles[i].pose.y;
        for (int j = 0; j < scan.ranges.size(); j += amcl->scan_step)
        {
            if (!amcl->is_valid_scan_points[j])
            {
                double pz = z_max * max_dist_prob + z_rand * z_rand_mult;
                w += log(pz);
                continue;
            }
            double angle = scan.angle_min + scan.angle_increment * (double)j;
            double yaw = angle + amcl->particles[i].pose.yaw;
            double r = scan.ranges[j];
            float x = (float)(r * cos(yaw) + xo);
            float y = (float)(r * sin(yaw) + yo);
            double pz = 0.0;
            double z = ndt->compute_probability(x, y);
            if (z < 0.0)
                z = 0.0;
            if (z > 1.0)
                z = 1.0;
            pz += z_hit * z + z_max * max_dist_prob + z_rand * z_rand_mult;
            if (pz > 1.0)
                pz = 1.0;
            w += log(pz);
        }
        double weight = exp(w);
        amcl->particles[i].w *= weight;
        if (i == 0)
        {
            amcl->max_particle_likelihood_num = i;
            max = amcl->particles[i].w;
        }
        else
        {
            if (max < amcl->particles[i].w)
            {
                amcl->max_particle_likelihood_num = i;
                max = amcl->particles[i].w;
            }
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "amcl");
    ros::NodeHandle nh("~");
    // read parameters
    std::string map_file_name = "/tmp/ndt_map.txt";
    int min_points_num = 20;
    double occupancy_rate_threshold = 0.65f;
    nh.param("/amcl/map_file_name", map_file_name, map_file_name);
    nh.param("/amcl/min_points_num", min_points_num, min_points_num);
    nh.param("/amcl/occupancy_rate_threshold", occupancy_rate_threshold, occupancy_rate_threshold);
    // publisher
    mean_points_pub = nh.advertise<sensor_msgs::PointCloud>("/ndt_map_mean_points", 10);
    grid_lines_pub = nh.advertise<visualization_msgs::Marker>("/ndt_map_grid_lines", 10);
    ellipses_pub = nh.advertise<visualization_msgs::MarkerArray>("/ndt_map_ellipses", 10);
    // initialization
    amcl = new AMCL;
    ndt = new NDT;
    tf_listener = new tf::TransformListener;
    if (!ndt->read_ndt_map(map_file_name))
    {
        ROS_ERROR("a map file is not given");
        exit(1);
    }
    ndt->set_min_points_num(min_points_num);
    ndt->set_occupancy_rate_threshold(occupancy_rate_threshold);
    // map data publisher will be parallely executed
    pthread_t tid;
    pthread_create(&tid, NULL, map_data_publisher, NULL);
    // main loop
    ros::Rate loop_rate(amcl->pose_publish_hz);
    double prev_time = 0.0;
    while (ros::ok())
    {
        ros::spinOnce();
        // perform localization
        amcl->update_particle_pose_by_odom();
        if (!amcl->is_scan_data || !amcl->is_tf_initialized)
        {
            ROS_ERROR("initialization is not finished yet");
            continue;
        }
        sensor_msgs::LaserScan scan = amcl->curr_scan;
        double curr_time = scan.header.stamp.toSec();;
        amcl->check_scan_points_validity(scan);
        evaluate_particles_using_ndt_map(scan);
        amcl->compute_total_weight_and_effective_sample_size();
        amcl->estimate_robot_pose();
        amcl->compute_random_particle_rate();
        double d_time = curr_time - prev_time;
        bool do_mapping = false;
        if (fabs(amcl->delta_dist) >= amcl->update_dist || fabs(amcl->delta_yaw) >= amcl->update_yaw || d_time >= amcl->update_time)
        {
            amcl->resample_particles();
            prev_time = curr_time;
            amcl->delta_dist = amcl->delta_yaw = 0.0;
            prev_time = curr_time;
            do_mapping = true;
            printf("x = %.3lf [m], y = %.3lf [m], yaw = %.3lf [deg]\n", amcl->robot_pose.x, amcl->robot_pose.y, amcl->robot_pose.yaw * 180.0 / M_PI);
            printf("particle_num = %d\n", amcl->particle_num);
            printf("effective_sample_size = %lf, total_weight = %lf\n", amcl->effective_sample_size, amcl->total_weight);
            printf("random_particle_rate = %lf\n", amcl->random_particle_rate);
            printf("\n");
        }
        // publish localization result messages
        amcl->broadcast_tf();
        amcl->publish_pose();
        amcl->publish_particles();
        // sleep
        loop_rate.sleep();
    }
    return 0;
}
