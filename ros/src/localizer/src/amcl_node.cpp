/****************************************************************************
 * Copyright (C) 2018 Naoki Akai.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 ****************************************************************************/

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <localizer/amcl.h>

AMCL* amcl;

void wait_for_new_map(void)
{
    bool finish_navigation = false;
    amcl->is_map_data = amcl->is_initial_pose = false;
    amcl->nh.setParam("/nav_params/reach_at_goal", false);
    amcl->nh.setParam("/nav_params/request_new_map", true);
    ros::Rate loop_rate(10.0);
    while (ros::ok())
    {
        ros::spinOnce();
        amcl->broadcast_tf();
        amcl->publish_pose();
        amcl->publish_particles();
        if (amcl->is_map_data && amcl->is_initial_pose)
        {
            amcl->nh.setParam("/nav_params/is_new_map_data", true);
            break;
        }
        amcl->nh.getParam("/nav_params/finish_navigation", finish_navigation);
        if (finish_navigation)
        {
            printf("finish navigation from amcl_node\n");
            exit(0);
        }
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    // initialize and read parameters
    ros::init(argc, argv, "amcl");
    amcl = new AMCL;
    bool use_nav_core_server = false;
    amcl->nh.param("use_nav_core_server", use_nav_core_server, use_nav_core_server);
    if (use_nav_core_server)
        amcl->nh.setParam("/nav_params/reach_at_goal", false);
    bool plot_likelihood_distribution = false;
    amcl->nh.param("plot_likelihood_distribution", plot_likelihood_distribution, plot_likelihood_distribution);

    // main loop
    ros::Rate loop_rate(amcl->pose_publish_hz);
    double prev_time = 0.0;
    while (ros::ok())
    {
        ros::spinOnce();
        // perform localization
        amcl->update_particle_pose_by_odom();
        if (!amcl->is_map_data || !amcl->is_scan_data || !amcl->is_tf_initialized)
        {
            ROS_ERROR("initialization is not finished yet");
            continue;
        }
        sensor_msgs::LaserScan scan = amcl->curr_scan;
        double curr_time = scan.header.stamp.toSec();
        amcl->start_timer();
        amcl->check_scan_points_validity(scan);
        if (amcl->use_class_conditional_observation_model)
            amcl->evaluate_particles_with_class_conditional_observation_model(scan);
        else if (amcl->use_beam_model)
            amcl->evaluate_particles_using_beam_model(scan);
        else
            amcl->evaluate_particles_using_likelihood_field_model(scan);
        amcl->stop_timer();
        amcl->compute_total_weight_and_effective_sample_size();
        amcl->estimate_robot_pose();
        amcl->compute_random_particle_rate();
        double d_time = curr_time - prev_time;
        if (fabs(amcl->delta_dist) >= amcl->update_dist || fabs(amcl->delta_yaw) >= amcl->update_yaw || d_time >= amcl->update_time)
        {
//            amcl->compute_scan_fractions(amcl->robot_pose, scan, NULL, NULL);
            amcl->resample_particles();
            prev_time = curr_time;
            amcl->reset_moving_amounts();
            printf("x = %.3lf [m], y = %.3lf [m], yaw = %.3lf [deg]\n", amcl->robot_pose.x, amcl->robot_pose.y, amcl->robot_pose.yaw * 180.0 / M_PI);
            printf("particle_num = %d\n", amcl->particle_num);
            printf("effective_sample_size = %lf, total_weight = %lf\n", amcl->effective_sample_size, amcl->total_weight);
            printf("random_particle_rate = %lf\n", amcl->random_particle_rate);
            printf("estimation_time = %lf [msec]\n", amcl->get_timer_data() * 1000.0);
            printf("\n");
        }
        // publish localization result messages
        amcl->broadcast_tf();
        amcl->publish_pose();
        amcl->publish_particles();
        amcl->publish_residual_errors_as_laser_scan(amcl->robot_pose, scan);
         amcl->publish_expected_map_distances_as_laser_scan(amcl->robot_pose);
        // visualize likelihood distribution using gnuplot
        if (plot_likelihood_distribution)
            amcl->plot_likelihood_distribution(amcl->robot_pose, scan);
        // for cmmunication with nav_core_server
        if (use_nav_core_server)
        {
            bool reach_at_goal;
            amcl->nh.getParam("/nav_params/reach_at_goal", reach_at_goal);
            if (reach_at_goal)
                wait_for_new_map();
        }
        loop_rate.sleep();
    }
    if (plot_likelihood_distribution)
        int val = system("killall -9 gnuplot");
    return 0;
}
