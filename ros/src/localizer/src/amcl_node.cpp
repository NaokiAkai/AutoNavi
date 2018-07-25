// Copyright © 2018 Naoki Akai. All rights reserved.

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
    ros::init(argc, argv, "amcl");
    amcl = new AMCL;
    bool use_nav_core_server = false;
    amcl->nh.param("/amcl/use_nav_core_server", use_nav_core_server, use_nav_core_server);
    if (use_nav_core_server)
        amcl->nh.setParam("/nav_params/reach_at_goal", false);
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
        amcl->check_scan_points_validity(scan);
        if (amcl->use_dspd)
            amcl->evaluate_particles_with_dspd(scan);
        else if (amcl->use_beam_model)
            amcl->evaluate_particles_using_beam_model(scan);
        else
            amcl->evaluate_particles_using_likelihood_field_model(scan);
        amcl->compute_total_weight_and_effective_sample_size();
        amcl->estimate_robot_pose();
        amcl->compute_random_particle_rate();
        double d_time = curr_time - prev_time;
        if (fabs(amcl->delta_dist) >= amcl->update_dist || fabs(amcl->delta_yaw) >= amcl->update_yaw || d_time >= amcl->update_time)
        {
//            amcl->compute_scan_fractions(amcl->robot_pose, scan, NULL, NULL);
            amcl->resample_particles();
            prev_time = curr_time;
            amcl->delta_dist = amcl->delta_yaw = 0.0;
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
//    std::string fname("/tmp/amcl_map.txt");
//    amcl->save_map_as_txt_file(fname);
    return 0;
}
