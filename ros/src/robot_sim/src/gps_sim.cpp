/****************************************************************************
 * Copyright (C) 2018 Naoki Akai.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 ****************************************************************************/

#include <ros/ros.h>
#include <time.h>
#include <std_msgs/Time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <robot_sim/common.h>

class GPSSim
{
private:
    ros::NodeHandle nh;
    ros::Publisher pose_pub;
    std::string output_pose_topic_name;
    std::string map_frame;
    ros::Time robot_pose_stamp;
    pose_t robot_pose;
    tf::TransformListener tf_listener;
    double gps_noise_x, gps_noise_y;
    double gps_cov_xx, gps_cov_yy;
    double pose_publish_hz;

public:
    GPSSim();
    ~GPSSim() {};
    bool get_ground_truth_robot_pose(void);

    inline double nrand(double n)
    {
        return (n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX));
    }

    void publish_pose(void);
};

GPSSim::GPSSim():
    nh("~"),
    output_pose_topic_name("/gps_pose"),
    map_frame("/world"),
    gps_noise_x(1.0),
    gps_noise_y(1.0),
    gps_cov_xx(1.0),
    gps_cov_yy(1.0),
    pose_publish_hz(10.0)
{
    // initialize random values
    srand((unsigned)time(NULL));
    // read parameters
    nh.param("output_pose_topic_name", output_pose_topic_name, output_pose_topic_name);
    nh.param("gps_noise_x", gps_noise_x, gps_noise_x);
    nh.param("gps_noise_y", gps_noise_y, gps_noise_y);
    nh.param("gps_cov_xx", gps_cov_xx, gps_cov_xx);
    nh.param("gps_cov_yy", gps_cov_yy, gps_cov_yy);
    // Publisher
    pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(output_pose_topic_name, 100);
    // main loop
    ros::Rate loop_rate(pose_publish_hz);
    while (ros::ok())
    {
        ros::spinOnce();
        if (get_ground_truth_robot_pose())
            publish_pose();
        loop_rate.sleep();
    }
}

bool GPSSim::get_ground_truth_robot_pose(void)
{
    tf::StampedTransform map2base_link;
    ros::Time now = ros::Time::now();
    try
    {
        tf_listener.waitForTransform(map_frame, "/ground_truth", now, ros::Duration(0.1));
        tf_listener.lookupTransform(map_frame, "/ground_truth", now, map2base_link);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return false;
    }
    robot_pose_stamp = map2base_link.stamp_;
    robot_pose.x = map2base_link.getOrigin().x();
    robot_pose.y = map2base_link.getOrigin().y();
    tf::Quaternion q(map2base_link.getRotation().x(),
        map2base_link.getRotation().y(),
        map2base_link.getRotation().z(),
        map2base_link.getRotation().w());
    double roll, pitch;
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, robot_pose.yaw);
    return true;
}

void GPSSim::publish_pose(void)
{
    geometry_msgs::PoseWithCovarianceStamped gps_pose;
    gps_pose.header.stamp = ros::Time::now();
    gps_pose.header.frame_id = map_frame;
    gps_pose.pose.pose.position.x = robot_pose.x + nrand(gps_noise_x);
    gps_pose.pose.pose.position.y = robot_pose.y + nrand(gps_noise_y);
    gps_pose.pose.pose.position.z = 0.0;
    gps_pose.pose.pose.orientation.x = 0.0;
    gps_pose.pose.pose.orientation.y = 0.0;
    gps_pose.pose.pose.orientation.z = 0.0;
    gps_pose.pose.pose.orientation.w = 1.0;
    gps_pose.pose.covariance[0] = gps_cov_xx;
    gps_pose.pose.covariance[7] = gps_cov_yy;
    pose_pub.publish(gps_pose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_sim");
    GPSSim node;
    return 0;
}
