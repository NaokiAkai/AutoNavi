/****************************************************************************
 * Copyright (C) 2018 Naoki Akai.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 ****************************************************************************/

#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <robot_sim/ScanObjectID.h>
#include <robot_sim/SemanticScan.h>
#include <robot_sim/common.h>

class AddNoise
{
private:
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_sub;
    message_filters::Subscriber<robot_sim::ScanObjectID>* object_ids_sub;
    message_filters::TimeSynchronizer<sensor_msgs::LaserScan, robot_sim::ScanObjectID>* sync;
    ros::Publisher odom_pub, scan_pub, points_pub, semantic_scan_pub;
    std::string input_odom_topic_name, input_scan_topic_name, input_scan_object_ids_topic_name;
    std::string output_odom_topic_name, output_scan_topic_name, output_scan_points_topic_name, output_semantic_scan_topic_name;
    double odom_linear_noise, odom_angular_noise;
    double odom_linear_noise_stationary, odom_angular_noise_stationary;
    double scan_measurement_noise;
    pose_t robot_pose;
    bool is_odom_initialized;

public:
    AddNoise();
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
        const robot_sim::ScanObjectID::ConstPtr& object_ids_msg);
    void get_noisy_semantic_probs(int pre_type, int* type, std::vector<float>* probs);

    inline double nrand(double n)
    {
        return (n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX));
    }

    inline void mod_yaw(double *yaw)
    {
        while (*yaw < -M_PI)    *yaw += 2.0 * M_PI;
        while (*yaw > M_PI)        *yaw -= 2.0 * M_PI;
    }
};

AddNoise::AddNoise():
    nh("~"),
    input_odom_topic_name("/odom_robot_sim"),
    input_scan_topic_name("/scan_robot_sim"),
    input_scan_object_ids_topic_name("/scan_object_ids"),
    output_odom_topic_name("/odom"),
    output_scan_topic_name("/scan"),
    output_scan_points_topic_name("/scan_points"),
    output_semantic_scan_topic_name("/semantic_scan"),
    odom_linear_noise(0.05),
    odom_angular_noise(0.01),
    odom_linear_noise_stationary(0.99),
    odom_angular_noise_stationary(0.99),
    scan_measurement_noise(0.05),
    is_odom_initialized(false)
{
    // read parameters
    nh.param("input_odom_topic_name", input_odom_topic_name, input_odom_topic_name);
    nh.param("input_scan_topic_name", input_scan_topic_name, input_scan_topic_name);
    nh.param("output_odom_topic_name", output_odom_topic_name, output_odom_topic_name);
    nh.param("output_scan_topic_name", output_scan_topic_name, output_scan_topic_name);
    nh.param("output_scan_points_topic_name", output_scan_points_topic_name, output_scan_points_topic_name);
    nh.param("odom_linear_noise", odom_linear_noise, odom_linear_noise);
    nh.param("odom_angular_noise", odom_angular_noise, odom_angular_noise);
    nh.param("odom_linear_noise_stationary", odom_linear_noise_stationary, odom_linear_noise_stationary);
    nh.param("odom_angular_noise_stationary", odom_angular_noise_stationary, odom_angular_noise_stationary);
    nh.param("scan_measurement_noise", scan_measurement_noise, scan_measurement_noise);
    // subscriber
    odom_sub = nh.subscribe(input_odom_topic_name, 100, &AddNoise::odom_callback, this);
//    scan_sub = nh.subscribe(input_scan_topic_name, 100, &AddNoise::scan_callback, this);
    scan_sub = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, input_scan_topic_name, 10);
    object_ids_sub = new message_filters::Subscriber<robot_sim::ScanObjectID>(nh, input_scan_object_ids_topic_name, 10);
    sync = new message_filters::TimeSynchronizer<sensor_msgs::LaserScan, robot_sim::ScanObjectID>(*scan_sub, *object_ids_sub, 30);
    sync->registerCallback(boost::bind(&AddNoise::scan_callback, this, _1, _2));
    // publisher
    odom_pub = nh.advertise<nav_msgs::Odometry>(output_odom_topic_name, 100);
    scan_pub = nh.advertise<sensor_msgs::LaserScan>(output_scan_topic_name, 100);
    points_pub = nh.advertise<sensor_msgs::PointCloud2>(output_scan_points_topic_name, 100);
    semantic_scan_pub = nh.advertise<robot_sim::SemanticScan>(output_semantic_scan_topic_name, 100);
    // wait for topics
    ros::spin();
}

void AddNoise::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    static double prev_time;
    nav_msgs::Odometry odom = *msg;
    if (!is_odom_initialized)
    {
        prev_time = msg->header.stamp.toSec();
        tf::Quaternion q(msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        robot_pose.x = msg->pose.pose.position.x;
        robot_pose.y = msg->pose.pose.position.y;
        robot_pose.yaw = yaw;
        is_odom_initialized = true;
        return;
    }
    double curr_time = msg->header.stamp.toSec();
    if (fabs(odom.twist.twist.linear.x) > 0.01 || fabs(odom.twist.twist.linear.y) > 0.01 || fabs(odom.twist.twist.angular.z) > 0.002)
    {
        if (fabs(odom.twist.twist.linear.x) > 0.01)
            odom.twist.twist.linear.x += nrand(odom_linear_noise);
        if (fabs(odom.twist.twist.linear.y) > 0.01)
            odom.twist.twist.linear.y += nrand(odom_linear_noise);
        if (fabs(odom.twist.twist.angular.z) > 0.002)
            odom.twist.twist.angular.z += nrand(odom_angular_noise);
        double delta_time = curr_time - prev_time;
        double delta_dist_x = odom.twist.twist.linear.x * delta_time * odom_linear_noise_stationary;
        double delta_dist_y = odom.twist.twist.linear.y * delta_time * odom_linear_noise_stationary;
        double delta_yaw = odom.twist.twist.angular.z * delta_time * odom_angular_noise_stationary;
        double theta = robot_pose.yaw + delta_yaw / 2.0;
        robot_pose.x += delta_dist_x * cos(theta) + delta_dist_y * sin(theta);
        robot_pose.y += delta_dist_x * sin(theta) + delta_dist_y * cos(theta);
        robot_pose.yaw += delta_yaw;
        mod_yaw(&robot_pose.yaw);
    }
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_pose.yaw);
    odom.pose.pose.position.x = robot_pose.x;
    odom.pose.pose.position.y = robot_pose.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = odom.twist.twist.angular.y = 0.0;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(robot_pose.x, robot_pose.y, 0.0));
    q.setRPY(0.0, 0.0, robot_pose.yaw);
    transform.setRotation(q);
    static tf::TransformBroadcaster br;
    // NOTE: warning will occur if the same time stamp (msg->header.stamp) is used for the tf broadcast
    // this makes redundant timestamp
//    br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, odom.header.frame_id, odom.child_frame_id));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odom.header.frame_id, odom.child_frame_id));
    prev_time = curr_time;
    odom_pub.publish(odom);
}

void AddNoise::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
    const robot_sim::ScanObjectID::ConstPtr& object_ids_msg)
{
    sensor_msgs::LaserScan scan = *scan_msg;
    robot_sim::SemanticScan semantic_scan;
    semantic_scan.scan = *scan_msg;
    semantic_scan.probs.resize(get_semantic_label_num() * scan_msg->ranges.size());
    pcl::PointCloud<pcl::PointXYZRGB> points;
    pcl_conversions::toPCL(scan_msg->header.stamp, points.header.stamp);
    points.header.frame_id = scan_msg->header.frame_id;
    for (int i = 0; i < scan.ranges.size(); i++)
    {
        if (scan.ranges[i] < scan.range_min || scan.range_max < scan.ranges[i])
            continue;
        else
        {
            scan.ranges[i] += nrand(scan_measurement_noise);
            semantic_scan.scan.ranges[i] = scan.ranges[i];
            int maximum_type, start_index, end_index;
            std::vector<float> probs;
            get_noisy_semantic_probs(object_ids_msg->ids[i].data, &maximum_type, &probs);
            get_corresponding_semantic_prob_indexes(i, &start_index, &end_index);
            for (int idx = start_index; idx <= end_index; idx++)
                semantic_scan.probs[idx].data = probs[idx - start_index];
            double yaw = scan.angle_min + scan.angle_increment * i;
            pcl::PointXYZRGB p;
            p.x = scan.ranges[i] * cos(yaw);
            p.y = scan.ranges[i] * sin(yaw);
            p.z = 0.0;
            get_semantic_point_color(maximum_type, &p.r, &p.g, &p.b);
            points.points.push_back(p);
        }
    }
    scan_pub.publish(scan);
    sensor_msgs::PointCloud2 points_msg;
    pcl::toROSMsg(points, points_msg);
    points_msg.header = scan_msg->header;
    points_pub.publish(points_msg);
    semantic_scan_pub.publish(semantic_scan);
}

void AddNoise::get_noisy_semantic_probs(int pre_type, int* type, std::vector<float>* probs)
{
    double sum = 0.0;
    std::vector<float> s_probs(get_semantic_label_num());
    for (int i = 0; i < (int)s_probs.size(); i++)
    {
        if (i != pre_type)
            s_probs[i] = fabs(0.0 + nrand(0.5));
        else
            s_probs[i] = fabs(1.0 + nrand(0.5));
        sum += s_probs[i];
    }
    int s_type;
    double max;
    for (int i = 0; i < (int)s_probs.size(); i++)
    {
        s_probs[i] /= sum;
        if (i == 0)
        {
            s_type = i;
            max = s_probs[i];
        }
        else if (s_probs[i] > max)
        {
            s_type = i;
            max = s_probs[i];
        }
    }
    *type = s_type;
    *probs = s_probs;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_noise_to_sensor_data");
    AddNoise node;
    return 0;
}
