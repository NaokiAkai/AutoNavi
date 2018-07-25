// Copyright © 2018 Naoki Akai. All rights reserved.

#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <robot_sim/common.h>

class AddNoise
{
private:
    ros::NodeHandle nh;
    ros::Subscriber odom_sub, scan_sub;
    ros::Publisher odom_pub, scan_pub, points_pub;
    std::string input_odom_topic_name, input_scan_topic_name;
    std::string output_odom_topic_name, output_scan_topic_name, output_scan_points_topic_name;
    double odom_linear_noise, odom_angular_noise;
    double odom_linear_noise_stationary, odom_angular_noise_stationary;
    double scan_measurement_noise;
    pose_t robot_pose;
    bool is_odom_initialized;

public:
    AddNoise();
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

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
    output_odom_topic_name("/odom"),
    output_scan_topic_name("/scan"),
    output_scan_points_topic_name("/scan_points"),
    odom_linear_noise(0.05),
    odom_angular_noise(0.01),
    odom_linear_noise_stationary(0.99),
    odom_angular_noise_stationary(0.99),
    scan_measurement_noise(0.05),
    is_odom_initialized(false)
{
    // read parameters
    nh.param("/add_noise_to_sensor_data/input_odom_topic_name", input_odom_topic_name, input_odom_topic_name);
    nh.param("/add_noise_to_sensor_data/input_scan_topic_name", input_scan_topic_name, input_scan_topic_name);
    nh.param("/add_noise_to_sensor_data/output_odom_topic_name", output_odom_topic_name, output_odom_topic_name);
    nh.param("/add_noise_to_sensor_data/output_scan_topic_name", output_scan_topic_name, output_scan_topic_name);
    nh.param("/add_noise_to_sensor_data/output_scan_points_topic_name", output_scan_points_topic_name, output_scan_points_topic_name);
    nh.param("/add_noise_to_sensor_data/odom_linear_noise", odom_linear_noise, odom_linear_noise);
    nh.param("/add_noise_to_sensor_data/odom_angular_noise", odom_angular_noise, odom_angular_noise);
    nh.param("/add_noise_to_sensor_data/odom_linear_noise_stationary", odom_linear_noise_stationary, odom_linear_noise_stationary);
    nh.param("/add_noise_to_sensor_data/odom_angular_noise_stationary", odom_angular_noise_stationary, odom_angular_noise_stationary);
    nh.param("/add_noise_to_sensor_data/scan_measurement_noise", scan_measurement_noise, scan_measurement_noise);
    // subscriber
    odom_sub = nh.subscribe(input_odom_topic_name, 100, &AddNoise::odom_callback, this);
    scan_sub = nh.subscribe(input_scan_topic_name, 100, &AddNoise::scan_callback, this);
    // publisher
    odom_pub = nh.advertise<nav_msgs::Odometry>(output_odom_topic_name, 100);
    scan_pub = nh.advertise<sensor_msgs::LaserScan>(output_scan_topic_name, 100);
    points_pub = nh.advertise<sensor_msgs::PointCloud>(output_scan_points_topic_name, 100);
    // wait for topics
    ros::spin();
}

void AddNoise::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    static double prev_time;
    nav_msgs::Odometry odom = *msg;
//    odom.header.frame_id = "/odom";
    odom.child_frame_id = "/odom_frame";
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
    if (fabs(odom.twist.twist.linear.x) > 0.01 || fabs(odom.twist.twist.angular.z) > 0.002)
    {
        odom.twist.twist.linear.x += nrand(odom_linear_noise);
        odom.twist.twist.angular.z += nrand(odom_angular_noise);
        double delta_time = curr_time - prev_time;
        double delta_dist = odom.twist.twist.linear.x * delta_time * odom_linear_noise_stationary;
        double delta_yaw = odom.twist.twist.angular.z * delta_time * odom_angular_noise_stationary;
        robot_pose.x += delta_dist * cos(robot_pose.yaw + delta_yaw / 2.0);
        robot_pose.y += delta_dist * sin(robot_pose.yaw + delta_yaw / 2.0);
        robot_pose.yaw += delta_yaw;
        mod_yaw(&robot_pose.yaw);
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_pose.yaw);
        odom.pose.pose.position.x = robot_pose.x;
        odom.pose.pose.position.y = robot_pose.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.twist.twist.linear.y = odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = odom.twist.twist.angular.y = 0.0;
    }
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(robot_pose.x, robot_pose.y, 0.0));
    q.setRPY(0.0, 0.0, robot_pose.yaw);
    transform.setRotation(q);
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, odom.header.frame_id, odom.child_frame_id));
    prev_time = curr_time;
    odom_pub.publish(odom);
}

void AddNoise::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensor_msgs::LaserScan scan = *msg;
    sensor_msgs::PointCloud points;
    points.header = scan.header;
    points.channels.resize(1);
    points.channels[0].name = "intensity";
    for (int i = 0; i < scan.ranges.size(); i++)
    {
        if (scan.ranges[i] < scan.range_min || scan.range_max < scan.ranges[i])
        {
            continue;
        }
        else
        {
            scan.ranges[i] += nrand(scan_measurement_noise);
            double yaw = scan.angle_min + scan.angle_increment * i;
            geometry_msgs::Point32 p;
            p.x = scan.ranges[i] * cos(yaw);
            p.y = scan.ranges[i] * sin(yaw);
            p.z = 0.0;
            points.points.push_back(p);
            points.channels[0].values.push_back(100);
        }
    }
    scan_pub.publish(scan);
    points_pub.publish(points);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_noise_to_sensor_data");
    AddNoise node;
    return 0;
}
