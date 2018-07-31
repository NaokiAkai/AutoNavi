// Copyright © 2018 Naoki Akai. All rights reserved.

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class GyroOdom
{
private:
    ros::NodeHandle nh;
    std::string input_imu_topic_name, input_odom_topic_name, output_odom_topic_name;
    std::string parent_frame, child_frame;
    ros::Subscriber imu_sub, odom_sub;
    ros::Publisher odom_pub;
    double w, x, y, yaw;
    int moving_average_filter_num;
    std::vector<double> ws;

public:
    GyroOdom();
    ~GyroOdom() {};
    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
};

GyroOdom::GyroOdom():
    nh("~"),
    input_imu_topic_name("/imu/data"),
    input_odom_topic_name("/odom"),
    output_odom_topic_name("/gyro_odom"),
    parent_frame("/map"),
    child_frame("/gyro_odom"),
    moving_average_filter_num(3),
    w(0.0),
    x(0.0),
    y(0.0),
    yaw(0.0)
{
    // read parameters
    nh.param("/gyro_odom/input_imu_topic_name", input_imu_topic_name, input_imu_topic_name);
    nh.param("/gyro_odom/input_odom_topic_name", input_odom_topic_name, input_odom_topic_name);
    nh.param("/gyro_odom/output_odom_topic_name", output_odom_topic_name, output_odom_topic_name);
    nh.param("/gyro_odom/parent_frame", parent_frame, parent_frame);
    nh.param("/gyro_odom/child_frame", child_frame, child_frame);
    nh.param("/gyro_odom/moving_average_filter_num", moving_average_filter_num, moving_average_filter_num);
    // parameter preparation
    ws.resize(moving_average_filter_num, 0.0);
    // subscriber
    imu_sub = nh.subscribe(input_imu_topic_name, 100, &GyroOdom::imu_callback, this);
    odom_sub = nh.subscribe(input_odom_topic_name, 100, &GyroOdom::odom_callback, this);
    // publisher
    odom_pub = nh.advertise<nav_msgs::Odometry>(output_odom_topic_name, 100);
}

void GyroOdom::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    static bool is_first = true;
    static double prev_time, old_w1, old_w2;
    if (is_first)
    {
        prev_time = msg->header.stamp.toSec();
        is_first = false;
        return;
    }
    double curr_time = msg->header.stamp.toSec();
    double dtime = curr_time - prev_time;
    if (dtime > 1.0)
    {
        prev_time = msg->header.stamp.toSec();
        is_first = false;
        return;
    }
/*
    w = msg->angular_velocity.z;
    w = (w + old_w1 + old_w2) / 3.0;
    old_w2 = old_w1;
    old_w1 = w;
 */
    for (int i = moving_average_filter_num - 1; i > 0 ; i--)
        ws[i] = ws[i - 1];
    ws[0] = msg->angular_velocity.z;
    w = 0.0;
    for (int i = 0; i < moving_average_filter_num; i++)
        w += ws[i];
    w /= (double)moving_average_filter_num;
    yaw += w * dtime;
    if (yaw < -M_PI)
        yaw += 2.0 * M_PI;
    if (yaw > M_PI)
        yaw -= 2.0 * M_PI;
    prev_time = curr_time;
}

void GyroOdom::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    static bool is_first = true;
    static double prev_time;
    if (is_first)
    {
        prev_time = msg->header.stamp.toSec();
        is_first = false;
        return;
    }
    double curr_time = msg->header.stamp.toSec();
    double dtime = curr_time - prev_time;
    if (dtime > 1.0)
    {
        prev_time = msg->header.stamp.toSec();
        is_first = false;
        return;
    }
    nav_msgs::Odometry odom = *msg;
    x += odom.twist.twist.linear.x * cos(yaw) * dtime;
    y += odom.twist.twist.linear.x * sin(yaw) * dtime;
    odom.header.frame_id = parent_frame;
    odom.child_frame_id = child_frame;
    odom.twist.twist.angular.z = w;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    odom_pub.publish(odom);
    tf::Transform transform;    
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, 0.0));
    q.setRPY(0.0, 0.0, yaw);
    transform.setRotation(q);
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, odom.header.stamp, parent_frame, child_frame));
    prev_time = curr_time;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gyro_odom");
    GyroOdom node;
    ros::spin();
    return 0;
}
