/****************************************************************************
 * Copyright (C) 2018 Naoki Akai.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 ****************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class SOAN
{
private:
    ros::NodeHandle nh;
    ros::Subscriber twist_sub, scan_sub;
    ros::Publisher cmd_pub, twist_pub, local_map_pub;
    std::string input_twist_topic_name, input_scan_topic_name;
    std::string output_twist_topic_name;
    cv::Mat local_map;
    geometry_msgs::TwistStamped current_cmd;
    sensor_msgs::LaserScan current_scan;
    double old_target_w;
    bool do_stop, do_back, is_first_stop;
    double first_stop_time, first_back_time;
    double local_map_size_x, local_map_size_y, local_map_pixel_size;
    double robot_width, predict_time, step_time, kv, w_range;
    bool use_emergency_stop;
    double stop_x_size, stop_y_size, stop_time, back_time, back_speed;
    double cmd_publish_hz;

public:
    SOAN();
    ~SOAN() {};
    void init(void);
    void twist_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void avoid(void);
    bool check_collision(double v, double w, double *free_dist, int green);
    bool do_emergency_stop(sensor_msgs::LaserScan scan);
};

SOAN::SOAN():
    nh("~"),
    input_twist_topic_name("/twist_cmd"),
    input_scan_topic_name("/scan"),
    output_twist_topic_name("/twist_cmd_with_avoidance"),
    local_map_size_x(10.0),
    local_map_size_y(10.0),
    local_map_pixel_size(0.05),
    robot_width(1.0),
    predict_time(5.0),
    step_time(0.1),
    kv(0.8),
    w_range(0.4),
    use_emergency_stop(true),
    stop_x_size(0.6),
    stop_y_size(0.4),
    stop_time(3.0),
    back_time(5.0),
    back_speed(0.2),
    cmd_publish_hz(20.0),
    do_stop(false),
    do_back(false),
    is_first_stop(true)
{
    // read parameters
    nh.param("input_twist_topic_name", input_twist_topic_name, input_twist_topic_name);
    nh.param("input_scan_topic_name", input_scan_topic_name, input_scan_topic_name);
    nh.param("output_twist_topic_name", output_twist_topic_name, output_twist_topic_name);
    nh.param("local_map_size_x", local_map_size_x, local_map_size_x);
    nh.param("local_map_size_y", local_map_size_y, local_map_size_y);
    nh.param("local_map_pixel_size", local_map_pixel_size, local_map_pixel_size);
    nh.param("robot_width", robot_width, robot_width);
    nh.param("predict_time", predict_time, predict_time);
    nh.param("step_time", step_time, step_time);
    nh.param("kv", kv, kv);
    nh.param("w_range", w_range, w_range);
    nh.param("use_emergency_stop", use_emergency_stop, use_emergency_stop);
    nh.param("stop_x_size", stop_x_size, stop_x_size);
    nh.param("stop_y_size", stop_y_size, stop_y_size);
    nh.param("stop_time", stop_time, stop_time);
    nh.param("back_time", back_time, back_time);
    nh.param("back_speed", back_speed, back_speed);
    nh.param("cmd_publish_hz", cmd_publish_hz, cmd_publish_hz);
    // subscriber
    twist_sub = nh.subscribe(input_twist_topic_name, 100, &SOAN::twist_callback, this);
    scan_sub = nh.subscribe(input_scan_topic_name, 10, &SOAN::scan_callback, this);
    // publisher
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    twist_pub = nh.advertise<geometry_msgs::TwistStamped>(output_twist_topic_name, 100);
    local_map_pub = nh.advertise<sensor_msgs::Image>("/local_map_for_simple_obstacle_avoidance", 100);
    // initialization
    init();
    // main loop
    ros::Rate loop_rate(cmd_publish_hz);
    while (ros::ok())
    {
        ros::spinOnce();
        avoid();
        loop_rate.sleep();
    }
}

void SOAN::init(void)
{
    int w = (int)(local_map_size_x / local_map_pixel_size);
    int h = (int)(local_map_size_y / local_map_pixel_size);
    local_map = cv::Mat::zeros(cv::Size(w, h), CV_8UC3);
}

void SOAN::twist_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_cmd = *msg;
}

void SOAN::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    current_scan = *msg;
}

void SOAN::avoid(void)
{
    // memorize current path following command and scan data
    double v0 = current_cmd.twist.linear.x;
    double w0 = current_cmd.twist.angular.z;
    sensor_msgs::LaserScan scan = current_scan;
    // mapping scan data
    local_map = cv::Scalar::all(0);
    for (int i = 0; i < scan.ranges.size(); i++)
    {
        double r = scan.ranges[i];
        if (r < scan.range_min || scan.range_max < r)
            continue;
        double angle = scan.angle_min + scan.angle_increment * (double)i + M_PI / 2.0;
        double x = r * cos(angle);
        double y = r * sin(angle);
        int u = local_map.cols / 2 + (int)(x / local_map_pixel_size);
        int v = local_map.rows - (int)(y / local_map_pixel_size);
        if (1 <= u && u < local_map.cols -  1 && 1 <= v && v < local_map.rows - 1)
        {
            local_map.at<cv::Vec3b>(v, u)[2] = 255;
            local_map.at<cv::Vec3b>(v, u - 1)[2] = 255;
            local_map.at<cv::Vec3b>(v, u + 1)[2] = 255;
            local_map.at<cv::Vec3b>(v - 1, u)[2] = 255;
            local_map.at<cv::Vec3b>(v + 1, u)[2] = 255;
        }
    }
    // check collision and search obstacle avoidance command
    double free_dist, target_v, target_w;
    if (check_collision(v0, w0, &free_dist, 50))
    {
        double v = v0 * kv;
        bool is_first = true;
        double max_e;
        for (double w = old_target_w - w_range; w <= old_target_w + w_range; w += 0.02)
        {
            check_collision(v, w, &free_dist, 50);
            double dw = w0 - w;
            double e = free_dist * exp(-(dw * dw) / (2.0 * 0.5 * 0.5));
            if (is_first)
            {
                target_v = v;
                target_w = w;
                max_e = e;
                is_first = false;
            }
            else if (max_e < e)
            {
                target_v = v;
                target_w = w;
                max_e = e;
            }
        }
        check_collision(target_v, target_w, &free_dist, 255);
    }
    else
    {
        target_v = v0;
        target_w = w0;
    }
    old_target_w = target_w;
    // emergency stop and back
    double current_time = ros::Time::now().toSec();
    bool do_stop = false;
    if (use_emergency_stop)
        do_stop = do_emergency_stop(scan);
    if (do_stop)
    {
        target_v = target_w = 0.0;
        if (is_first_stop)
        {
            first_stop_time = current_time;
            is_first_stop = false;
        }
        else if (current_time - first_stop_time > stop_time && !do_back)
        {
            first_back_time = current_time;
            do_back = true;
        }
    }
    else
    {
        is_first_stop = true;
    }
    if (do_back)
    {
        target_v = -back_speed;
        if (target_v > 0.0)    target_v *= -1.0;
        target_w = 0.0;
        if (current_time - first_back_time > back_time)
        {
            target_v = target_w = 0.0;
            do_back = false;
        }
    }
    // publish
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::TwistStamped avoid_cmd;
    avoid_cmd.header.stamp = ros::Time::now();
    cmd_vel.linear.x = avoid_cmd.twist.linear.x = target_v;
    cmd_vel.angular.z = avoid_cmd.twist.angular.z = target_w;
    cmd_pub.publish(cmd_vel);
    twist_pub.publish(avoid_cmd);
    cv_bridge::CvImage local_map_msg(scan.header, "bgr8", local_map);
    local_map_pub.publish(local_map_msg.toImageMsg());
}

bool SOAN::check_collision(double v, double w, double *free_dist, int green)
{
    double x0, y0, t0, d0;
    double dd = v * step_time;
    double dt = w * step_time;
    double dist = v * predict_time;
    double cos90 = cos(M_PI / 2.0);
    double sin90 = sin(M_PI / 2.0);
    x0 = y0 = t0 = d0 = 0.0;
    for (double t = 0.0; t < predict_time; t += step_time)
    {
        x0 += dd * cos(t0);
        y0 += dd * sin(t0);
        t0 += dt;
        d0 += dd;
        double tt = t0 + M_PI / 2.0;
        for (double l = -robot_width / 2; l <= robot_width / 2; l += local_map_pixel_size)
        {
            double xx = x0 + l * cos(tt);
            double yy = y0 + l * sin(tt);
            double x = xx * cos90 - yy * sin90;
            double y = xx * sin90 + yy * cos90;
            int u = local_map.cols / 2 + (int)(x / local_map_pixel_size);
            int v = local_map.rows - (int)(y / local_map_pixel_size);
            if (0 <= u && u < local_map.cols && 0 <= v && v < local_map.rows)
            {
                if (local_map.at<cv::Vec3b>(v, u)[2] == 255)
                {
                    *free_dist = d0;
                    return true;
                }
                else
                {
                    local_map.at<cv::Vec3b>(v, u)[1] = green;
                }
            }
        }
        if (d0 > dist)    break;
    }
    *free_dist = dist;
    return false;
}

bool SOAN::do_emergency_stop(sensor_msgs::LaserScan scan)
{
    int c = 0;
    for (int i = 0; i < scan.ranges.size(); i++)
    {
        double r = scan.ranges[i];
        if (r < scan.range_min || scan.range_max < r)
            continue;
        double angle = scan.angle_min + scan.angle_increment * (double)i + M_PI / 2.0;
        double x = r * cos(angle);
        double y = r * sin(angle);
        if (fabs(x) < stop_x_size && y < stop_y_size)
        {
            c++;
            if (c >= 5)
                return true;
        }
    }
    return false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_obstacle_avoidance");
    SOAN node;
    return 0;
}
