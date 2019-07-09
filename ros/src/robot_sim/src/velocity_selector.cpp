/****************************************************************************
 * Copyright (C) 2018 Naoki Akai.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 ****************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

class VelocitySelector
{
private:
    ros::NodeHandle nh;
    std::string input_dev_cmd_name, input_path_follow_cmd_name, output_twist_cmd_name;
    geometry_msgs::TwistStamped dev_cmd, path_follow_cmd;
    double cmd_publish_hz;
    ros::Subscriber dev_cmd_sub, path_follow_cmd_sub;
    ros::Publisher twist_pub;

public:
    VelocitySelector();
    ~VelocitySelector() {};
    void dev_cmd_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void path_follow_cmd_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void spin(void);
};

VelocitySelector::VelocitySelector():
    nh("~"),
    input_dev_cmd_name("/twist_cmd_dev"),
    input_path_follow_cmd_name("/twist_cmd_path_follow"),
    output_twist_cmd_name("/twist_cmd"),
    cmd_publish_hz(40.0)
{
    // read parameters
    nh.param("input_dev_cmd_name", input_dev_cmd_name, input_dev_cmd_name);
    nh.param("input_path_follow_cmd_name", input_path_follow_cmd_name, input_path_follow_cmd_name);
    nh.param("output_twist_cmd_name", output_twist_cmd_name, output_twist_cmd_name);
    nh.param("cmd_publish_hz", cmd_publish_hz, cmd_publish_hz);
    // subscriber
    dev_cmd_sub = nh.subscribe(input_dev_cmd_name, 10, &VelocitySelector::dev_cmd_callback, this);
    path_follow_cmd_sub = nh.subscribe(input_path_follow_cmd_name, 10, &VelocitySelector::path_follow_cmd_callback, this);
    // publisher
    twist_pub = nh.advertise<geometry_msgs::TwistStamped>(output_twist_cmd_name, 10);
}

void VelocitySelector::dev_cmd_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    dev_cmd = *msg;
}

void VelocitySelector::path_follow_cmd_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    path_follow_cmd = *msg;
}

void VelocitySelector::spin(void)
{
    ros::Rate loop_rate(cmd_publish_hz);
    geometry_msgs::TwistStamped prev_cmd;
    bool is_first = true;
    while (ros::ok())
    {
        ros::spinOnce();
        geometry_msgs::TwistStamped cmd;
        if (path_follow_cmd.twist.linear.z > 0.0)
            cmd = path_follow_cmd;
        else
            cmd = dev_cmd;
        if (is_first)
        {
            twist_pub.publish(cmd);
            is_first = false;
        }
        else
        {
            double delta_time = cmd.header.stamp.toSec() - prev_cmd.header.stamp.toSec();
            if (delta_time > 0.0)
                twist_pub.publish(cmd);
        }
        prev_cmd = cmd;
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_selector");
    VelocitySelector node;
    node.spin();
    return 0;
}
