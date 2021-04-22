/****************************************************************************
 * Copyright (C) 2018 Naoki Akai.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 ****************************************************************************/

#include <ros/ros.h>
#include <keyboard/Key.h>
#include <geometry_msgs/TwistStamped.h>

class Key2TwistCmd
{
private:
    ros::NodeHandle nh;
    ros::Subscriber key_sub;
    ros::Publisher twist_cmd_pub;
    std::string output_twist_topic_name;
    double trans_vel_step, angle_vel_step, output_hz;
    geometry_msgs::TwistStamped twist_cmd;
    bool use_ackermann_simulator, use_omni_simulator;
    double max_steering_angle;

public:
    Key2TwistCmd();
    ~Key2TwistCmd() {};
    void key_callback(const keyboard::Key::ConstPtr& msg);
};

Key2TwistCmd::Key2TwistCmd():
    nh("~"),
    output_twist_topic_name("/twist_cmd"),
    trans_vel_step(0.2),
    angle_vel_step(0.03),
    output_hz(40.0),
    use_ackermann_simulator(false),
    use_omni_simulator(false),
    max_steering_angle(0.698)
{
    // read parameters
    nh.param("output_twist_topic_name", output_twist_topic_name, output_twist_topic_name);
    nh.param("trans_vel_step", trans_vel_step, trans_vel_step);
    nh.param("angle_vel_step", angle_vel_step, angle_vel_step);
    nh.param("output_hz", output_hz, output_hz);
    nh.param("use_ackermann_simulator", use_ackermann_simulator, use_ackermann_simulator);
    nh.param("use_omni_simulator", use_omni_simulator, use_omni_simulator);
    nh.param("max_steering_angle", max_steering_angle, max_steering_angle);
    // check variables
    if (use_ackermann_simulator && use_omni_simulator)
    {
        ROS_ERROR("use_ackermann_simulator and use_omni_simulator are true. One must be false.");
        exit(1);
    }
    // subscriber
    key_sub = nh.subscribe("/keyboard/keydown", 100, &Key2TwistCmd::key_callback, this);
    // publisher
    twist_cmd_pub = nh.advertise<geometry_msgs::TwistStamped>(output_twist_topic_name, 100);
    // main loop
    ros::Rate loop_rate(output_hz);
    while (ros::ok()) 
    {
        twist_cmd.header.stamp = ros::Time::now();
        twist_cmd_pub.publish(twist_cmd);
        ros::spinOnce();
        printf("linear.x = %.2lf [m/sec], anguler.z = %.3lf [rad/sec]\n", twist_cmd.twist.linear.x, twist_cmd.twist.angular.z);
        loop_rate.sleep();
    }
}

void Key2TwistCmd::key_callback(const keyboard::Key::ConstPtr &msg)
{
    if (msg->code == keyboard::Key::KEY_UP)
    {
        twist_cmd.twist.linear.x += trans_vel_step;
    }
    else if (msg->code == keyboard::Key::KEY_DOWN)
    {
        twist_cmd.twist.linear.x -= trans_vel_step;
    }
    else if (msg->code == keyboard::Key::KEY_LEFT)
    {
        if (!use_ackermann_simulator && !use_omni_simulator)
        {
            twist_cmd.twist.angular.z += angle_vel_step;
        }
        else if (use_ackermann_simulator)
        {
            twist_cmd.twist.angular.z += angle_vel_step;
            if (twist_cmd.twist.angular.z > max_steering_angle)
                twist_cmd.twist.angular.z = max_steering_angle;
        }
        else
        {
            twist_cmd.twist.linear.y += trans_vel_step;
        }
    }
    else if (msg->code == keyboard::Key::KEY_RIGHT)
    {
        if (!use_ackermann_simulator && !use_omni_simulator)
        {
            twist_cmd.twist.angular.z -= angle_vel_step;
        }
        else if (use_ackermann_simulator)
        {
            twist_cmd.twist.angular.z -= angle_vel_step;
            if (twist_cmd.twist.angular.z < -max_steering_angle)
                twist_cmd.twist.angular.z = -max_steering_angle;
        }
        else
        {
            twist_cmd.twist.linear.y -= trans_vel_step;
        }
    }
    else if (msg->code == keyboard::Key::KEY_a)
    {
        twist_cmd.twist.angular.z += angle_vel_step;
    }
    else if (msg->code == keyboard::Key::KEY_s)
    {
        twist_cmd.twist.angular.z -= angle_vel_step;
    }
    else if (msg->code == keyboard::Key::KEY_SPACE)
    {
        twist_cmd.twist.linear.x = twist_cmd.twist.linear.y = twist_cmd.twist.angular.z = 0.0;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "key_to_twist_cmd");
    Key2TwistCmd node;
    return 0;
}
