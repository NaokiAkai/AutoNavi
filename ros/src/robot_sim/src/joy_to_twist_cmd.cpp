/****************************************************************************
 * Copyright (C) 2018 Naoki Akai.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 ****************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>

#define DEFAULT_AXIS_LINEAR_ID        (1)
#define DEFAULT_AXIS_ANGULAR_ID       (0)
#define DEFAULT_ANGULAR_POSI_AXIS_ID  (3)
#define DEFAULT_ANGULAR_NEGA_AXIS_ID  (4)
#define DEFAULT_ACTIVATE_BUTTON_ID    (0)

class Joy2TwistCmd
{
private:
    ros::NodeHandle nh;
    ros::Subscriber joy_sub;
    ros::Publisher twist_cmd_pub;
    std::string output_twist_topic_name;
    double max_linear_vel, max_angular_vel, output_hz;
    geometry_msgs::TwistStamped twist_cmd;
    bool use_ackermann_simulator, use_omni_simulator;
    double max_steering_angle;

public:
    Joy2TwistCmd();
    ~Joy2TwistCmd() {};
    void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);
};

Joy2TwistCmd::Joy2TwistCmd():
    nh("~"),
    output_twist_topic_name("/twist_cmd"),
    max_linear_vel(1.0),
    max_angular_vel(0.6),
    output_hz(40.0),
    use_ackermann_simulator(false),
    use_omni_simulator(false),
    max_steering_angle(0.698)
{
    // read parameters
    nh.param("output_twist_topic_name", output_twist_topic_name, output_twist_topic_name);
    nh.param("max_linear_vel", max_linear_vel, max_linear_vel);
    nh.param("max_angular_vel", max_angular_vel, max_angular_vel);
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
    joy_sub = nh.subscribe("/joy", 100, &Joy2TwistCmd::joy_callback, this);
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

void Joy2TwistCmd::joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
    if (msg->buttons[DEFAULT_ACTIVATE_BUTTON_ID] == 1 && !use_ackermann_simulator && !use_omni_simulator)
    {
        twist_cmd.twist.linear.x = max_linear_vel * msg->axes[DEFAULT_AXIS_LINEAR_ID];
        twist_cmd.twist.angular.z = max_angular_vel * msg->axes[DEFAULT_AXIS_ANGULAR_ID]; 
    }
    else if (msg->buttons[DEFAULT_ACTIVATE_BUTTON_ID] == 1 && use_ackermann_simulator)
    {
        twist_cmd.twist.linear.x = max_linear_vel * msg->axes[DEFAULT_AXIS_LINEAR_ID];
        twist_cmd.twist.angular.z = max_steering_angle * msg->axes[DEFAULT_AXIS_ANGULAR_ID]; 
    }
    else if (msg->buttons[DEFAULT_ACTIVATE_BUTTON_ID] == 1 && use_omni_simulator)
    {
        twist_cmd.twist.linear.x = max_linear_vel * msg->axes[DEFAULT_AXIS_LINEAR_ID];
        twist_cmd.twist.linear.y = max_linear_vel * msg->axes[DEFAULT_AXIS_ANGULAR_ID];
        if (msg->axes[DEFAULT_ANGULAR_POSI_AXIS_ID] < 0.0)
            twist_cmd.twist.angular.z += -1.0 * max_angular_vel * msg->axes[DEFAULT_ANGULAR_POSI_AXIS_ID]; 
        if (msg->axes[DEFAULT_ANGULAR_NEGA_AXIS_ID] < 0.0)
            twist_cmd.twist.angular.z += 1.0 * max_angular_vel * msg->axes[DEFAULT_ANGULAR_NEGA_AXIS_ID]; 
    }
    else
    {
        twist_cmd.twist.linear.x = twist_cmd.twist.linear.y = twist_cmd.twist.angular.z = 0.0;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_to_twist_cmd");
    Joy2TwistCmd node;
    return 0;
}
