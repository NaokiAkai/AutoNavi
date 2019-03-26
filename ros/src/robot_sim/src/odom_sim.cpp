// Copyright © 2018 Naoki Akai. All rights reserved.

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <robot_sim/common.h>

class OdomSim
{
private:
    ros::NodeHandle nh;
    ros::Subscriber twist_sub;
    ros::Publisher odom_pub, pose_pub;
    std::string input_twist_topic_name;
    std::string output_odom_topic_name;
    std::string map_frame;
    nav_msgs::Odometry odom;
    geometry_msgs::PoseStamped pose;
    pose_t robot_pose;
    double odom_publish_hz;
    bool use_ackermann_simulator, use_omni_simulator;
    double base_line, steering_angle;
    bool record_ground_truth_trajectory;

public:
    OdomSim();
    ~OdomSim() {};
    void twist_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void robot_sim_init(void);
    void publish_messages(void);
    void record_ground_truth(void);

    inline void mod_yaw(double *yaw)
    {
        while (*yaw < -M_PI)    *yaw += 2.0 * M_PI;
        while (*yaw > M_PI)     *yaw -= 2.0 * M_PI;
    }
};

OdomSim::OdomSim():
    nh("~"),
    input_twist_topic_name("/twist_cmd"),
    output_odom_topic_name("/odom_robot_sim"),
    map_frame("/world"),
    odom_publish_hz(50.0),
    use_ackermann_simulator(false),
    base_line(1.0),
    record_ground_truth_trajectory(false),
    steering_angle(0.0)
{
    // read parameters
    nh.param("input_twist_topic_name", input_twist_topic_name, input_twist_topic_name);
    nh.param("output_odom_topic_name", output_odom_topic_name, output_odom_topic_name);
    nh.param("map_frame", map_frame, map_frame);
    nh.param("odom_publish_hz", odom_publish_hz, odom_publish_hz);
    nh.param("use_ackermann_simulator", use_ackermann_simulator, use_ackermann_simulator);
    nh.param("use_omni_simulator", use_omni_simulator, use_omni_simulator);
    nh.param("base_line", base_line, base_line);
    nh.param("record_ground_truth_trajectory", record_ground_truth_trajectory, record_ground_truth_trajectory);
    // check variables
    if (use_ackermann_simulator && use_omni_simulator)
    {
        ROS_ERROR("use_ackermann_simulator and use_omni_simulator are true. One must be false.");
        exit(1);
    }
    // set initial state
    odom.header.frame_id = pose.header.frame_id = map_frame;
    odom.child_frame_id = "/ground_truth";
    robot_pose.x = robot_pose.y = robot_pose.yaw = 0.0;
    odom.pose.pose.position.x = pose.pose.position.x = robot_pose.x;
    odom.pose.pose.position.y = pose.pose.position.y = robot_pose.y;
    odom.pose.pose.position.z = pose.pose.position.z = 0.0;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_pose.yaw);
    odom.pose.pose.orientation = pose.pose.orientation = odom_quat;
    // Subscriber
    twist_sub = nh.subscribe(input_twist_topic_name, 100, &OdomSim::twist_callback, this);
    // Publisher
    odom_pub = nh.advertise<nav_msgs::Odometry>(output_odom_topic_name, 100);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ground_truth_pose", 100);
    // main loop
    ros::Rate loop_rate(odom_publish_hz);
    while (ros::ok())
    {
        publish_messages();
        ros::spinOnce();
        printf("x = %.3lf [m], y = %.3lf [m], yaw = %.3lf [deg]\n", robot_pose.x, robot_pose.y, robot_pose.yaw * 180.0 / M_PI);
        if (record_ground_truth_trajectory)
            record_ground_truth();
        loop_rate.sleep();
    }
}

void OdomSim::twist_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    static bool is_first = true;
    static double prev_time;
    if (is_first) 
    {
        prev_time = msg->header.stamp.toSec();
        is_first = false;
        return;
    }
    // update robot pose based on twist command
    double curr_time = msg->header.stamp.toSec();
    double delta_time = curr_time - prev_time;
    if (delta_time > 0.5)
    {
        prev_time = curr_time;
        is_first = true;
        return;
    }
    double delta_dist, delta_yaw;
    if (!use_ackermann_simulator && !use_omni_simulator) // differential drive
    {
        delta_dist = msg->twist.linear.x * delta_time;
        delta_yaw = msg->twist.angular.z * delta_time;
        robot_pose.x += delta_dist * cos(robot_pose.yaw + delta_yaw / 2.0);
        robot_pose.y += delta_dist * sin(robot_pose.yaw + delta_yaw / 2.0);
        robot_pose.yaw += delta_yaw;
    }
    else if (use_ackermann_simulator) // ackermann steering
    {
        delta_dist = msg->twist.linear.x * delta_time;
        steering_angle = msg->twist.angular.z; // unit of twist.angular.z must be radian
        double c = cos(robot_pose.yaw);
        double s = sin(robot_pose.yaw);
        if (steering_angle * steering_angle < 0.0001)
        {
            robot_pose.x += delta_dist * c;
            robot_pose.y += delta_dist * s; 
        }
        else
        {
            double k = tan(steering_angle) / base_line;
            double dw = msg->twist.linear.x * k;
            delta_yaw = dw * delta_time;
            robot_pose.yaw += delta_yaw;
            double c1 = cos(robot_pose.yaw);
            double s1 = sin(robot_pose.yaw);
            double r = 1.0 / k;
            robot_pose.x += r * (-s + s1);
            robot_pose.y += r * (c - c1);
        }
    }
    else // omni directional
    {
        double delta_dist_x = msg->twist.linear.x * delta_time;
        double delta_dist_y = msg->twist.linear.y * delta_time;
        double theta = robot_pose.yaw + delta_yaw / 2.0;
        delta_dist = sqrt(delta_dist_x * delta_dist_x + delta_dist_y * delta_dist_y);
        delta_yaw = msg->twist.angular.z * delta_time;
        robot_pose.x += delta_dist_x * cos(theta) + delta_dist_y * sin(theta);
        robot_pose.y += delta_dist_x * sin(theta) + delta_dist_y * cos(theta);
        robot_pose.yaw += delta_yaw;
    }
    mod_yaw(&robot_pose.yaw);
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_pose.yaw);
    odom.header.stamp = pose.header.stamp = msg->header.stamp;
    odom.pose.pose.position.x = pose.pose.position.x = robot_pose.x;
    odom.pose.pose.position.y = pose.pose.position.y = robot_pose.y;
    odom.pose.pose.position.z = pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = msg->twist.linear.x;
    odom.twist.twist.linear.y = msg->twist.linear.y;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.z = delta_yaw / delta_time;
    odom.twist.twist.angular.x = odom.twist.twist.angular.y = 0.0;
    prev_time = curr_time;
}

void OdomSim::publish_messages(void)
{
    odom_pub.publish(odom);
    pose_pub.publish(pose);
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(robot_pose.x, robot_pose.y, 0.0));
    q.setRPY(0.0, 0.0, robot_pose.yaw);
    transform.setRotation(q);
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), map_frame, "/ground_truth"));
    if (use_ackermann_simulator)
    {
        double c = cos(robot_pose.yaw);
        double s = sin(robot_pose.yaw);
        transform.setOrigin(tf::Vector3(robot_pose.x + base_line * c, robot_pose.y + base_line * s, 0.0));
        q.setRPY(0.0, 0.0, robot_pose.yaw + steering_angle);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), map_frame, "/ground_truth_front_link"));
    }
}

void OdomSim::record_ground_truth(void)
{
    static bool is_first = true;
    static pose_t prev_robot_pose;
    static FILE* fp;
    if (fp == NULL)
        fp = fopen("/tmp/odom_sim_ground_truth.txt", "w");
    if (is_first)
    {
        fprintf(fp, "%lf %lf %lf\n", robot_pose.x, robot_pose.y, robot_pose.yaw);
        prev_robot_pose = robot_pose;
        is_first = false;
    }
    else
    {
        double dx = robot_pose.x - prev_robot_pose.x;
        double dy = robot_pose.y - prev_robot_pose.y;
        double dl = sqrt(dx * dx + dy * dy);
        if (dl > 0.3)
        {
            fprintf(fp, "%lf %lf %lf\n", robot_pose.x, robot_pose.y, robot_pose.yaw);
            prev_robot_pose = robot_pose;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_sim");
    OdomSim node;
    return 0;
}