// Copyright © 2018 Naoki Akai. All rights reserved.

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class PathFollower
{
private:
    ros::NodeHandle nh;
    ros::Subscriber path_sub, max_vel_sub;
    ros::Publisher cmd_pub, twist_pub;
    std::string map_frame, base_link_frame;
    std::string input_path_topic_name, input_max_vel_topic_name, output_twist_topic_name;
    nav_msgs::Path path;
    tf::TransformListener tf_listener;
    double look_ahead_dist, max_vel, kv;
    double cmd_publish_hz;
    bool use_nav_core_server;
    int nearest_path_index, prev_nearest_path_index;

public:
    PathFollower();
    ~PathFollower() {};
    void path_callback(const nav_msgs::Path::ConstPtr& msg);
    void max_vel_callback(const std_msgs::Float32::ConstPtr& msg);
    void spin(void);
    void wait_for_new_path(void);
};

PathFollower::PathFollower():
    nh("~"),
    map_frame("/map"),
    base_link_frame("/base_link"),
    input_path_topic_name("/target_path"),
    input_max_vel_topic_name("/max_vel"),
    output_twist_topic_name("/twist_cmd"),
    look_ahead_dist(1.0),
    max_vel(1.0),
    kv(0.6),
    cmd_publish_hz(20.0),
    prev_nearest_path_index(-1),
    use_nav_core_server(false),
    tf_listener()
{
    // read parameters
    nh.param("map_frame", map_frame, map_frame);
    nh.param("base_link_frame", base_link_frame, base_link_frame);
    nh.param("input_path_topic_name", input_path_topic_name, input_path_topic_name);
    nh.param("input_max_vel_topic_name", input_max_vel_topic_name, input_max_vel_topic_name);
    nh.param("output_twist_topic_name", output_twist_topic_name, output_twist_topic_name);
    nh.param("look_ahead_dist", look_ahead_dist, look_ahead_dist);
    nh.param("max_vel", max_vel, max_vel);
    nh.param("kv", kv, kv);
    nh.param("cmd_publish_hz", cmd_publish_hz, cmd_publish_hz);
    nh.param("use_nav_core_server", use_nav_core_server, use_nav_core_server);
    // subscriber
    path_sub = nh.subscribe(input_path_topic_name, 1, &PathFollower::path_callback, this);
    max_vel_sub = nh.subscribe(input_max_vel_topic_name, 1, &PathFollower::max_vel_callback, this);
    // publisher
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    twist_pub = nh.advertise<geometry_msgs::TwistStamped>(output_twist_topic_name, 100);
}

void PathFollower::path_callback(const nav_msgs::Path::ConstPtr& msg)
{
    path = *msg;
    prev_nearest_path_index = -1;
}

void PathFollower::max_vel_callback(const std_msgs::Float32::ConstPtr& msg)
{
    max_vel = msg->data;
}

void PathFollower::spin(void)
{
    double eo = 0.0;
    ros::Rate loop_rate(cmd_publish_hz);
    while (ros::ok())
    {
        ros::spinOnce();
        // read robot pose (base_link_frame) from tf tree in map_frame
        ros::Time now = ros::Time::now();
        tf::StampedTransform map2base_link;
        try
        {
            tf_listener.waitForTransform(map_frame, base_link_frame, now, ros::Duration(1.0));
            tf_listener.lookupTransform(map_frame, base_link_frame, now, map2base_link);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            continue;
        }
        // x, y, and yaw mean 2d robot pose
        double x = map2base_link.getOrigin().x();
        double y = map2base_link.getOrigin().y();
        tf::Quaternion q(map2base_link.getRotation().x(), map2base_link.getRotation().y(), map2base_link.getRotation().z(), map2base_link.getRotation().w());
        double roll, pitch, yaw;
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        // search the nearest path point
        double min_dist;
        int i0, i1;
        if (prev_nearest_path_index < 0)
        {
            i0 = 1;
            i1 = path.poses.size();
        }
        else
        {
            i0 = prev_nearest_path_index - 30;
            i1 = prev_nearest_path_index + 30;
            if (i0 < 1)
                i0 = 1;
            if (i1 >= path.poses.size())
                i1 = path.poses.size();
        }
        for (int i = i0; i < i1; i++)
        {
            double dx = path.poses[i].pose.position.x - x;
            double dy = path.poses[i].pose.position.y - y;
            double dl = sqrt(dx * dx + dy * dy);
            if (i == i0)
            {
                nearest_path_index = i;
                min_dist = dl;
            }
            else if (dl < min_dist)
            {
                nearest_path_index = i;
                min_dist = dl;
            }
        }
        prev_nearest_path_index = nearest_path_index;
        // search the target path point
        int target_path_index = -1;
        for (int i = nearest_path_index; i < path.poses.size(); i++)
        {
            double dx = path.poses[i].pose.position.x - x;
            double dy = path.poses[i].pose.position.y - y;
            double dl = sqrt(dx * dx + dy * dy);
            if (dl >= look_ahead_dist)
            {
                target_path_index = i;
                break;
            }
        }
        // determine twist command
        geometry_msgs::Twist cmd_vel;
        geometry_msgs::TwistStamped twist_cmd;
        twist_cmd.header.stamp = ros::Time::now();
        bool stop;
        nh.getParam("/path_follower/stop", stop);
        if (target_path_index < 0 || path.poses.size() == 0 || stop)
        {
            // reach at the goal point or no path data
            cmd_vel.linear.x = twist_cmd.twist.linear.x = 0.0;
            cmd_vel.angular.z = twist_cmd.twist.angular.z = 0.0;
            eo = 0.0;
            if (path.poses.size() != 0 && !stop && use_nav_core_server)
                wait_for_new_path();
        }
        else
        {
            // compute deviation from the path and determine twist_cmd based on PD control
            double dx = path.poses[target_path_index].pose.position.x - path.poses[target_path_index - 1].pose.position.x;
            double dy = path.poses[target_path_index].pose.position.y - path.poses[target_path_index - 1].pose.position.y;
            double th = atan2(dy, dx);
            double x2p = dx * cos(th) + dy * sin(th);
            dx = path.poses[target_path_index].pose.position.x - x;
            dy = path.poses[target_path_index].pose.position.y - y;
            double xrp = dx * cos(th) + dy * sin(th);
            double yrp = -dx * sin(th) + dy * cos(th);
            double t = atan2(dy, dx);
            double e = t - yaw;
            if (e < -M_PI)    e += 2.0 * M_PI;
            if (e > M_PI)    e -= 2.0 * M_PI;
            cmd_vel.linear.x = twist_cmd.twist.linear.x = max_vel - kv * fabs(e);
            cmd_vel.angular.z = twist_cmd.twist.angular.z = 0.4 * e + 0.01 * eo;
            eo = e;
        }
        cmd_pub.publish(cmd_vel);
        twist_pub.publish(twist_cmd);
        // print debug message
        printf("robot pose: x = %.3lf [m], y = %.3lf [m], yaw = %.3lf [deg]\n", x, y, yaw * 180.0 / M_PI);
        printf("target path point: x = %.3lf [m], y = %.3lf [m]\n", path.poses[target_path_index].pose.position.x, path.poses[target_path_index].pose.position.y);
        printf("target path index = %d, path point num = %d\n", target_path_index, (int)path.poses.size());
        printf("twist command: vel = %.3lf [m/sec], ang_vel = %.3lf [rad/sec]\n", twist_cmd.twist.linear.x, twist_cmd.twist.angular.z);
        printf("\n");
        loop_rate.sleep();
    }
}

void PathFollower::wait_for_new_path(void)
{
    nh.setParam("/nav_params/reach_at_goal", true);
    bool is_new_path_data = false, is_new_map_data = false, finish_navigation = false;
    ros::Rate loop_rate(10.0);
    while (ros::ok())
    {
        ros::spinOnce();
        geometry_msgs::TwistStamped twist_cmd;
        twist_cmd.header.stamp = ros::Time::now();
        twist_cmd.twist.linear.x = 0.0;
        twist_cmd.twist.angular.z = 0.0;
        twist_pub.publish(twist_cmd);
        if (prev_nearest_path_index < 0 && !is_new_path_data)
            is_new_path_data = true;
        nh.getParam("/nav_params/is_new_map_data", is_new_map_data);
        if (is_new_map_data)
            nh.setParam("/nav_params/is_new_map_data", false);
        nh.getParam("/nav_params/finish_navigation", finish_navigation);
        if (finish_navigation)
        {
            printf("finish navigation from path_follower\n");
            exit(0);
        }
        if (is_new_path_data && is_new_map_data)
            break;
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_follower");
    PathFollower node;
    node.spin();
    return 0;
}
