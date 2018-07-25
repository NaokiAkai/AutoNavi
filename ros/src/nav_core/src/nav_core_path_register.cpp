// Copyright © 2018 Naoki Akai. All rights reserved.

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>

typedef struct
{
    double x, y, yaw;
} pose_t;

class PathRegister
{
private:
    ros::NodeHandle nh;
    std::string map_frame, laser_frame, base_link_frame, root_dir_name;
    ros::Publisher map_pub, path_pub;
    ros::Subscriber odom_sub, scan_sub;
    std::string output_map_topic_name, output_path_topic_name;
    std::string input_odom_topic_name, input_scan_topic_name;
    tf::TransformListener tf_listener;
    nav_msgs::OccupancyGrid map;
    nav_msgs::Path path;
    sensor_msgs::LaserScan curr_scan;
    pose_t robot_pose, prev_robot_pose, base_link2laser;
    double map_size_x, map_size_y, map_resolution, map_origin_x, map_origin_y;
    double update_interval_dist, update_interval_angle;
    bool is_first_odom, is_first_update;
    int count;

public:
    PathRegister();
    ~PathRegister() {};
    void init(void);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void spin(void);
    bool do_update(void);
    void update_map(void);
    void update_cell(int u, int v, double dr);
    void update_path(void);
    void save_last_robot_pose(void);
    void save_map(void);
    void save_path(void);
    void save_count(void);
    void reset_map(void);
    void reset_path(void);
    void reset_robot_pose(void);
};

PathRegister::PathRegister():
    nh("~"),
    map_frame("/map"),
    laser_frame("/laser"),
    base_link_frame("/base_link"),
    root_dir_name("/home/akai/Dropbox/work/AutoNavi/ros/data/sample/"),
    output_map_topic_name("/amcl_map"),
    output_path_topic_name("/target_path"),
    input_odom_topic_name("/odom"),
    input_scan_topic_name("/scan"),
    map_size_x(300.0),
    map_size_y(300.0),
    map_resolution(0.1),
    map_origin_x(-150.0),
    map_origin_y(-150.0),
    update_interval_dist(0.5),
    update_interval_angle(5.0),
    tf_listener(),
    is_first_odom(true),
    is_first_update(true),
    count(0)
{
    // read parameters
    nh.param("/nav_core_path_register/map_frame", map_frame, map_frame);
    nh.param("/nav_core_path_register/laser_frame", laser_frame, laser_frame);
    nh.param("/nav_core_path_register/base_link_frame", base_link_frame, base_link_frame);
    nh.param("/nav_core_path_register/root_dir_name", root_dir_name, root_dir_name);
    nh.param("/nav_core_path_register/output_map_topic_name", output_map_topic_name, output_map_topic_name);
    nh.param("/nav_core_path_register/output_path_topic_name", output_path_topic_name, output_path_topic_name);
    nh.param("/nav_core_path_register/input_odom_topic_name", input_odom_topic_name, input_odom_topic_name);
    nh.param("/nav_core_path_register/input_scan_topic_name", input_scan_topic_name, input_scan_topic_name);
    nh.param("/nav_core_path_register/map_size_x", map_size_x, map_size_x);
    nh.param("/nav_core_path_register/map_size_y", map_size_y, map_size_y);
    nh.param("/nav_core_path_register/map_resolution", map_resolution, map_resolution);
    nh.param("/nav_core_path_register/map_origin_x", map_origin_x, map_origin_x);
    nh.param("/nav_core_path_register/map_origin_y", map_origin_y, map_origin_y);
    nh.param("/nav_core_path_register/update_interval_dist", update_interval_dist, update_interval_dist);
    nh.param("/nav_core_path_register/update_interval_angle", update_interval_angle, update_interval_angle);
    update_interval_angle *= M_PI / 180.0;
    robot_pose.x = robot_pose.y = robot_pose.yaw = 0.0;
    // subscriber
    odom_sub = nh.subscribe(input_odom_topic_name, 10, &PathRegister::odom_callback, this);
    scan_sub = nh.subscribe(input_scan_topic_name, 10, &PathRegister::scan_callback, this);
    // publisher
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>(output_map_topic_name, 1);
    path_pub = nh.advertise<nav_msgs::Path>(output_path_topic_name, 1);
    // initialization
    init();
}

void PathRegister::init(void)
{
    // get relative position of base link and laser sensor
    tf::StampedTransform tf_base_link2laser;
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        try
        {
            ros::Time now = ros::Time::now();
            tf_listener.waitForTransform(base_link_frame, laser_frame, now, ros::Duration(1.0));
            tf_listener.lookupTransform(base_link_frame, laser_frame, now, tf_base_link2laser);
            break;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            loop_rate.sleep();
        }
    }
    tf::Point zero_point(0.0, 0.0, 0.0);
    tf::Point laser_point_in_base_link_frame = tf_base_link2laser * zero_point;
    tf::Quaternion q(tf_base_link2laser.getRotation().x(),
        tf_base_link2laser.getRotation().y(),
        tf_base_link2laser.getRotation().z(),
        tf_base_link2laser.getRotation().w());
    double roll, pitch, yaw;
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    base_link2laser.x = laser_point_in_base_link_frame.getX();
    base_link2laser.y = laser_point_in_base_link_frame.getY();
    base_link2laser.yaw = yaw;
    // initialization of map
    map.header.frame_id = map_frame;
    map.info.width = (int)(map_size_x / map_resolution);
    map.info.height = (int)(map_size_y / map_resolution);
    map.info.resolution = map_resolution;
    map.info.origin.position.x = map_origin_x;
    map.info.origin.position.y = map_origin_y;
    map.data.resize(map.info.width * map.info.height, -1);
    // initialization of path
    path.header.frame_id = map_frame;
    path.poses.clear();
}

void PathRegister::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // updata robot pose
    static double prev_time;
    if (is_first_odom)
    {
        prev_time = msg->header.stamp.toSec();
        is_first_odom = false;
        return;
    }
    double curr_time = msg->header.stamp.toSec();
    double d_time = curr_time - prev_time;
    if (d_time > 1.0)
    {
        prev_time = curr_time;
        return;
    }
    double delta_dist = msg->twist.twist.linear.x * d_time;
    double delta_yaw = msg->twist.twist.angular.z * d_time;
    robot_pose.x += delta_dist * cos(robot_pose.yaw);
    robot_pose.y += delta_dist * sin(robot_pose.yaw);
    robot_pose.yaw += delta_yaw;
    if (robot_pose.yaw < -M_PI)    robot_pose.yaw += 2.0 * M_PI;
    if (robot_pose.yaw > M_PI)    robot_pose.yaw -= 2.0 * M_PI;
    prev_time = curr_time;
    // broadcast tf
    tf::Transform tf;
    tf::Quaternion q;
    static tf::TransformBroadcaster br;
    tf.setOrigin(tf::Vector3(robot_pose.x, robot_pose.y, 0.0));
    q.setRPY(0.0, 0.0, robot_pose.yaw);
    tf.setRotation(q);
    br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), map_frame, "/base_link"));
}

void PathRegister::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    curr_scan = *msg;
}

void PathRegister::spin(void)
{
    printf("start path registration\n");
    ros::Rate loop_rate(20.0);
    nh.setParam("/nav_core_path_register/change_map", false);
    while (ros::ok())
    {
        ros::spinOnce();
        if (do_update())
        {
            update_map();
            update_path();
            map_pub.publish(map);
            path_pub.publish(path);
            printf("update\n");
        }
        bool change_map;
        nh.getParam("/nav_core_path_register/change_map", change_map);
        if (change_map)
        {
            save_map();
            save_path();
            save_last_robot_pose();
            reset_map();
            reset_path();
            reset_robot_pose();
            map_pub.publish(map);
            path_pub.publish(path);
            printf("%d data was recorded\n", count);
            count++;
            nh.setParam("/nav_core_path_register/change_map", false);
        }
        loop_rate.sleep();
    }
    save_count();
}

bool PathRegister::do_update(void)
{
    if (is_first_update)
    {
        is_first_update = false;
        prev_robot_pose = robot_pose;
        return true;
    }
    double dx = robot_pose.x - prev_robot_pose.x;
    double dy = robot_pose.y - prev_robot_pose.y;
    double dl = sqrt(dx * dx + dy * dy);
    double dyaw = robot_pose.yaw - prev_robot_pose.yaw;
    if (dyaw < -M_PI)    dyaw += 2.0 * M_PI;
    if (dyaw > M_PI)    dyaw -= 2.0 * M_PI;
    if (dl >= update_interval_dist || fabs(dyaw) >= update_interval_angle)
    {
        prev_robot_pose = robot_pose;
        return true;
    }
    return false;
}

void PathRegister::update_map(void)
{
    // end-point model-based map update using binary Bayes filtering algorithm
    pose_t pose = robot_pose;
    sensor_msgs::LaserScan scan = curr_scan;
    double c = cos(pose.yaw);
    double s = sin(pose.yaw);
    double xo = base_link2laser.x * c - base_link2laser.y * s + pose.x;
    double yo = base_link2laser.x * s + base_link2laser.y * c + pose.y;
    for (int i = 0; i < scan.ranges.size(); i++)
    {
        double range = scan.ranges[i];
        if (range < scan.range_min || scan.range_max < range)
            continue;
        double yaw = pose.yaw + scan.angle_min + scan.angle_increment * (double)i;
        double dx = map.info.resolution * cos(yaw);
        double dy = map.info.resolution * sin(yaw);
        double x = xo;
        double y = yo;
        for (double r = 0.0; r < range - map.info.resolution; r += map.info.resolution)
        {
            double dx_ = x - map.info.origin.position.x;
            double dy_ = y - map.info.origin.position.y;
            int u = (int)(dx_ / map.info.resolution);
            int v = (int)(dy_ / map.info.resolution);
            if (0 <= u && u < map.info.width && 0 <= v && v < map.info.height)
                update_cell(u, v, range - r);
            x += dx;
            y += dy;
        }
        x = range * cos(yaw) + xo;
        y = range * sin(yaw) + yo;
        double dx_ = x - map.info.origin.position.x;
        double dy_ = y - map.info.origin.position.y;
        int u = (int)(dx_ / map.info.resolution);
        int v = (int)(dy_ / map.info.resolution);
        if (1 <= u && u < map.info.width - 1 && 1 <= v && v < map.info.height - 1)
        {
            update_cell(u, v, 0.0);
/*
            update_cell(u - 1, v, 0.0);
            update_cell(u + 1, v, 0.0);
            update_cell(u, v - 1, 0.0);
            update_cell(u, v + 1, 0.0);
 */
        }
    }
    map.header.stamp = ros::Time::now();
}

void PathRegister::update_cell(int u, int v, double dr)
{
    double z_hit = 0.90;
    double z_max = 0.05;
    double max_dist_prob = 0.043937;
    double z_hit_denom = 0.08;
    double z_rand = 0.05;
    double z_rand_mult = 0.033333;
    int node = v * map.info.width + u;
    int val = map.data[node];
    double po;
    if (val == -1)
        po = 0.5;
    else
        po = (double)val / 100.0;
    double z = dr;
    double pz = z_hit * exp(-(z * z) / z_hit_denom) + z_hit * max_dist_prob + z_rand * z_rand_mult;
    if (pz > 1.0)
        pz = 1.0;
    double l = log(po / (1.0 - po)) + log(pz / (1.0 - pz));
    double p = 1.0 / (1.0 + exp(-l));
    if (p < 0.01)
        p = 0.01;
    if (val > 0.99)
        val = 0.99;
    val = (int)(p * 100.0);
//    if (val < 5)
//        val = 0;
//    if (val > 95)
//        val = 100;
    map.data[node] = val;
}

void PathRegister::update_path(void)
{
    geometry_msgs::PoseStamped pose;
    tf::Transform tf;
    tf::Quaternion q;
    tf.setOrigin(tf::Vector3(robot_pose.x, robot_pose.y, 0.0));
    q.setRPY(0.0, 0.0, robot_pose.yaw);
    tf.setRotation(q);
    pose.pose.position.x = tf.getOrigin().x();
    pose.pose.position.y = tf.getOrigin().y();
    pose.pose.position.z = tf.getOrigin().z();
    pose.pose.orientation.x = tf.getRotation().x();
    pose.pose.orientation.y = tf.getRotation().y();
    pose.pose.orientation.z = tf.getRotation().z();
    pose.pose.orientation.w = tf.getRotation().w();
    path.header.stamp = pose.header.stamp = ros::Time::now();
    path.poses.push_back(pose);
}

void PathRegister::save_map(void)
{
    char fname[1024];
    sprintf(fname, "%sogm_%d.yaml", root_dir_name.c_str(), count);
    FILE* fp = fopen(fname, "w");
    fprintf(fp, "image: ogm_%d.pgm\n", count);
    fprintf(fp, "resolution: %lf\n", map.info.resolution);
    fprintf(fp, "origin: [%lf, %lf, 0.000000]\n", map.info.origin.position.x, map.info.origin.position.y);
    fprintf(fp, "negate: 0\n");
    fprintf(fp, "occupied_thresh: 0.65\n");
    fprintf(fp, "free_thresh: 0.196\n");
    fclose(fp);
    cv::Mat map_img(map.info.height, map.info.width, CV_8UC1);
    for (int u = 0; u < map.info.width; u++)
    {
        for (int v = 0; v < map.info.height; v++)
        {
            int node = v * map.info.width + u;
            int val = map.data[node];
            if (val == -1)
                val = 128;
            else if (val > 95)
                val = 0;
            else
                val = 255;
            map_img.at<uchar>(map.info.height - 1 - v, u) = val;
        }
    }
    sprintf(fname, "%sogm_%d.pgm", root_dir_name.c_str(), count);
    cv::imwrite(fname, map_img);
}

void PathRegister::save_path(void)
{
    char fname[1024];
    sprintf(fname, "%spath_%d.txt", root_dir_name.c_str(), count);
    FILE* fp = fopen(fname, "w");
    for (int i = 0; i < path.poses.size(); i++)
        fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf\n", path.poses[i].pose.position.x, path.poses[i].pose.position.y, path.poses[i].pose.position.z,
            path.poses[i].pose.orientation.x, path.poses[i].pose.orientation.y, path.poses[i].pose.orientation.z, path.poses[i].pose.orientation.w);
    fclose(fp);
}

void PathRegister::save_last_robot_pose(void)
{
    char fname[1024];
    sprintf(fname, "%slast_pose_%d.txt", root_dir_name.c_str(), count);
    FILE* fp = fopen(fname, "w");
    fprintf(fp, "%lf %lf %lf\n", robot_pose.x, robot_pose.y, robot_pose.yaw);
    fclose(fp);
}

void PathRegister::save_count(void)
{
    char fname[1024];
    sprintf(fname, "%ssub_map_num.txt", root_dir_name.c_str());
    FILE* fp = fopen(fname, "w");
    fprintf(fp, "%d\n", count);
    fclose(fp);
}

void PathRegister::reset_map(void)
{
    for (int i = 0; i < map.data.size(); i++)
        map.data[i] = -1;
    is_first_update = true;
}

void PathRegister::reset_path(void)
{
    path.poses.clear();
}

void PathRegister::reset_robot_pose(void)
{
    robot_pose.x = 0.0;
    robot_pose.y = 0.0;
    robot_pose.yaw = 0.0;
    is_first_odom = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_register");
    PathRegister node;
    node.spin();
    return 0;
}
