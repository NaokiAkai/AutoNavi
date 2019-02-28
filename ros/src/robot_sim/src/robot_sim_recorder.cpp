// Copyright © 2018 Naoki Akai. All rights reserved.

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <robot_sim/common.h>

class RobotSimRecorder
{
private:
    ros::NodeHandle nh;
    std::string root_dir;
    std::string map_frame;
    std::string input_odom_topic_name, input_scan_topic_name;
    ros::Subscriber odom_sub, scan_sub;
    double record_hz;
    nav_msgs::Odometry curr_odom;
    sensor_msgs::LaserScan curr_scan;
    tf::TransformListener tf_listener;
    pose_t ground_truth_pose;
    ros::Time odom_time;
    bool is_writing;
    FILE *fp_odom, *fp_laser, *fp_gt_pose;

public:
    RobotSimRecorder();
    ~RobotSimRecorder() {};
    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
    bool get_ground_truth_pose(void);
    void record();
    void spin(void);
};

RobotSimRecorder::RobotSimRecorder():
    nh("~"),
    root_dir("/tmp/robot_sim_data/"),
    map_frame("/map"),
    input_odom_topic_name("/odom"),
    input_scan_topic_name("/scan"),
    record_hz(10.0),
    tf_listener(),
    is_writing(false)
{
    // read parameters
    nh.param("root_dir", root_dir, root_dir);
    nh.param("input_odom_topic_name", input_odom_topic_name, input_odom_topic_name);
    nh.param("input_scan_topic_name", input_scan_topic_name, input_scan_topic_name);
    nh.param("record_hz", record_hz, record_hz);
    // subscriber
    odom_sub = nh.subscribe(input_odom_topic_name, 10, &RobotSimRecorder::odom_callback, this);
    scan_sub = nh.subscribe(input_scan_topic_name, 10, &RobotSimRecorder::scan_callback, this);
    // initialize files
    char fname[1024];
    sprintf(fname, "%sodom.txt", root_dir.c_str());
    fp_odom = fopen(fname, "w");
    sprintf(fname, "%slaser.txt", root_dir.c_str());
    fp_laser = fopen(fname, "w");
    sprintf(fname, "%sgt_pose.txt", root_dir.c_str());
    fp_gt_pose = fopen(fname, "w");
    if (fp_odom == NULL || fp_laser == NULL || fp_gt_pose == NULL)
    {
        ROS_ERROR("could not open recording files in the directory -> %s", root_dir.c_str());
        exit(1);
    }
}

void RobotSimRecorder::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (!is_writing)
    {
        odom_time = msg->header.stamp;
        curr_odom = *msg;
    }
}

void RobotSimRecorder::scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    static bool is_first = true;
    if (is_first)
    {
        char fname[1024];
        sprintf(fname, "%sscan_params.txt", root_dir.c_str());
        FILE *fp = fopen(fname, "w");
        if (fp == NULL)
        {
            ROS_ERROR("could not open %s", fname);
            exit(1);
        }
        fprintf(fp, "angle_min %lf\n", msg->angle_min);
        fprintf(fp, "angle_max %lf\n", msg->angle_max);
        fprintf(fp, "angle_increment %lf\n", msg->angle_increment);
        fprintf(fp, "time_increment %lf\n", msg->time_increment);
        fprintf(fp, "scan_time %lf\n", msg->scan_time);
        fprintf(fp, "range_min %lf\n", msg->range_min);
        fprintf(fp, "range_max %lf\n", msg->range_max);
        fprintf(fp, "range_size %d\n", (int)msg->ranges.size());
        fclose(fp);
        is_first = false;
    }
    if (!is_writing)
        curr_scan = *msg;
}

bool RobotSimRecorder::get_ground_truth_pose(void)
{
    tf::StampedTransform map2gt;
    try
    {
        tf_listener.waitForTransform(map_frame, "ground_truth", odom_time, ros::Duration(0.1));
        tf_listener.lookupTransform(map_frame, "ground_truth", odom_time, map2gt);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return false;
    }
    ground_truth_pose.x = map2gt.getOrigin().x();
    ground_truth_pose.y = map2gt.getOrigin().y();
    tf::Quaternion q(map2gt.getRotation().x(),
        map2gt.getRotation().y(),
        map2gt.getRotation().z(),
        map2gt.getRotation().w());
    double roll, pitch;
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, ground_truth_pose.yaw);
    return true;
}

void RobotSimRecorder::record(void)
{
    is_writing = true;

    // write odom
    fprintf(fp_odom, "%lf %lf\n", curr_odom.twist.twist.linear.x, curr_odom.twist.twist.angular.z);

    // write laser
    for (int i = 0; i < (int)curr_scan.ranges.size(); i++)
        fprintf(fp_laser, "%lf ", curr_scan.ranges[i]);
    fprintf(fp_laser, "\n");

    // write gt pose
    fprintf(fp_gt_pose, "%lf %lf %lf\n", ground_truth_pose.x, ground_truth_pose.y, ground_truth_pose.yaw);

    is_writing = false;
}

void RobotSimRecorder::spin(void)
{
    fprintf(stderr, "Ready to record? Put the enter to start recording\n");
    getchar();

    ros::Rate loop_rate(record_hz);
    while (ros::ok())
    {
        ros::spinOnce();
        if (!get_ground_truth_pose())
        {
            ROS_ERROR("failed to get ground truth pose");
            continue;
        }
        record();
        fprintf(stderr, "recorded at time %lf\n", odom_time.toSec());
        loop_rate.sleep();
    }
    fclose(fp_odom);
    fclose(fp_laser);
    fclose(fp_gt_pose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_sim_recorder");
    RobotSimRecorder node;
    node.spin();
    return 0;
}
