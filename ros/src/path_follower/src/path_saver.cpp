// Copyright © 2018 Naoki Akai. All rights reserved.

#include <ros/ros.h>
#include <nav_msgs/Path.h>

class PathSaver
{
private:
    ros::NodeHandle nh;
    std::string input_path_topic_name, path_file_name;
    ros::Subscriber path_sub;

public:
    PathSaver(std::string fname);
    ~PathSaver() {};
    void path_callback(const nav_msgs::Path::ConstPtr& msg);
};

PathSaver::PathSaver(std::string fname):
    nh("~"),
    input_path_topic_name("/recorded_path")
{
    // read parameters
    nh.param("input_path_topic_name", input_path_topic_name, input_path_topic_name);
    // subscriber
    path_sub = nh.subscribe(input_path_topic_name, 1, &PathSaver::path_callback, this);
    // wait for path
    path_file_name = fname;
    ros::spin();
}

void PathSaver::path_callback(const nav_msgs::Path::ConstPtr& msg)
{
    FILE* fp = fopen(path_file_name.c_str(), "w");
    if (fp == NULL)
    {
        ROS_ERROR("path saver could not open the give file -> %s", path_file_name.c_str());
        exit(-1);
    }
    for (int i = 0; i < msg->poses.size(); i++)
        fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf\n", msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, msg->poses[i].pose.position.z,
            msg->poses[i].pose.orientation.x, msg->poses[i].pose.orientation.y, msg->poses[i].pose.orientation.z, msg->poses[i].pose.orientation.w);
    fclose(fp);
    printf("path saver saved path data\n");
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_saver");
    if (argv[1] == NULL)
    {
        ROS_ERROR("path saver requires file name as argv[1]");
        exit(-1);
    }
    std::string fname(argv[1]);
    PathSaver node(fname);
    return 0;
}
