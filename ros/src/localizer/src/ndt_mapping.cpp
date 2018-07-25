// Copyright © 2018 Naoki Akai. All rights reserved.

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <localizer/ndt.h>

class NDTMapping
{
private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Publisher mean_points_pub, grid_lines_pub, ellipses_pub;
    std::string input_scan_topic_name;
    std::string map_frame, laser_frame;
    float map_size_x, map_size_y, map_grid_size, map_origin_x, map_origin_y;
    std::string map_file_name;
    NDT ndt;
    double mapping_interval_dist, mapping_interval_angle;
    tf::TransformListener tf_listener;
    double xo, yo, yawo;

public:
    NDTMapping();
    ~NDTMapping() {};
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
};

NDTMapping::NDTMapping():
    nh("~"),
    input_scan_topic_name("/scan"),
    map_frame("/map"),
    laser_frame("/laser"),
    map_size_x(700.0f),
    map_size_y(600.0f),
    map_grid_size(0.5f),
    map_origin_x(-200.0f),
    map_origin_y(-200.0f),
    map_file_name("/tmp/ndt_map.txt"),
    mapping_interval_dist(0.5),
    mapping_interval_angle(10.0),
    tf_listener(),
    xo(0.0),
    yo(0.0),
    yawo(0.0)
{
    // read parameters
    nh.param("/ndt_mapping/input_scan_topic_name", input_scan_topic_name, input_scan_topic_name);
    nh.param("/ndt_mapping/map_frame", map_frame, map_frame);
    nh.param("/ndt_mapping/laser_frame", laser_frame, laser_frame);
    nh.param("/ndt_mapping/map_size_x", map_size_x, map_size_x);
    nh.param("/ndt_mapping/map_size_y", map_size_y, map_size_y);
    nh.param("/ndt_mapping/map_grid_size", map_grid_size, map_grid_size);
    nh.param("/ndt_mapping/map_origin_x", map_origin_x, map_origin_x);
    nh.param("/ndt_mapping/map_origin_y", map_origin_y, map_origin_y);
    nh.param("/ndt_mapping/map_file_name", map_file_name, map_file_name);
    nh.param("/ndt_mapping/mapping_interval_dist", mapping_interval_dist, mapping_interval_dist);
    nh.param("/ndt_mapping/mapping_interval_angle", mapping_interval_angle, mapping_interval_angle);
    nh.param("/ndt_mapping/xo", xo, xo);
    nh.param("/ndt_mapping/yo", yo, yo);
    nh.param("/ndt_mapping/yawo", yawo, yawo);
    printf("map_size_x = %f\n", map_size_x);
    printf("map_size_y = %f\n", map_size_y);
    printf("map_grid_size = %f\n", map_grid_size);
    printf("map_origin_x = %f\n", map_origin_x);
    printf("map_origin_y = %f\n", map_origin_y);
    printf("xo = %lf\n", xo);
    printf("yo = %lf\n", yo);
    printf("yawo = %lf\n", yawo);
    mapping_interval_angle *= M_PI / 180.0;
    // subscriber
    scan_sub = nh.subscribe(input_scan_topic_name, 10, &NDTMapping::scan_callback, this);
    // publisher
    mean_points_pub = nh.advertise<sensor_msgs::PointCloud>("/ndt_map_mean_points", 10);
    grid_lines_pub = nh.advertise<visualization_msgs::Marker>("/ndt_map_grid_lines", 10);
    ellipses_pub = nh.advertise<visualization_msgs::MarkerArray>("/ndt_map_ellipses", 10);
    // init
    ndt.init_ndt_map(map_size_x, map_size_y, map_grid_size, map_origin_x, map_origin_y);
    ndt.set_min_points_num(5);
    visualization_msgs::Marker grid_lines = ndt.get_grid_lines_of_ndt_map(map_frame);
    // main loop
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        ros::spinOnce();
        sensor_msgs::PointCloud mean_points = ndt.get_mean_points_of_ndt_map(map_frame, 50.0f, (float)xo, (float)yo);
        visualization_msgs::MarkerArray ellipses = ndt.get_ellipses_of_ndt_map(map_frame, 50.0f, (float)xo, (float)yo);
        mean_points_pub.publish(mean_points);
        grid_lines_pub.publish(grid_lines);
        ellipses_pub.publish(ellipses);
        loop_rate.sleep();
    }
    ndt.save_ndt_map(map_file_name);
}

void NDTMapping::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    static bool is_first = true;
    static double prev_xo, prev_yo, prev_yawo;
    tf::StampedTransform map2laser;
    try
    {
        tf_listener.waitForTransform(map_frame, msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));
        tf_listener.lookupTransform(map_frame, msg->header.frame_id, msg->header.stamp, map2laser);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
    xo = map2laser.getOrigin().x();
    yo = map2laser.getOrigin().y();
    tf::Quaternion q(map2laser.getRotation().x(), map2laser.getRotation().y(), map2laser.getRotation().z(), map2laser.getRotation().w());
    double rollo, pitcho;
    tf::Matrix3x3 m(q);
    m.getRPY(rollo, pitcho, yawo);
    bool do_mapping = false;
    if (is_first)
    {
        do_mapping = true;
        is_first = false;
    }    
    else
    {
        double dx = xo - prev_xo;
        double dy = yo - prev_yo;
        double dl = sqrt(dx * dx + dy * dy);
        double dyaw = yawo - prev_yawo;
        if (dyaw < -M_PI)    dyaw += 2.0 * M_PI;
        if (dyaw > M_PI)    dyaw -= 2.0 * M_PI;
        if (dl >= mapping_interval_dist || fabs(dyaw) > mapping_interval_angle)
            do_mapping = true;
    }
    if (do_mapping)
    {
        // decrease occupancy rate
        for (int i = 0; i < msg->ranges.size(); i++)
        {
            float r = msg->ranges[i];
            if (r < msg->range_min || msg->range_max < r)
                continue;
            float yaw = (float)yawo + msg->angle_min + msg->angle_increment * (float)i;
            float dx = ndt.ndt_map_grid_size * cos(yaw);
            float dy = ndt.ndt_map_grid_size * sin(yaw);
            float x = (float)xo;
            float y = (float)yo;
            float dr = ndt.ndt_map_grid_size;
            for (float l = 0.0f; l < 0.9f * r; l += dr)
            {
                ndt.decrease_occupancy_rate_of_ndt_map(x, y, r - l);
                x += dx;
                y += dy;
            }
        }
        // add points to the ndt map
        for (int i = 0; i < msg->ranges.size(); i++)
        {
            float r = msg->ranges[i];
            if (r < msg->range_min || msg->range_max < r)
                continue;
            float yaw = (float)yawo + msg->angle_min + msg->angle_increment * (float)i;
            float x = r * cos(yaw) + (float)xo;
            float y = r * sin(yaw) + (float)yo;
//            printf("i = %d, x = %f, y = %f\n", i, x, y);
            ndt.add_point_to_ndt_map(x, y);
        }
        prev_xo = xo;
        prev_yo = yo;
        prev_yawo = yawo;
        printf("x = %.3lf [m], y = %.3lf [m], yaw = %.2lf [degree]\n", xo, yo, yawo * 180.0 / M_PI);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ndt_mapping");
    NDTMapping node;
    return 0;
}
