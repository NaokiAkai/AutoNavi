/****************************************************************************
 * Copyright (C) 2018 Naoki Akai.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 ****************************************************************************/

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <opencv2/opencv.hpp>
#include <localizer/amcl.h>

AMCL::AMCL():
    nh("~"),
    map_frame("/world"),
    laser_frame("/laser"),
    base_link_frame("/base_link"),
    input_map_topic_name("/amcl_map"),
    input_odom_topic_name("/odom"),
    input_scan_topic_name("/scan"),
    min_particle_num(100),
    max_particle_num(1000),
    resample_threshold(0.5),
    scan_step(10),
    max_dist_to_obstacle(0.5),
    alpha_slow(0.0001),
    alpha_fast(0.1),
    delta_dist(0.0),
    delta_yaw(0.0),
    update_dist(0.2),
    update_yaw(2.0),
    update_time(5.0),
    odom_noise_dist_dist(1.0),
    odom_noise_dist_head(0.7),
    odom_noise_head_dist(0.7),
    odom_noise_head_head(1.0),
    start_x(0.0),
    start_y(0.0),
    start_yaw(0.0),
    initial_cov_xx(0.5),
    initial_cov_yy(0.5),
    initial_cov_yawyaw(3.0),
    pose_publish_hz(20.0),
    use_kld_sampling(false),
    use_test_range_measurement(false),
    add_random_particle(false),
    add_random_particle_rate(0.1),
    dynamic_scan_point_threshold(0.9),
    z_hit(0.90),
    z_short(0.10),
    z_max(0.05),
    z_rand(0.05),
    z_hit_var(0.1),
    lambda_short(0.10),
    max_dist_prob(0.043937),
    z_rand_mult(0.033333),
    use_beam_model(false),
    use_class_conditional_observation_model(false),
    is_map_data(false),
    is_scan_data(false),
    is_first_time(true),
    is_tf_initialized(false),
    is_initial_pose(false)
{
    // read parameters
    nh.param("map_frame", map_frame, map_frame);
    nh.param("laser_frame", laser_frame, laser_frame);
    nh.param("base_link_frame", base_link_frame, base_link_frame);
    nh.param("input_map_topic_name", input_map_topic_name, input_map_topic_name);
    nh.param("input_odom_topic_name", input_odom_topic_name, input_odom_topic_name);
    nh.param("input_scan_topic_name", input_scan_topic_name, input_scan_topic_name);
    nh.param("min_particle_num", min_particle_num, min_particle_num);
    nh.param("max_particle_num", max_particle_num, max_particle_num);
    nh.param("resample_threshold", resample_threshold, resample_threshold);
    nh.param("scan_step", scan_step, scan_step);
    nh.param("max_dist_to_obstacle", max_dist_to_obstacle, max_dist_to_obstacle);
    nh.param("alpha_slow", alpha_slow, alpha_slow);
    nh.param("alpha_fast", alpha_fast, alpha_fast);
    nh.param("update_dist", update_dist, update_dist);
    nh.param("update_yaw", update_yaw, update_yaw);
    nh.param("update_time", update_time, update_time);
    nh.param("odom_noise_dist_dist", odom_noise_dist_dist, odom_noise_dist_dist);
    nh.param("odom_noise_dist_head", odom_noise_dist_head, odom_noise_dist_head);
    nh.param("odom_noise_head_dist", odom_noise_head_dist, odom_noise_head_dist);
    nh.param("odom_noise_head_head", odom_noise_head_head, odom_noise_head_head);
    nh.param("use_omni_odom", use_omni_odom, use_omni_odom);
    nh.param("start_x", start_x, start_x);
    nh.param("start_y", start_y, start_y);
    nh.param("start_yaw", start_yaw, start_yaw);
    nh.param("initial_cov_xx", initial_cov_xx, initial_cov_xx);
    nh.param("initial_cov_yy", initial_cov_yy, initial_cov_yy);
    nh.param("initial_cov_yawyaw", initial_cov_yawyaw, initial_cov_yawyaw);
    nh.param("pose_publish_hz", pose_publish_hz, pose_publish_hz);
    nh.param("use_kld_sampling", use_kld_sampling, use_kld_sampling);
    nh.param("use_test_range_measurement", use_test_range_measurement, use_test_range_measurement);
    nh.param("add_random_particle", add_random_particle, add_random_particle);
    nh.param("add_random_particle_rate", add_random_particle_rate, add_random_particle_rate);
    nh.param("dynamic_scan_point_threshold", dynamic_scan_point_threshold, dynamic_scan_point_threshold);
    nh.param("z_hit", z_hit, z_hit);
    nh.param("z_short", z_short, z_short);
    nh.param("z_max", z_max, z_max);
    nh.param("z_rand", z_rand, z_rand);
    nh.param("z_hit_var", z_hit_var, z_hit_var);
    nh.param("lambda_short", lambda_short, lambda_short);
    nh.param("max_dist_prob", max_dist_prob, max_dist_prob);
    nh.param("z_rand_mult", z_rand_mult, z_rand_mult);
    nh.param("use_beam_model", use_beam_model, use_beam_model);
    nh.param("use_class_conditional_observation_model", use_class_conditional_observation_model, use_class_conditional_observation_model);
    // check values and flags
    // check value of resample threshold
    if (resample_threshold < 0.0 || 1.0 < resample_threshold)
    {
        ROS_ERROR("resample_threshold must be included from 0 to 1 (0.5 is recommended)");
        exit(1);
    }
    // check dynamic scan point threshold when rejection algorithm will be used
    if (use_test_range_measurement || use_class_conditional_observation_model)
    {
        if (dynamic_scan_point_threshold < 0.0 || 1.0 < dynamic_scan_point_threshold)
        {
            ROS_ERROR("dynamic_scan_point_threshold must be included from 0 to 1 (0.9 is recommended)");
            exit(1);
        }
    }
    // beam model and test range measurement will not be used if use_class_conditional_observation_model = true
    if (use_class_conditional_observation_model)
    {
        use_beam_model = false;
        use_test_range_measurement = false;
        ROS_INFO("beam model will not be used because use_class_conditional_observation_model is true");
    }
    // convert degree to radian
    update_yaw *= M_PI / 180.0;
    start_yaw *= M_PI / 180.0;
    initial_cov_yawyaw *= M_PI / 180.0;
    // set initial state
    robot_pose.x = start_x;
    robot_pose.y = start_y;
    robot_pose.yaw = start_yaw;
    particle_num = min_particle_num;
    if (add_random_particle && use_kld_sampling)
    {
        use_kld_sampling = false;
        ROS_INFO("KLD sampling will not be used because add_random_particle is true");
    }
    if (!use_kld_sampling)
        particle_num = max_particle_num;
    if (add_random_particle)
    {
        max_particle_num += (int)((double)max_particle_num * add_random_particle_rate);
        ROS_INFO("ACML adds random particles about %.1lf %% when resampling", add_random_particle_rate * 100.0);
    }
    // compute parameters
    norm_const_hit = 1.0 / (sqrt(2.0 * M_PI) * z_hit_var);
    // subscriber
    pose_sub = nh.subscribe("/initialpose", 1, &AMCL::initial_pose_callback, this);
    map_sub = nh.subscribe(input_map_topic_name, 1, &AMCL::map_callback, this);
    odom_sub = nh.subscribe(input_odom_topic_name, 100, &AMCL::odom_callback, this);
    scan_sub = nh.subscribe(input_scan_topic_name, 1, &AMCL::scan_callback, this);
    // publisher
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/amcl_pose", 1);
    particles_pub = nh.advertise<geometry_msgs::PoseArray>("/amcl_particles", 1);
    uscan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan_used_for_localization", 1);
    dscan_pub = nh.advertise<sensor_msgs::LaserScan>("/dynamic_scan", 1);
    upoints_pub = nh.advertise<sensor_msgs::PointCloud>("/points_used_for_localization", 1);
    dpoints_pub = nh.advertise<sensor_msgs::PointCloud>("/dynamic_scan_points", 1);
    likelihood_dist_map_pub = nh.advertise<grid_map_msgs::GridMap>("/likelihood_dist_map", 1);
    matching_error_pub = nh.advertise<sensor_msgs::LaserScan>("/scan_residual_errors", 1);
    expected_distances_pub = nh.advertise<sensor_msgs::LaserScan>("/expected_scan", 1);
    // initialization
    amcl_init();
}

double AMCL::nrand(double n)
{
    return (n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX));
}

void AMCL::start_timer(void)
{
    timer_start = ros::Time::now().toSec();
}

void AMCL::stop_timer(void)
{
    timer_end = ros::Time::now().toSec();
}

double AMCL::get_timer_data(void)
{
    return (timer_end - timer_start);
}

void AMCL::xy2uv(double x, double y, int* u, int* v)
{
    double dx = x - map.info.origin.position.x;
    double dy = y - map.info.origin.position.y;
    *u = (int)(dx / map.info.resolution);
    *v = (int)(dy / map.info.resolution);
}

void AMCL::uv2xy(int u, int v, double* x, double* y)
{
    double dx = (double)u * map.info.resolution;
    double dy = (double)v * map.info.resolution;
    *x = dx + map.info.origin.position.x;
    *y = dy + map.info.origin.position.y;
}

void AMCL::reset_particles(void)
{
    double wo = 1.0 / (double)particle_num;
    for (int i = 0; i < particle_num; i++)
    {
        particles[i].pose.x = robot_pose.x + nrand(initial_cov_xx);
        particles[i].pose.y = robot_pose.y + nrand(initial_cov_yy);
        particles[i].pose.yaw = robot_pose.yaw + nrand(initial_cov_yawyaw);
        while (particles[i].pose.yaw < -M_PI)
            particles[i].pose.yaw += 2.0 * M_PI;
        while (particles[i].pose.yaw > M_PI)
            particles[i].pose.yaw -= 2.0 * M_PI;
        particles[i].w = wo;
    }
    w_slow = w_fast = 0.0;
}

void AMCL::amcl_init(void)
{
    // initilizatioin of pf
    particles.resize(max_particle_num);
    reset_particles();
    // initilizatioin of tf to know relative position of base link and laser
//    tf::TransformListener tf_listener;
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
    is_tf_initialized = true;
}

bool AMCL::get_pose_from_tf(std::string source_frame, std::string target_frame, ros::Time target_time, double duration, pose_t* pose, ros::Time *stamp)
{
    tf::StampedTransform tf_s2t;
    try
    {
        tf_listener.waitForTransform(source_frame, target_frame, target_time, ros::Duration(duration));
        tf_listener.lookupTransform(source_frame, target_frame, target_time, tf_s2t);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return false;
    }
    tf::Quaternion q(tf_s2t.getRotation().x(), tf_s2t.getRotation().y(), tf_s2t.getRotation().z(), tf_s2t.getRotation().w());
    double roll, pitch, yaw;
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    pose->x = tf_s2t.getOrigin().x();
    pose->y = tf_s2t.getOrigin().y();
    pose->yaw = yaw;
    *stamp = tf_s2t.stamp_;
    return true;
}

void AMCL::initial_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    tf::Quaternion q(msg->pose.pose.orientation.x, 
        msg->pose.pose.orientation.y, 
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    robot_pose.x = msg->pose.pose.position.x;
    robot_pose.y = msg->pose.pose.position.y;
    robot_pose.yaw = yaw;
    reset_particles();
    is_first_time = true;
    is_initial_pose = true;
}

void AMCL::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map = *msg;
    // build distance map from a target point to the nearest obstacle (occupied cell)
/*
    int dr = (int)(max_dist_to_obstacle / map.info.resolution);
    dist_map.resize(map.info.width);
    for (int u = 0; u < map.info.width; u++)
        dist_map[u].resize(map.info.height);
    for (int u = 0; u < map.info.width; u++)
    {
        for (int v = 0; v < map.info.height; v++)
        {
            float min_d;
            bool is_first = true;
            for (int uu = u - dr; uu <= u + dr; uu++)
            {
                for (int vv = v - dr; vv <= v + dr; vv++)
                {
                    if (0 <= uu && uu < map.info.width && 0 <= vv && vv < map.info.height)
                    {
                        int node = vv * map.info.width + uu;
                        if (map.data[node] == 100)
                        {
                            if (is_first)
                            {
                                float du = (float)(uu - u);
                                float dv = (float)(vv - v);
                                min_d = sqrt(du * du + dv * dv);
                                is_first = false;
                            }
                            else
                            {
                                float du = (float)(uu - u);
                                float dv = (float)(vv - v);
                                float d = sqrt(du * du + dv * dv);
                                if (d < min_d)
                                    min_d = d;
                            }
                        }
                    }
                }
            }
            min_d *= map.info.resolution;
            if (!is_first && min_d < max_dist_to_obstacle)
                dist_map[u][v] = min_d;
            else
                dist_map[u][v] = max_dist_to_obstacle;
        }
    }
 */
    // use the distance transform algorithm contained in OpenCV
    // because this is faster than above my code...
    cv::Mat occ_map((int)map.info.height, (int)map.info.width, CV_8UC1);
    for (int v = 0; v < occ_map.rows; v++) {
        for (int u = 0; u < occ_map.cols; u++) {
            int node = v * (int)map.info.width + u;
            int val = map.data[node];
            if (val == 100)
                occ_map.at<uchar>(v, u) = 0;
            else
                occ_map.at<uchar>(v, u) = 1;
        }
    }
    cv::Mat dist((int)map.info.height, (int)map.info.width, CV_32FC1);
    cv::distanceTransform(occ_map, dist, CV_DIST_L2, 5);
    for (int v = 0; v < occ_map.rows; v++) {
        for (int u = 0; u < occ_map.cols; u++) {
            float d = dist.at<float>(v, u) * map.info.resolution;
            dist.at<float>(v, u) = d;
        }
    }
    dist_map = dist;
    is_map_data = true;
    printf("got a map\n");
}

void AMCL::save_map_as_txt_file(std::string fname)
{
    FILE* fp = fopen(fname.c_str(), "w");
    if (fp == NULL)
    {
        ROS_ERROR("cannot open file -> %s", fname.c_str());
        return;
    }
    if (!is_map_data)
    {
        ROS_ERROR("no map data in [save_map_as_txt_file()]");
        return;
    }
    for (int u = 0; u < (int)map.info.width; u++)
    {
        for (int v = 0; v < (int)map.info.height; v++)
        {
            int node = v * map.info.width + u;
            if (map.data[node] == 100)
            {
                double x, y;
                uv2xy(u, v, &x, &y);
                fprintf(fp, "%lf %lf\n", x, y);
            }
        }
    }
    fclose(fp);
    printf("saved the amcl map as a text file [%s]\n", fname.c_str());
}

void AMCL::broadcast_tf(void)
{
    tf::Transform tf;
    tf::Quaternion q;
    static tf::TransformBroadcaster br;
    tf.setOrigin(tf::Vector3(robot_pose.x, robot_pose.y, 0.0));
    q.setRPY(0.0, 0.0, robot_pose.yaw);
    tf.setRotation(q);
    br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), map_frame, "/amcl_frame"));
}

void AMCL::publish_pose(void)
{
    geometry_msgs::PoseStamped pose;
    tf::Transform tf;
    tf::Quaternion q;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = map_frame;
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
    pose_pub.publish(pose);
}

void AMCL::publish_particles(void)
{
    geometry_msgs::Pose pose;
    geometry_msgs::PoseArray poses;
    tf::Transform tf;
    tf::Quaternion q;
    poses.header.stamp = ros::Time::now();
    poses.header.frame_id = map_frame;
    for (int i = 0; i < particle_num; i++)
    {
        tf.setOrigin(tf::Vector3(particles[i].pose.x, particles[i].pose.y, 0.0));
        q.setRPY(0.0, 0.0, particles[i].pose.yaw);
        tf.setRotation(q);
        pose.position.x = tf.getOrigin().x();
        pose.position.y = tf.getOrigin().y();
        pose.position.z = tf.getOrigin().z();
        pose.orientation.x = tf.getRotation().x();
        pose.orientation.y = tf.getRotation().y();
        pose.orientation.z = tf.getRotation().z();
        pose.orientation.w = tf.getRotation().w();
        poses.poses.push_back(pose);
    }
    particles_pub.publish(poses);
}

void AMCL::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    curr_odom = *msg;
}

void AMCL::update_particle_pose_by_odom(void)
{
    // update robot and particle pose based on odometry
    static double prev_time;
    nav_msgs::Odometry odom = curr_odom;
    double curr_time = odom.header.stamp.toSec();
    if (is_first_time)
    {
        prev_time = curr_time;
        is_first_time = false;
        return;
    }
    double d_time = curr_time - prev_time;
    if (d_time > (1.0 / pose_publish_hz) * 4.0)
    {
        ROS_ERROR("update rate based on odometry is too late");
        prev_time = curr_time;
        return;
    }
    if (!use_omni_odom)
    {
        double d_dist = odom.twist.twist.linear.x * d_time;
        double d_yaw = odom.twist.twist.angular.z * d_time;
        double d_dist2 = d_dist * d_dist;
        double d_yaw2 = d_yaw * d_yaw;
        delta_dist += d_dist;
        delta_yaw += d_yaw;
        robot_pose.x += d_dist * cos(robot_pose.yaw);
        robot_pose.y += d_dist * sin(robot_pose.yaw);
        robot_pose.yaw += d_yaw;
        while (robot_pose.yaw < -M_PI)
            robot_pose.yaw += 2.0 * M_PI;
        while (robot_pose.yaw > M_PI)
            robot_pose.yaw -= 2.0 * M_PI;
        for (int i = 0; i < particle_num; i++)
        {
            double dd = d_dist + nrand(d_dist2 * odom_noise_dist_dist + d_yaw2 * odom_noise_head_dist);
            double dy = d_yaw + nrand(d_dist2 * odom_noise_dist_head + d_yaw2 * odom_noise_head_head);
            particles[i].pose.x += dd * cos(particles[i].pose.yaw);
            particles[i].pose.y += dd * sin(particles[i].pose.yaw);
            particles[i].pose.yaw += dy;
            while (particles[i].pose.yaw < -M_PI)
                particles[i].pose.yaw += 2.0 * M_PI;
            while (particles[i].pose.yaw > M_PI)
                particles[i].pose.yaw -= 2.0 * M_PI;
        }
    }
    else
    {
        double d_dist_x = odom.twist.twist.linear.x * d_time;
        double d_dist_y = odom.twist.twist.linear.y * d_time;
        double d_yaw = odom.twist.twist.angular.z * d_time;
        double d_dist_x2 = d_dist_x * d_dist_x;
        double d_dist_y2 = d_dist_y * d_dist_y;
        double d_yaw2 = d_yaw * d_yaw;
        delta_dist += sqrt(d_dist_x2 + d_dist_y2);
        delta_yaw += d_yaw;
        robot_pose.x += d_dist_x * cos(robot_pose.yaw) + d_dist_y * sin(robot_pose.yaw);
        robot_pose.y += d_dist_x * sin(robot_pose.yaw) + d_dist_y * cos(robot_pose.yaw);
        robot_pose.yaw += d_yaw;
        while (robot_pose.yaw < -M_PI)
            robot_pose.yaw += 2.0 * M_PI;
        while (robot_pose.yaw > M_PI)
            robot_pose.yaw -= 2.0 * M_PI;
        for (int i = 0; i < particle_num; i++)
        {
            double ddx = d_dist_x + nrand(d_dist_x2 * odom_noise_dist_dist + d_dist_y2 * odom_noise_dist_dist + d_yaw2 * odom_noise_head_dist);
            double ddy = d_dist_y + nrand(d_dist_x2 * odom_noise_dist_dist + d_dist_y2 * odom_noise_dist_dist + d_yaw2 * odom_noise_head_dist);
            double dy = d_yaw + nrand(d_dist_x2 * odom_noise_dist_head + d_dist_y2 * odom_noise_dist_head + d_yaw2 * odom_noise_head_head);
            particles[i].pose.x += ddx * cos(particles[i].pose.yaw) + ddy * sin(particles[i].pose.yaw);
            particles[i].pose.y += ddx * sin(particles[i].pose.yaw) + ddy * cos(particles[i].pose.yaw);
            particles[i].pose.yaw += dy;
            while (particles[i].pose.yaw < -M_PI)
                particles[i].pose.yaw += 2.0 * M_PI;
            while (particles[i].pose.yaw > M_PI)
                particles[i].pose.yaw -= 2.0 * M_PI;
        }
    }
    prev_time = curr_time;
}

void AMCL::check_scan_points_validity(sensor_msgs::LaserScan scan)
{
    sensor_msgs::LaserScan uscan;
    sensor_msgs::PointCloud upoints;
    uscan = scan;
    upoints.header = scan.header;
    for (int i = 0; i < (int)scan.ranges.size(); i++)
    {
        double r = scan.ranges[i];
        if (r < scan.range_min || scan.range_max < r)
        {
            is_valid_scan_points[i] = false;
        }
        else
        {
            is_valid_scan_points[i] = true;
            if (i % scan_step == 0)
            {
                double angle = scan.angle_min + scan.angle_increment * (double)i;
                geometry_msgs::Point32 point;
                point.x = scan.ranges[i] * cos(angle);
                point.y = scan.ranges[i] * sin(angle);
                point.z = 0.0;
                upoints.points.push_back(point);
            }
            else
            {
                uscan.ranges[i] = scan.range_min;
            }
        }
    }
    uscan_pub.publish(uscan);
    upoints_pub.publish(upoints);
    if (use_test_range_measurement && is_map_data)
    {
        std::vector<double> p, q;
        p.resize(scan.ranges.size(), 0.0);
        q.resize(scan.ranges.size(), 0.0);
        double z_hit_ = (1.0 - z_short - z_max - z_rand);
        for (int i = 0; i < particle_num; i++)
        {
            double c = cos(particles[i].pose.yaw);
            double s = sin(particles[i].pose.yaw);
            double xo = base_link2laser.x * c - base_link2laser.y * s + particles[i].pose.x;
            double yo = base_link2laser.x * s + base_link2laser.y * c + particles[i].pose.y;
            for (int j = 0; j < (int)scan.ranges.size(); j += scan_step)
            {
                if (is_valid_scan_points[j])
                {
                    double angle = scan.angle_min + scan.angle_increment * (double)j;
                    double yaw = angle + particles[i].pose.yaw + base_link2laser.yaw;
                    double dx = map.info.resolution * cos(yaw);
                    double dy = map.info.resolution * sin(yaw);
                    double x = xo;
                    double y = yo;
                    double range = -1.0;
                    for (double r = 0.0; r <= scan.range_max; r += map.info.resolution)
                    {
                        int u, v;
                        xy2uv(x, y, &u, &v);
                        if (0 <= u && u < map.info.width && 0 <= v && v < map.info.height)
                        {
                            int node = v * map.info.width + u;
                            if (map.data[node] == 100)
                            {
                                range = r;
                                break;
                            }
                        }
                        else
                        {
                            break;
                        }
                        x += dx;
                        y += dy;
                    }
                    double p1 = z_max * max_dist_prob + z_rand * z_rand_mult;
                    double r = scan.ranges[j];
                    double p2 = 0.0, p3 = 0.0;
                    double p2_ = 0.0;
                    if (r <= range)
                    {
                        p2_ = (1.0 / (1.0 - exp(-lambda_short * range))) * lambda_short * exp(-lambda_short * r);
                        p2 = z_short * p2_;
                        if (p2 > 1.0)
                            p2 = 1.0;
                        if (p2_ > 1.0)
                            p2_ = 1.0;
                    }
                    x = r * cos(yaw) + xo;
                    y = r * sin(yaw) + yo;
                    int u, v;
                    xy2uv(x, y, &u, &v);
                    if (0 <= u && u < map.info.width && 0 <= v && v < map.info.height)
                    {
//                        double z = (double)dist_map[u][v];
                        double z = (double)dist_map.at<float>(v, u);
                        p3 = z_hit_ * norm_const_hit * exp(-(z * z) / (2.0 * z_hit_var));
                    }
                    double pz = p1 + p2 + p3;
                    if (pz > 1.0)
                        pz = 1.0;
                    if (range < 0.0)
                    {
                        p2 = 1.0;
                        p2_ = 1.0;
                        pz = 0.0;
                    }
                    p[j] += p2_;
                    q[j] += pz;
                }
                else
                {
                    double pz = z_max * max_dist_prob + z_rand * z_rand_mult;
                    q[j] += pz;
                }
            }
        }
        sensor_msgs::LaserScan dscan;
        sensor_msgs::PointCloud dpoints;
        dscan = scan;
        dpoints.header = scan.header;
        for (int i = 0; i < (int)scan.ranges.size(); i += scan_step)
        {
            double w = p[i] / q[i];
            dscan.intensities[i] = w;
            if (w > dynamic_scan_point_threshold && is_valid_scan_points[i])
            {
                double angle = scan.angle_min + scan.angle_increment * (double)i;
                geometry_msgs::Point32 point;
                point.x = scan.ranges[i] * cos(angle);
                point.y = scan.ranges[i] * sin(angle);
                point.z = 0.0;
                dpoints.points.push_back(point);
                is_valid_scan_points[i] = false;
            }
            else
            {
                dscan.ranges[i] = scan.range_min;
                dscan.intensities[i] = 0.0;
            }
        }
        dscan_pub.publish(dscan);
        dpoints_pub.publish(dpoints);
    }
}

double AMCL::compute_weight_using_likelihood_field_model(pose_t pose, sensor_msgs::LaserScan scan)
{
    double w = 0.0;
    double c = cos(pose.yaw);
    double s = sin(pose.yaw);
    double xo = base_link2laser.x * c - base_link2laser.y * s + pose.x;
    double yo = base_link2laser.x * s + base_link2laser.y * c + pose.y;
    for (int i = 0; i < (int)scan.ranges.size(); i += scan_step)
    {
        double pz;
        if (!is_valid_scan_points[i])
        {
            pz = z_max * 1.0 * map.info.resolution + z_rand * z_rand_mult;
        }
        else
        {
            double angle = scan.angle_min + scan.angle_increment * (double)i;
            double yaw = angle + pose.yaw + base_link2laser.yaw;
            double r = scan.ranges[i];
            double x = r * cos(yaw) + xo;
            double y = r * sin(yaw) + yo;
            int u, v;
            xy2uv(x, y, &u, &v);
            if (0 <= u && u < map.info.width && 0 <= v && v < map.info.height)
            {
//                double z = (double)dist_map[u][v];
                double z = dist_map.at<float>(v, u);
                pz = z_hit * norm_const_hit * exp(-(z * z) / (2.0 * z_hit_var)) * map.info.resolution + z_rand * z_rand_mult;
                pz += z_max * 1.0 * map.info.resolution;
            }
            else
            {
                pz = z_max * 1.0 * map.info.resolution + z_rand * z_rand_mult;
            }
        }
        w += log(pz);
    }
    return exp(w);
}

void AMCL::evaluate_particles_using_likelihood_field_model(sensor_msgs::LaserScan scan)
{
    double max;
    for (int i = 0; i < particle_num; i++)
    {
        double weight = compute_weight_using_likelihood_field_model(particles[i].pose, scan);
        particles[i].w *= weight;
        if (i == 0)
        {
            max_particle_likelihood_num = i;
            max = particles[i].w;
        }
        else
        {
            if (max < particles[i].w)
            {
                max_particle_likelihood_num = i;
                max = particles[i].w;
            }
        }
    }
}

double AMCL::compute_weight_using_beam_model(pose_t pose, sensor_msgs::LaserScan scan)
{
    double w = 0.0;
    double z_hit_ = (1.0 - z_short - z_max - z_rand);
    double c = cos(pose.yaw);
    double s = sin(pose.yaw);
    double xo = base_link2laser.x * c - base_link2laser.y * s + pose.x;
    double yo = base_link2laser.x * s + base_link2laser.y * c + pose.y;
    for (int i = 0; i < (int)scan.ranges.size(); i += scan_step)
    {
        double pz;
        if (is_valid_scan_points[i])
        {
            double angle = scan.angle_min + scan.angle_increment * (double)i;
            double yaw = angle + pose.yaw + base_link2laser.yaw;
            double dx = map.info.resolution * cos(yaw);
            double dy = map.info.resolution * sin(yaw);
            double x = xo;
            double y = yo;
            double range = -1.0;
            for (double r = 0.0; r <= scan.range_max; r += map.info.resolution)
            {
                int u, v;
                xy2uv(x, y, &u, &v);
                if (0 <= u && u < (int)map.info.width && 0 <= v && v < (int)map.info.height)
                {
                    int node = v * map.info.width + u;
                    if ((int)map.data[node] == 100)
                    {
                        range = r;
                        break;
                    }
                }
                else
                {
                    break;
                }
                x += dx;
                y += dy;
            }
            if (range < 0.0)
            {
                pz = z_max * 1.0 * map.info.resolution + z_rand * z_rand_mult;
            }
            else
            {
                double r = scan.ranges[i];
                pz = z_max * 1.0 * map.info.resolution;
                if (0.0 <= r && r <= range)
                    pz += z_short * (1.0 / (1.0 - exp(-lambda_short * range))) * lambda_short * exp(-lambda_short * r) * map.info.resolution;
                double dr = r - range;
                pz += z_hit_ * norm_const_hit * exp(-(dr * dr) / (2.0 * z_hit_var)) * map.info.resolution;
            }
        }
        else
        {
            pz = z_max * 1.0 * map.info.resolution + z_rand * z_rand_mult;
        }
        w += log(pz);
    }
    return exp(w);
}

void AMCL::evaluate_particles_using_beam_model(sensor_msgs::LaserScan scan)
{
    double max;
    for (int i = 0; i < particle_num; i++)
    {
        double weight = compute_weight_using_beam_model(particles[i].pose, scan);
        particles[i].w *= weight;
        if (i == 0)
        {
            max_particle_likelihood_num = i;
            max = particles[i].w;
        }
        else
        {
            if (max < particles[i].w)
            {
                max_particle_likelihood_num = i;
                max = particles[i].w;
            }
        }
    }
}

double AMCL::compute_weight_with_class_conditional_observation_model(pose_t pose, sensor_msgs::LaserScan scan, bool use_all_scan)
{
    double w = 0.0;
    double c = cos(pose.yaw);
    double s = sin(pose.yaw);
    double xo = base_link2laser.x * c - base_link2laser.y * s + pose.x;
    double yo = base_link2laser.x * s + base_link2laser.y * c + pose.y;
    double p_known_prior = 0.5;
    double p_unknown_prior = 1.0 - p_known_prior;
    int step = scan_step;
    if (use_all_scan)
        step = 1;
    for (int i = 0; i < scan.ranges.size(); i += step)
    {
        double pz_known, pz_unknown;
        if (!is_valid_scan_points[i])
        {
            pz_known = z_max * 1.0 * map.info.resolution + z_rand * z_rand_mult;
            pz_unknown = 1.0 / (1.0 - exp(-lambda_short * scan.range_max)) * lambda_short * exp(-lambda_short * scan.range_max) * map.info.resolution;
        }
        else
        {
            double angle = scan.angle_min + scan.angle_increment * (double)i;
            double yaw = angle + pose.yaw;
            double r = scan.ranges[i];
            double x = r * cos(yaw) + xo;
            double y = r * sin(yaw) + yo;
            int u, v;
            xy2uv(x, y, &u, &v);
            if (0 <= u && u < map.info.width && 0 <= v && v < map.info.height)
            {
//                double z = dist_map[u][v];
                double z = dist_map.at<float>(v, u);
                pz_known = z_hit * norm_const_hit * exp(-(z * z) / (2.0 * z_hit_var)) * map.info.resolution + z_rand * z_rand_mult;
                pz_unknown = 1.0 / (1.0 - exp(-lambda_short * scan.range_max)) * lambda_short * exp(-lambda_short * r) * map.info.resolution;
            }
            else
            {
                pz_known = z_max * 1.0 * map.info.resolution + z_rand * z_rand_mult;
                pz_unknown = 1.0 / (1.0 - exp(-lambda_short * scan.range_max)) * lambda_short * exp(-lambda_short * scan.range_max) * map.info.resolution;
            }
        }
        double p_known = pz_known * p_known_prior;
        double p_unknown = pz_unknown * p_unknown_prior;
        double sum = p_known + p_unknown;
        if (use_all_scan)
            unknown_probs[i] = p_unknown / sum;
        if (i % scan_step == 0)
            w += log(sum);
    }
    return exp(w);
}

// consider two classes; known and unknown
void AMCL::evaluate_particles_with_class_conditional_observation_model(sensor_msgs::LaserScan scan)
{
    double max;
    for (int i = 0; i < particle_num; i++)
    {
        // partial scan points are used to calculate likelihood in order to accelerate computation time
        double weight = compute_weight_with_class_conditional_observation_model(particles[i].pose, scan, false);
        particles[i].w *= weight;
        if (i == 0)
        {
            max_particle_likelihood_num = i;
            max = particles[i].w;
        }
        else
        {
            if (max < particles[i].w)
            {
                max_particle_likelihood_num = i;
                max = particles[i].w;
            }
        }
    }
    // compute dynamic scan point probability again using all scan points with the maximum likelihood particle
    compute_weight_with_class_conditional_observation_model(particles[max_particle_likelihood_num].pose, scan, true);
    // dynamic scan points detection
    sensor_msgs::LaserScan dscan;
    sensor_msgs::PointCloud dpoints;
    dscan = scan;
    dpoints.header = scan.header;
    if (dscan.ranges.size() != dscan.intensities.size())
        dscan.intensities.resize((int)dscan.ranges.size());
    for (int i = 0; i < scan.ranges.size(); i++)
    {
        dscan.intensities[i] = unknown_probs[i];
        if (is_valid_scan_points[i] && unknown_probs[i] >= dynamic_scan_point_threshold)
        {
            geometry_msgs::Point32 p;
            double r = scan.ranges[i];
            double yaw = scan.angle_min + scan.angle_increment * (double)i;
            p.x = r * cos(yaw);
            p.y = r * sin(yaw);
            p.z = 0.0;
            dpoints.points.push_back(p);
        }
        else
        {
            dscan.ranges[i] = scan.range_min - 0.1;
            dscan.intensities[i] = 0.0;
        }
    }
    dscan_pub.publish(dscan);
    dpoints_pub.publish(dpoints);
}

void AMCL::compute_total_weight_and_effective_sample_size(void)
{
    double wo = 1.0 / (double)particle_num;
    total_weight = 0.0;
    for (int i = 0; i < particle_num; i++)
        total_weight += particles[i].w;
    double sum = 0.0;
    for (int i = 0; i < particle_num; i++)
    {
        double w = particles[i].w / total_weight;
        if (std::isnan(w) != 0)
            w = wo;
        particles[i].w = w;
        sum += w * w;
    }
    effective_sample_size = 1.0 / sum;
}

void AMCL::compute_random_particle_rate(void)
{
    w_avg = total_weight / (double)particle_num;
    if (w_slow == 0.0)
        w_slow = w_avg;
    else
        w_slow = w_slow + alpha_slow * (w_avg - w_slow);
    if (w_fast == 0.0)
        w_fast = w_avg;
    else
        w_fast = w_fast + alpha_fast * (w_avg - w_fast);
    double r = 1.0 - w_fast / w_slow;
    // reset weight history to avoid spiraling off into complete randomness
    if (r > 0.0)
        w_fast = w_slow = 0.0;
    random_particle_rate = r;
}

void AMCL::estimate_robot_pose(void)
{
    double tmp_yaw = robot_pose.yaw;
    pose_t pose;
    pose.x = pose.y = pose.yaw = 0.0;
    for (int i = 0; i < particle_num; i++)
    {
        pose.x += particles[i].pose.x * particles[i].w;
        pose.y += particles[i].pose.y * particles[i].w;
        double dyaw = tmp_yaw - particles[i].pose.yaw;
        while (dyaw < -M_PI)
            dyaw += 2.0 * M_PI;
        while (dyaw > M_PI)
            dyaw -= 2.0 * M_PI;
        pose.yaw += dyaw * particles[i].w;
    }
    pose.yaw = tmp_yaw - pose.yaw;
    while (pose.yaw < -M_PI)
        pose.yaw += 2.0 * M_PI;
    while (pose.yaw > M_PI)
        pose.yaw -= 2.0 * M_PI;
    robot_pose = pose;
}

void AMCL::resample_particles(void)
{
    static FILE* fp;
    if (fp == NULL)
        fp = fopen("/tmp/estimated_trajectory.txt", "w");
    fprintf(fp, "%lf %lf %lf\n", robot_pose.x, robot_pose.y, robot_pose.yaw);
    fflush(fp);
    if (effective_sample_size > (double)particle_num * resample_threshold)
        return;
    double darts, wo, wb[particle_num];
    wo = 1.0 / (double)particle_num;
    wb[0] = particles[0].w;
    for (int i = 1; i < particle_num; i++)
        wb[i] = particles[i].w + wb[i - 1];
    std::vector<particle_t> tmp_particles = particles;
    if (use_kld_sampling)
    {
        double pose_reso = 0.1, angle_reso = 0.2 * M_PI / 180.0;
        double pose_range = 2.0, angle_range = 5.0 * M_PI / 180.0;
        int pose_size = (int)(pose_range / pose_reso);
        int angle_size = (int)(angle_range / angle_reso);
        std::vector<bool> is_empty;
        is_empty.resize(pose_size * pose_size * angle_size, true);
        int k = 0, m = 0;
        double m_chi = 0.0, epsilon = 0.05, z = 0.83891;
        // determined value of z from this pdf -> http://math.arizona.edu/~rsims/ma464/standardnormaltable.pdf
        do
        {
            darts = (double)rand() / ((double)RAND_MAX + 1.0);
            for (int i = 0; i < particle_num; i++)
            {
                if (darts < wb[i])
                {
                    particles[m] = tmp_particles[i];
                    particles[m].w = wo;
                    double dx = particles[m].pose.x - robot_pose.x;
                    double dy = particles[m].pose.y - robot_pose.y;
                    double dyaw = particles[m].pose.yaw - robot_pose.yaw;
                    if (dyaw < -M_PI)
                        dyaw += 2.0 * M_PI;
                    if (dyaw > M_PI)
                        dyaw -= 2.0 * M_PI;
                    int u = (int)(dx / pose_reso) + pose_size / 2;
                    int v = (int)(dy / pose_reso) + pose_size / 2;
                    int w = (int)(dyaw / angle_reso) + angle_size / 2;
                    int node = w * pose_size * pose_size + v * pose_size + u;
                    bool fall_empty = true;
                    if (0 <= u && u < pose_size && 0 <= v && v < pose_size && 0 <= w && w < angle_size)
                    {
                        if (!is_empty[node])
                            fall_empty = false;
                    }
                    if (fall_empty)
                    {
                        k++;
                        if (0 <= u && u < pose_size && 0 <= v && v < pose_size && 0 <= w && w < angle_size)
                            is_empty[node] = false;
                        if (k > 1)
                        {
                            double m1 = (double)(k - 1) / (2.0 * epsilon);
                            double m2 = 1.0 - 2.0 / (9.0 * (double)(k - 1)) + sqrt(2.0 / (9.0 * (double)(k - 1))) * z;
                            m_chi = m1 * pow(m2, 3.0);
                        }
                    }
                    m++;
                    i = particle_num;
                }
            }
            if (m >= max_particle_num)
                break;
        }
        while (m < (int)m_chi || m < min_particle_num);
        particle_num = m;
    }
    else if (add_random_particle)
    {
        // number of particles without random particle rate will be generated first
        particle_num = (int)((double)max_particle_num / (1.0 + add_random_particle_rate)) + 1;
        for (int i = 0; i < particle_num; i++)
        {
            darts = (double)rand() / ((double)RAND_MAX + 1.0);
            for (int j = 0; j < max_particle_num; j++)
            {
                if (darts < wb[j])
                {
                    particles[i] = tmp_particles[j];
                    particles[i].w = wo;
                    break;
                }
            }
        }
        // random particle will be generated
        for (int i = particle_num; i < max_particle_num; i++)
        {
            particles[i].pose.x = robot_pose.x + nrand(initial_cov_xx * 0.5);
            particles[i].pose.y = robot_pose.y + nrand(initial_cov_yy * 0.5);
            particles[i].pose.yaw = robot_pose.yaw + nrand(initial_cov_yawyaw * 1.0);
            while (particles[i].pose.yaw < -M_PI)
                particles[i].pose.yaw += 2.0 * M_PI;
            while (particles[i].pose.yaw > M_PI)
                particles[i].pose.yaw -= 2.0 * M_PI;
            particles[i].w = wo;
        }
        // particle_num is overwritten to calculate likelihood of all of the particles
        particle_num = max_particle_num;
    }
    else
    {
        for (int i = 0; i < particle_num; i++)
        {
            darts = (double)rand() / ((double)RAND_MAX + 1.0);
            for (int j = 0; j < particle_num; j++)
            {
                if (darts < wb[j])
                {
                    particles[i] = tmp_particles[j];
                    particles[i].w = wo;
                    break;
                }
            }
        }
    }
}

void AMCL::reset_moving_amounts(void)
{
    delta_dist = 0.0;
    delta_yaw = 0.0;
}

void AMCL::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    curr_scan = *msg;
    if (!is_scan_data)
    {
        is_valid_scan_points.resize((int)msg->ranges.size());
        if (use_class_conditional_observation_model)
        {
            unknown_probs.resize((int)msg->ranges.size());
            measurement_classes.resize((int)msg->ranges.size());
            for (int i = 0; i < (int)msg->ranges.size(); i++)
                measurement_classes[i].resize(3);
            printf("size of unknown_probs is = %d\n", (int)unknown_probs.size());
        }
        is_scan_data = true;
        printf("got a first scan\n");
    }
}

void AMCL::plot_likelihood_distribution(pose_t pose, sensor_msgs::LaserScan scan)
{
    double dr = 0.6;
    double reso = 0.1;
    static FILE* gp;
    if (gp == NULL)
    {
        gp = popen("/usr/bin/gnuplot", "w");
        fprintf(gp, "unset key\n");
        fprintf(gp, "set tics font 'Arial, %d'\n", 11);
        fprintf(gp, "set xlabel font 'Arial, %d'\n", 11);
        fprintf(gp, "set ylabel font 'Arial, %d'\n", 11);
        fprintf(gp, "set key font 'Arial, %d'\n", 11);
        fprintf(gp, "set grid\n");
        fprintf(gp, "set xrange [%f:%f]\n", -(dr - reso), dr - reso);
        fprintf(gp, "set yrange [%f:%f]\n", -(dr - reso), dr - reso);
//        fprintf(gp, "set xlabel '{/Symbol D}x [m]'\n");
//        fprintf(gp, "set ylabel '{/Symbol D}y [m]'\n");
        fprintf(gp, "set xlabel 'dx [m]'\n");
        fprintf(gp, "set ylabel 'dy [m]'\n");
        fprintf(gp, "set size ratio 1 1\n");
        fprintf(gp, "set pm3d map interpolate 10, 10\n");
//        fprintf(gp, "set palette rgbformulae 22, 13, -31\n");
        fprintf(gp, "set palette rgbformulae 21, 22, 23\n");
        fprintf(gp, "set cbrange [0.0:1.0]\n");
    }

    std::vector<double> likelihoods;
    double max = 0.0;
    for (double x = pose.x - dr; x <= pose.x + dr; x += reso)
    {
        for (double y = pose.y - dr; y <= pose.y + dr; y += reso)
        {
            pose_t tmp_pose = pose;
            tmp_pose.x = x;
            tmp_pose.y = y;
            double w;
            if (use_class_conditional_observation_model)
                w = compute_weight_with_class_conditional_observation_model(tmp_pose, scan, false);
            else if (use_beam_model)
                w = compute_weight_using_beam_model(tmp_pose, scan);
            else
                w = compute_weight_using_likelihood_field_model(tmp_pose, scan);
            likelihoods.push_back(w);
            if (max < w)
                max = w;
        }
    }
    FILE* fp = fopen("/tmp/likelihood_distribution.txt", "w");
    int i = 0;
    for (double x = pose.x - dr; x <= pose.x + dr; x += reso)
    {
        for (double y = pose.y - dr; y <= pose.y + dr; y += reso)
        {
            double dx = x - pose.x;
            double dy = y - pose.y;
            fprintf(fp, "%lf %lf %lf\n", dx, dy, likelihoods[i] / max);
            i++;
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
    fprintf(gp, "splot '/tmp/likelihood_distribution.txt' u 1:2:3 t ''\n");
    fflush(gp);
}

void AMCL::publish_likelihood_distribution_map_vis(double range, double reso, sensor_msgs::LaserScan scan)
{
    grid_map::GridMap map_vis({"likelihood"});
    map_vis.setFrameId(map_frame);
    map_vis.setGeometry(grid_map::Length(2.0 * range, 2.0 * range), reso, grid_map::Position(robot_pose.x, robot_pose.y));
    bool is_first = true;
    double max;
    for (double x = robot_pose.x - 1.2 * range; x <= robot_pose.x + 1.2 * range; x += 0.8 * reso)
    {
        for (double y = robot_pose.y - 1.2 * range; y <= robot_pose.y + 1.2 * range; y += 0.8 * reso)
        {
            pose_t pose;
            pose.x = x;
            pose.y = y;
            pose.yaw = robot_pose.yaw;
            double weight;
            if (use_beam_model)
                weight = compute_weight_using_beam_model(pose, scan);
            else
                weight = compute_weight_using_likelihood_field_model(pose, scan);
            if (is_first)
            {
                max = weight;
                is_first = false;
            }
            else
            {
                if (max < weight)
                    max = weight;
            }
        }
    }
    for (double x = robot_pose.x - 1.2 * range; x <= robot_pose.x + 1.2 * range; x += 0.8 * reso)
    {
        for (double y = robot_pose.y - 1.2 * range; y <= robot_pose.y + 1.2 * range; y += 0.8 * reso)
        {
            pose_t pose;
            pose.x = x;
            pose.y = y;
            pose.yaw = robot_pose.yaw;
            double weight;
            if (use_beam_model)
                weight = compute_weight_using_beam_model(pose, scan);
            else
                weight = compute_weight_using_likelihood_field_model(pose, scan);
            grid_map::Position p;
            p(0) = x;
            p(1) = y;
            grid_map::Index index;
            if (!map_vis.getIndex(p, index))
                continue;
            map_vis.atPosition("likelihood", p) = weight / max;
        }
    }
    grid_map_msgs::GridMap map_vis_msg;
    grid_map::GridMapRosConverter::toMessage(map_vis, map_vis_msg);
    likelihood_dist_map_pub.publish(map_vis_msg);
}

void AMCL::compute_scan_fractions(pose_t pose, sensor_msgs::LaserScan scan, double* valid_scan, double* matched_scan)
{
    static FILE* fp;
    if (fp == NULL)
        fp = fopen("/tmp/scan_fractions.txt", "w");
    int v_cnt = 0, m_cnt = 0;
    double c = cos(pose.yaw);
    double s = sin(pose.yaw);
    double xo = base_link2laser.x * c - base_link2laser.y * s + pose.x;
    double yo = base_link2laser.x * s + base_link2laser.y * c + pose.y;
    for (int i = 0; i < (int)scan.ranges.size(); i++)
    {
        double r = scan.ranges[i];
        if (scan.range_min <= r && r <= scan.range_max)
        {
            v_cnt++;
            double angle = scan.angle_min + scan.angle_increment * (double)i;
            double yaw = angle + pose.yaw + base_link2laser.yaw;
            double x = r * cos(yaw) + xo;
            double y = r * sin(yaw) + yo;
            int u, v;
            xy2uv(x, y, &u, &v);
            if (0 <= u && u < map.info.width && 0 <= v && v < map.info.height)
            {
//                double z = (double)dist_map[u][v];
                double z = dist_map.at<float>(v, u);
                if (z < 0.2)
                    m_cnt++;
            }
        }
    }
    double vs = v_cnt / (double)scan.ranges.size();
    double ms = m_cnt / (double)scan.ranges.size();
    if (valid_scan != NULL)
        *valid_scan = vs;
    if (matched_scan != NULL)
        *matched_scan = ms;
    fprintf(fp, "%lf %lf %lf\n", scan.header.stamp.toSec(), vs, ms);
    fflush(fp);
}

void AMCL::publish_residual_errors_as_laser_scan(pose_t pose, sensor_msgs::LaserScan scan)
{
    double c = cos(pose.yaw);
    double s = sin(pose.yaw);
    double xo = base_link2laser.x * c - base_link2laser.y * s + pose.x;
    double yo = base_link2laser.x * s + base_link2laser.y * c + pose.y;
    sensor_msgs::LaserScan errors = scan;
    for (int i = 0; i < (int)scan.ranges.size(); i++)
    {
        if (!is_valid_scan_points[i])
        {
            errors.ranges[i] = scan.range_max;
            errors.intensities[i] = 0.0;
            continue;
        }
        double angle = scan.angle_min + scan.angle_increment * (double)i;
        double yaw = angle + pose.yaw + base_link2laser.yaw;
        double r = scan.ranges[i];
        double x = r * cos(yaw) + xo;
        double y = r * sin(yaw) + yo;
        int u, v;
        xy2uv(x, y, &u, &v);
        if (0 <= u && u < map.info.width && 0 <= v && v < map.info.height)
        {
//            errors.ranges[i] = ((double)dist_map[u][v]);
            errors.ranges[i] = ((double)dist_map.at<float>(v, u));
            errors.intensities[i] = 0.0;
        }
        else
        {
            errors.ranges[i] = scan.range_max;
            errors.intensities[i] = 0.0;
        }
    }
    matching_error_pub.publish(errors);
}

std::vector<double> AMCL::get_residual_errors_as_std_vector(pose_t pose, sensor_msgs::LaserScan scan, int scan_step)
{
    double c = cos(pose.yaw);
    double s = sin(pose.yaw);
    double xo = base_link2laser.x * c - base_link2laser.y * s + pose.x;
    double yo = base_link2laser.x * s + base_link2laser.y * c + pose.y;
    std::vector<double> residual_errors;
    for (int i = 0; i < (int)scan.ranges.size(); i += scan_step)
    {
        if (!is_valid_scan_points[i])
        {
            residual_errors.push_back(-1.0);
            continue;
        }
        double angle = scan.angle_min + scan.angle_increment * (double)i;
        double yaw = angle + pose.yaw + base_link2laser.yaw;
        double r = scan.ranges[i];
        double x = r * cos(yaw) + xo;
        double y = r * sin(yaw) + yo;
        int u, v;
        xy2uv(x, y, &u, &v);
        if (0 <= u && u < map.info.width && 0 <= v && v < map.info.height)
//            residual_errors.push_back((double)dist_map[u][v]);
            residual_errors.push_back((double)dist_map.at<float>(v, u));
        else
            residual_errors.push_back(-1.0);
    }
    return residual_errors;
}

void AMCL::publish_expected_map_distances_as_laser_scan(pose_t pose)
{
    double c = cos(pose.yaw);
    double s = sin(pose.yaw);
    double xo = base_link2laser.x * c - base_link2laser.y * s + pose.x;
    double yo = base_link2laser.x * s + base_link2laser.y * c + pose.y;
    sensor_msgs::LaserScan expected_distances = curr_scan;
    for (int i = 0; i < (int)expected_distances.ranges.size(); i++)
    {
        if (is_valid_scan_points[i])
        {
            double angle = expected_distances.angle_min + expected_distances.angle_increment * (double)i;
            double yaw = angle + pose.yaw + base_link2laser.yaw;
            double dx = map.info.resolution * cos(yaw);
            double dy = map.info.resolution * sin(yaw);
            double x = xo;
            double y = yo;
            double range = 0.0;
            for (double r = 0.0; r <= expected_distances.range_max; r += map.info.resolution)
            {
                int u, v;
                xy2uv(x, y, &u, &v);
                if (0 <= u && u < (int)map.info.width && 0 <= v && v < (int)map.info.height)
                {
                    int node = v * map.info.width + u;
                    if ((int)map.data[node] == 100)
                    {
                        range = r;
                        break;
                    }
                }
                else
                {
                    break;
                }
                x += dx;
                y += dy;
            }
            expected_distances.ranges[i] = range;
        }
        else
        {
            expected_distances.ranges[i] = 0.0;
        }
    }
    expected_distances_pub.publish(expected_distances);
}
