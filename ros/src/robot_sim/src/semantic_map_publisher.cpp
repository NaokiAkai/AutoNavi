// Copyright © 2018 Naoki Akai. All rights reserved.
// This program was written by referring to map_server that is a ROS's default program for reading yaml and pgm files of OGM

#include <libgen.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <SDL/SDL_image.h>
#include <robot_sim/common.h>

class SemanticMapPublisher
{
private:
    ros::NodeHandle nh;
    std::string output_map_topic_name, output_meta_data_topic_name;
    std::string wall_map_yaml_name, door_map_yaml_name, table_map_yaml_name;
    ros::Publisher map_pub, meta_data_pub;
    int map_publish_interval;

public:
    SemanticMapPublisher();
    ~SemanticMapPublisher() {};
    bool read_map_data(nav_msgs::OccupancyGrid* map, nav_msgs::MapMetaData* meta_data,
        std::string fname, std::string frame_id);
    bool load_map_image(nav_msgs::OccupancyGrid* map, std::string fname, std::string frame_id,
        double reso, bool negate, double occ_th, double free_th, std::vector<double> origin, bool trinary);
    void publish(void);
    void spin();
};

SemanticMapPublisher::SemanticMapPublisher():
    nh("~"),
    output_map_topic_name("/laser_sim_semantic_map_source"),
    output_meta_data_topic_name("/laser_sim_semantic_map_source_metadata"),
    wall_map_yaml_name("/tmp/wall.yaml"),
    door_map_yaml_name("/tmp/door.yaml"),
    table_map_yaml_name("/tmp/table.yaml"),
    map_publish_interval(1)
{
    // read parameters
    nh.param("output_map_topic_name", output_map_topic_name, output_map_topic_name);
    nh.param("output_meta_data_topic_name", output_meta_data_topic_name, output_meta_data_topic_name);
    nh.param("wall_map_yaml_name", wall_map_yaml_name, wall_map_yaml_name);
    nh.param("door_map_yaml_name", door_map_yaml_name, door_map_yaml_name);
    nh.param("table_map_yaml_name", table_map_yaml_name, table_map_yaml_name);
    nh.param("map_publish_interval", map_publish_interval, map_publish_interval);
    // publisher
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>(output_map_topic_name, 1, true);
    meta_data_pub = nh.advertise<nav_msgs::MapMetaData>(output_meta_data_topic_name, 1, true);
}

bool SemanticMapPublisher::read_map_data(nav_msgs::OccupancyGrid* map, nav_msgs::MapMetaData* meta_data,
    std::string fname, std::string frame_id)
{
    // read map parameters from yaml file
    std::string mapfname = "";
    double reso, occ_th, free_th;
    std::vector<double> origin(3);
    bool negate, trinary = true;
    YAML::Node lconf = YAML::LoadFile(fname.c_str());
    if (lconf.IsNull())
    {
        fprintf(stderr, "could not load yaml file -> %s\n", fname.c_str());
        return false;
    }
    mapfname = lconf["image"].as<std::string>();
    if (mapfname.size() == 0)
    {
        ROS_ERROR("The image tag cannot be an empty string.");
        return false;
    }
    if (mapfname[0] != '/')
    {
        char* fname_copy = strdup(fname.c_str());
        mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
        free(fname_copy);
    }
    reso = lconf["resolution"].as<double>();
    negate = lconf["negate"].as<int>();
    occ_th = lconf["occupied_thresh"].as<double>();
    free_th = lconf["free_thresh"].as<double>();
    origin = lconf["origin"].as<std::vector<double> >();
//    trinary = lconf["trinary"].as<bool>(); // NOTE: default yaml files for OGM do not have trinary
//    printf("frame_id = %s, reso = %lf, negate = %d, occ_th = %lf, free_th = %lf, origin[0] = %lf, origin[1] = %lf, origin[2] = %lf\n",
//        frame_id.c_str(), reso, negate, occ_th, free_th, origin[0], origin[1], origin[2]);

    // read map image
    if (!load_map_image(map, mapfname, frame_id, reso, negate, occ_th, free_th, origin, trinary))
    {
        ROS_ERROR("could not load map image");
        return false;
    }
    map->info.map_load_time = ros::Time::now();
    map->header.frame_id = frame_id;
    map->header.stamp = ros::Time::now();
    *meta_data = map->info;
    return true;
}

bool SemanticMapPublisher::load_map_image(nav_msgs::OccupancyGrid* map, std::string fname, std::string frame_id,
    double reso, bool negate, double occ_th, double free_th, std::vector<double> origin, bool trinary)
{
    SDL_Surface* img;
    unsigned char* pixels, * p, value;
    int rowstride, n_channels, avg_channels, alpha, color_sum;
    double occ, color_avg;
    // load image
    img = IMG_Load(fname.c_str());
    if (!img)
    {
        ROS_ERROR("could not map image file -> %s", fname.c_str());
        return false;
    }
    // copy the image data into the map structure
    map->info.width = img->w;
    map->info.height = img->h;
    map->info.resolution = reso;
    map->info.origin.position.x = origin[0];
    map->info.origin.position.y = origin[1];
    map->info.origin.position.z = 0.0;
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, origin[2]);
    map->info.origin.orientation.x = q.x();
    map->info.origin.orientation.y = q.y();
    map->info.origin.orientation.z = q.z();
    map->info.origin.orientation.w = q.w();
    map->data.resize(map->info.width * map->info.height);
    rowstride = img->pitch;
    n_channels = img->format->BytesPerPixel;
    if (trinary || n_channels == 1)
        avg_channels = n_channels;
    else
        avg_channels = n_channels - 1;
    pixels = (unsigned char*)(img->pixels);
    for (unsigned int j = 0; j < map->info.height; j++)
    {
        for (unsigned int i = 0; i < map->info.width; i++)
        {
            p = pixels + j * rowstride + i * n_channels;
            color_sum = 0;
            for (int k = 0; k < avg_channels; k++)
                color_sum += *(p + (k));
            color_avg = color_sum / (double)avg_channels;
            if (n_channels == 1)
                alpha = 1;
            else
                alpha = *(p + n_channels - 1);
            if (negate)
                occ = color_avg / 255.0;
            else
                occ = (255 - color_avg) / 255.0;
            if (occ > occ_th)
                value = 100;
            else if (occ < free_th)
                value = 0;
            else if (trinary || alpha < 1)
                value = -1;
            else
                value = 99 * ((occ - free_th) / (occ_th - free_th));
            map->data[(map->info.height - j - 1) * map->info.width + i] = value;
        }
    }
    SDL_FreeSurface(img);
    return map;
}

void SemanticMapPublisher::publish(void)
{
    sleep(map_publish_interval); // wait for what other modules are executed
    nav_msgs::OccupancyGrid map;
    nav_msgs::MapMetaData meta_data;
    if (read_map_data(&map, &meta_data, wall_map_yaml_name, WALL_MAP_FRAME))
    {
        map_pub.publish(map);
        meta_data_pub.publish(meta_data);
        printf("published wall map from semantic_map_publisher\n");
        sleep(map_publish_interval);
    }
    else
        fprintf(stderr, "wall map could not be published from semantic_map_publisher\n");
    if (read_map_data(&map, &meta_data, door_map_yaml_name, DOOR_MAP_FRAME))
    {
        map_pub.publish(map);
        meta_data_pub.publish(meta_data);
        printf("published door map from semantic_map_publisher\n");
        sleep(map_publish_interval);
    }
    else
        fprintf(stderr, "door map could not be published from semantic_map_publisher\n");
    if (read_map_data(&map, &meta_data, table_map_yaml_name, TABLE_MAP_FRAME))
    {
        map_pub.publish(map);
        meta_data_pub.publish(meta_data);
        printf("published table map from semantic_map_publisher\n");
        sleep(map_publish_interval);
    }
    else
        fprintf(stderr, "table map could not be published from semantic_map_publisher\n");
}

void SemanticMapPublisher::spin(void)
{
    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "semantic_map_publisher");
    SemanticMapPublisher node;
    node.publish();
    node.spin();
    return 0;
}