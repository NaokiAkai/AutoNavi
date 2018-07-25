// Copyright © 2018 Naoki Akai. All rights reserved.

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class LDFS
{
private:
	ros::NodeHandle nh;
	std::string input_scan_topic_name;
	ros::Subscriber scan_sub;
	ros::Publisher line_points_pub, image_pub;
	cv::Mat local_map;
	int local_map_uo, local_map_vo;
	double local_map_pix_size;

public:
	LDFS();
	~LDFS() {};
	void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void init_local_map(const sensor_msgs::LaserScan::ConstPtr& msg);

	inline void xy2uv(double x, double y, int* u, int* v)
	{
		*u = (int)(x / local_map_pix_size) + local_map_uo;
		*v = local_map_vo - (int)(y / local_map_pix_size);
	}

	inline bool check_uv(int u, int v)
	{
		if (0 <= u && u < local_map.cols && 0 <= v && v < local_map.rows)
			return true;
		else
			return false;
	}
};

LDFS::LDFS():
	nh("~"),
	input_scan_topic_name("/scan"),
	local_map_pix_size(0.1)
{
	// read parameters
	nh.param("/line_detector_from_scan/input_scan_topic_name", input_scan_topic_name, input_scan_topic_name);
	nh.param("/line_detector_from_scan/local_map_pix_size", local_map_pix_size, local_map_pix_size);
	// subscriber
	scan_sub = nh.subscribe(input_scan_topic_name, 10, &LDFS::scan_callback, this);
	// publisher
//	line_points_pub = nh.advertise<>
	image_pub = nh.advertise<sensor_msgs::Image>("/line_detection_image", 1);
	// spin
	ros::spin();
}

void LDFS::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	static bool is_first = true;
	if (is_first)
	{
		init_local_map(msg);
		is_first = false;
	}
	local_map = cv::Mat::zeros(local_map.rows, local_map.cols, CV_8U);
	cv::Mat local_map_color = cv::Mat::zeros(local_map.rows, local_map.cols, CV_8UC3);
	for (int i = 0; i < msg->ranges.size(); i++)
	{
		double r = msg->ranges[i];
		if (r < msg->range_min || msg->range_max < r)
			continue;
		double yaw = msg->angle_min + msg->angle_increment * (double)i;
		double x = r * cos(yaw);
		double y = r * sin(yaw);
		int u, v;
		xy2uv(x, y, &u, &v);
		if (check_uv(u, v))
		{
			local_map.at<uchar>(v, u) = 255;
			local_map_color.at<cv::Vec3b>(v, u)[0] = local_map_color.at<cv::Vec3b>(v, u)[1] = local_map_color.at<cv::Vec3b>(v, u)[2] = 255;
		}
	}
	cv::vector<cv::Vec4i> lines;
	cv::HoughLinesP(local_map, lines, 2.0, CV_PI / 90.0, 10, 5.0, 5.0);
	std::random_device rnd;
	std::mt19937 mt(rnd());
	std::uniform_int_distribution<> rand255(0, 255);
	for (int i = 0; i < lines.size(); i++)
	{
		int b = rand255(mt);
		int g = rand255(mt);
		int r = rand255(mt);
		cv::line(local_map_color, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(b, g, r), 2, 8);
	}
	cv_bridge::CvImage local_map_msg(msg->header, "bgr8", local_map_color);
	image_pub.publish(local_map_msg.toImageMsg());
}

void LDFS::init_local_map(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	double range_max = 30.0;
	double height = range_max * 2.0;
	double width;
	double bangle_min = fabs(msg->angle_min) - M_PI / 2.0;
	double bangle_max = fabs(msg->angle_max) - M_PI / 2.0;
	if (bangle_min > 0.0 && bangle_min >= bangle_max)
	{
		width = range_max * (1.0 + sin(bangle_min));
		local_map_uo = (int)(range_max * sin(bangle_min) / local_map_pix_size);
	}
	else if (bangle_max > 0.0 && bangle_max >= bangle_min)
	{
		width = range_max * (1.0 + sin(bangle_max));
		local_map_uo = (int)(range_max * sin(bangle_max) / local_map_pix_size);
	}
	else
	{
		width = msg->range_max;
		local_map_uo = 0;
	}
	int cols = (int)(width / local_map_pix_size);
	int rows = (int)(height / local_map_pix_size);
	local_map_vo = rows / 2;
	local_map = cv::Mat::zeros(rows, cols, CV_8U);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "line_detector_from_scan");
	LDFS node;
	return 0;
}
