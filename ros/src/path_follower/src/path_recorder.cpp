#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class PathRecorder
{
private:
	ros::NodeHandle nh;
	std::string map_frame, base_link_frame;
	std::string output_path_topic_name;
	ros::Publisher path_pub;
	nav_msgs::Path path;
	double dist_interval, angle_interval;
	tf::TransformListener tf_listener;

public:
	PathRecorder();
	~PathRecorder() {};
	void spin(void);
};

PathRecorder::PathRecorder():
	nh("~"),
	map_frame("/map"),
	base_link_frame("/base_link"),
	output_path_topic_name("/recorded_path"),
	dist_interval(0.5),
	angle_interval(2.0),
	tf_listener()
{
	// read parameters
	nh.param("/path_recorder/map_frame", map_frame, map_frame);
	nh.param("/path_recorder/base_link_frame", base_link_frame, base_link_frame);
	nh.param("/path_recorder/output_path_topic_name", output_path_topic_name, output_path_topic_name);
	nh.param("/path_recorder/dist_interval", dist_interval, dist_interval);
	nh.param("/path_recorder/angle_interval", angle_interval, angle_interval);
	// convert degree to radian
	angle_interval *= M_PI / 180.0;
	// publisher
	path_pub = nh.advertise<nav_msgs::Path>(output_path_topic_name, 1, true);
}

void PathRecorder::spin(void)
{
	bool is_first = true;
	double xo, yo, yawo;
	bool do_record = false;
	ros::Rate loop_rate(40);
	while (ros::ok())
	{
		ros::spinOnce();
		tf::StampedTransform map2base_link;
		ros::Time now = ros::Time::now();
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
		double x = map2base_link.getOrigin().x();
		double y = map2base_link.getOrigin().y();
		tf::Quaternion q(map2base_link.getRotation().x(), map2base_link.getRotation().y(), map2base_link.getRotation().z(), map2base_link.getRotation().w());
		double roll, pitch, yaw;
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);
		if (is_first)
		{
			do_record = true;
			is_first = false;
		}
		else
		{
			double dx = x - xo;
			double dy = y - yo;
			double dl = sqrt(dx * dx + dy * dy);
			double dyaw = yaw - yawo;
			if (dyaw < -M_PI)	dyaw += 2.0 * M_PI;
			if (dyaw > M_PI)	dyaw -= 2.0 * M_PI;
			if (dl > dist_interval || fabs(dyaw) > angle_interval)
				do_record = true;
		}
		if (do_record)
		{
			geometry_msgs::PoseStamped pose;
			tf::Transform tf;
			tf::Quaternion q;
			tf.setOrigin(tf::Vector3(x, y, 0.0));
			q.setRPY(0.0, 0.0, yaw);
			tf.setRotation(q);
			pose.pose.position.x = tf.getOrigin().x();
			pose.pose.position.y = tf.getOrigin().y();
			pose.pose.position.z = tf.getOrigin().z();
			pose.pose.orientation.x = tf.getRotation().x();
			pose.pose.orientation.y = tf.getRotation().y();
			pose.pose.orientation.z = tf.getRotation().z();
			pose.pose.orientation.w = tf.getRotation().w();
			path.header.frame_id = pose.header.frame_id = map_frame;
			path.header.stamp = pose.header.stamp = ros::Time::now();
			path.poses.push_back(pose);
			path_pub.publish(path);
			printf("x = %.3lf [m], y = %.3lf [m], yaw = %lf [deg]\n", x, y, yaw * 180.0 / M_PI);
			xo = x;
			yo = y;
			yawo = yaw;
			do_record = false;
		}
		loop_rate.sleep();
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_recorder");
	PathRecorder node;
	node.spin();
	return 0;
}
