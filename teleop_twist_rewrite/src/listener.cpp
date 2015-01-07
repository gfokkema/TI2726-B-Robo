#include "ros/ros.h"
#include "geometry_msgs/Point32.h"

void chatterCallback(const geometry_msgs::Point32::ConstPtr& msg)
{
	ROS_INFO("linear x: [%f]", msg->x);
	ROS_INFO("angular z: [%f]", msg->z);
	ROS_INFO("-------------------------");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("cmd_vel", 1000, chatterCallback);

	ros::spin();

	return 0;
}
