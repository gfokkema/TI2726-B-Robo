#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ROS_INFO("linear x: [%f]", msg->linear.x);
  ROS_INFO("linear y: [%f]", msg->linear.y);
  ROS_INFO("linear z: [%f]", msg->linear.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, chatterCallback);

  ros::spin();

  return 0;
}
