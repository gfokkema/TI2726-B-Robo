#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sstream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    geometry_msgs::Twist twist;
    twist.linear.x = 1.0;
    twist.linear.y = 1.0;
    twist.linear.z = 1.0;
    twist.angular.x = 1.0;
    twist.angular.z = 1.0;

    cmd_vel_pub.publish(twist);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}

