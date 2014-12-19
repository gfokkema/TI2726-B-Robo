#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "motor.h"

#define LEFTREV   7
#define LEFTEN	 24
#define LEFTFWD   6

#define RIGHTREV  3
#define RIGHTEN  25
#define RIGHTFWD  2

void cmd_vel_cb(const geometry_msgs::Twist& cmd_vel_msg)
{
  // DO SOMETHING HERE
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);

Motor m1(LEFTREV, LEFTEN, LEFTFWD);
Motor m2(RIGHTREV, RIGHTEN, RIGHTFWD);

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  
  m1.enable(true);
  m2.enable(true);
  
  m1.setSpeed(100);
  m2.setSpeed(100);
}

void loop()
{
  nh.spinOnce();
}
