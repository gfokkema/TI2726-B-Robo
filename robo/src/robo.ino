#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "motor.h"
#include "dualmotor.h"

#define LEFTREV   7
#define LEFTEN	 24
#define LEFTFWD   6
#define RIGHTREV  3
#define RIGHTEN  25
#define RIGHTFWD  2

class NewHardware : public ArduinoHardware {
public:
	NewHardware() : ArduinoHardware(&Serial1, 57600) {};
};

Motor m1(LEFTREV, LEFTEN, LEFTFWD);
Motor m2(RIGHTREV, RIGHTEN, RIGHTFWD);
DualMotor motor(&m1, &m2);

ros::NodeHandle_<NewHardware> nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);

void cmd_vel_cb(const geometry_msgs::Twist& cmd_vel_msg)
{
	motor.update(cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
}

void setup()
{
	nh.initNode();
	nh.subscribe(sub);
}

void loop()
{
	nh.spinOnce();
}
