#include "ros.h"
#include "geometry_msgs/Point32.h"
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
ros::Subscriber<geometry_msgs::Point32> sub("cmd_vel", cmd_vel_cb);

void cmd_vel_cb(const geometry_msgs::Point32& cmd_vel_msg)
{
	Serial.print("speed: ");
	Serial.println(cmd_vel_msg.x);
	Serial.print("angular: ");
	Serial.println(cmd_vel_msg.z);
	
	motor.update(cmd_vel_msg.x, cmd_vel_msg.z);
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
