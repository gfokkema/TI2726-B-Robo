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
ros::Subscriber<geometry_msgs::Point32> cmd_vel_sub("cmd_vel", cmd_vel_cb);

long pulse_start = 0, pulse_end = 0;
bool pulse_dirty = false;

void cmd_vel_cb(const geometry_msgs::Point32& cmd_vel_msg)
{
	Serial.print("speed: ");
	Serial.println(cmd_vel_msg.x);
	Serial.print("angular: ");
	Serial.println(cmd_vel_msg.z);
	
	motor.update(cmd_vel_msg.x, cmd_vel_msg.z);
}

void pulse_start_cb()
{
	pulse_start = micros();
}

void pulse_end_cb()
{
	pulse_end = micros();
	pulse_dirty = true;
}

void setup()
{
	nh.initNode();
	nh.subscribe(cmd_vel_sub);
}

void loop()
{
	if (pulse_dirty)
	{
		long pulse_dt = pulse_end - pulse_start;
		// TODO: calculate distance here
		int pulse_distance = 20;
	}
	nh.spinOnce();
}
