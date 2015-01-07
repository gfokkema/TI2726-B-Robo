#include "ros.h"
#include "geometry_msgs/Point32.h"

#include "dualmotor.h"
#include "motor.h"
#include "sensor.h"

// Left engine pins
#define LEFTREV   7
#define LEFTEN	 24
#define LEFTFWD   6

// Right engine pins
#define RIGHTREV  3
#define RIGHTEN  25
#define RIGHTFWD  2

// Proximity sensor pins
#define ECHO     22
#define TRIGGER  23

class NewHardware : public ArduinoHardware {
public:
	NewHardware() : ArduinoHardware(&Serial1, 57600) {};
};

Motor m1(LEFTREV, LEFTEN, LEFTFWD);
Motor m2(RIGHTREV, RIGHTEN, RIGHTFWD);
DualMotor motor(&m1, &m2);
Sensor sensor(TRIGGER, ECHO);

ros::NodeHandle_<NewHardware> nh;
ros::Subscriber<geometry_msgs::Point32> cmd_vel_sub("cmd_vel", cmd_vel_cb);

void cmd_vel_cb(const geometry_msgs::Point32& cmd_vel_msg)
{
	// DEBUG
	Serial.print("speed: ");
	Serial.println(cmd_vel_msg.x);
	Serial.print("angular: ");
	Serial.println(cmd_vel_msg.z);
	
	motor.update(cmd_vel_msg.x, cmd_vel_msg.z);
}

void setup()
{
	nh.initNode();
	nh.subscribe(cmd_vel_sub);
}

void loop()
{
	if (sensor.m_timer_start_dirty)
	{
		Serial.print("start: ");
		Serial.println(sensor.m_timer_start);
		sensor.m_timer_start_dirty = false;
	}
	
	if (sensor.m_timer_end_dirty)
	{
		Serial.print("end:   ");
		Serial.println(sensor.m_timer_end);
		sensor.m_timer_end_dirty = false;
	}
	
	if (sensor.dirty())
	{
		Serial.print("Distance to obstacle: ");
		Serial.println(sensor.read());
	}
	
	nh.spinOnce();
}
