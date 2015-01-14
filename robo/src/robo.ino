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

Motor* m1 = NULL;
Motor* m2 = NULL;
DualMotor* motor = NULL;
Sensor* sensor = NULL;

ros::NodeHandle_<NewHardware> nh;
ros::Subscriber<geometry_msgs::Point32> cmd_vel_sub("/cmd_vel", cmd_vel_cb);

void cmd_vel_cb(const geometry_msgs::Point32& cmd_vel_msg)
{	
	motor->set(cmd_vel_msg.x, cmd_vel_msg.z);
	motor->resettimer();
}

void setup()
{
	m1 = new Motor(LEFTREV, LEFTEN, LEFTFWD);
	m2 = new Motor(RIGHTREV, RIGHTEN, RIGHTFWD);
	motor = new DualMotor(m1, m2);
	sensor = new Sensor(TRIGGER, ECHO);

	Serial.begin(9600);
	nh.initNode();
	nh.subscribe(cmd_vel_sub);
}

void loop()
{		
	int obstacle = sensor->read();
	if (obstacle > 0 && obstacle < 10)
	{
		motor->set(0, 0);
	}
	
	motor->update();
	nh.spinOnce();
}
