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

// Declare our global pointers
Motor* m1 = NULL;
Motor* m2 = NULL;
DualMotor* motor = NULL;
Sensor* sensor = NULL;

// Subscribe to the ROS topic /cmd_vel
ros::NodeHandle_<NewHardware> nh;
ros::Subscriber<geometry_msgs::Point32> cmd_vel_sub("/cmd_vel", cmd_vel_cb);

// ROS callback for messages received on /cmd_vel
// This callback resets the dualmotor timer.
// Therefore, when messages are received at a frequency < 1Hz,
// the robot will be stopped from the dualmotor timer interrupt
void cmd_vel_cb(const geometry_msgs::Point32& cmd_vel_msg)
{	
	motor->set(cmd_vel_msg.x, cmd_vel_msg.z);
	motor->resettimer();
}

void setup()
{
	// Initialize our global pointers
	m1 = new Motor(LEFTREV, LEFTEN, LEFTFWD);
	m2 = new Motor(RIGHTREV, RIGHTEN, RIGHTFWD);
	motor = new DualMotor(m1, m2);
	sensor = new Sensor(TRIGGER, ECHO);

	// Initialize our ROS subscription
	nh.initNode();
	nh.subscribe(cmd_vel_sub);
}

void loop()
{
	// Check for new ROS messages
	nh.spinOnce();

	// Check whether the sensor is dirty and update the dualmotor speedcap
	if (sensor->dirty()) motor->setSpeedcap(sensor->read());
	// Check whether the dualmotor is dirty and update it's speeds
	if (motor->dirty()) motor->update();
}
