#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include <sstream>
#include <stdio.h>
#include <termios.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

int kfd = 0;
struct termios cooked, raw;
double linear_, angular_, l_scale_, a_scale_;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	ros::Publisher twist_pub_ = n.advertise<geometry_msgs::Point32>("/cmd_vel", 1000);

	ros::Rate loop_rate(10);

	char c;
	bool dirty = false;
	geometry_msgs::Point32 twist;
	a_scale_ = l_scale_ = 10;

	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use arrow keys to move the turtle.");

	while (ros::ok())
	{
		// get the next event from the keyboard
		if(read(kfd, &c, 1) < 0)
		{
			perror("read():");
			exit(-1);
		}

		linear_ = angular_ = 0;
		ROS_DEBUG("value: 0x%02X\n", c);

		switch(c)
		{
			case KEYCODE_L:
				ROS_DEBUG("LEFT");
				angular_ = -1.0;
				dirty = true;
				break;
			case KEYCODE_R:
				ROS_DEBUG("RIGHT");
				angular_ = 1.0;
				dirty = true;
				break;
			case KEYCODE_U:
				ROS_DEBUG("UP");
				linear_ = 1.0;
				dirty = true;
				break;
			case KEYCODE_D:
				ROS_DEBUG("DOWN");
				linear_ = -1.0;
				dirty = true;
				break;
		}

		twist.x += l_scale_ * linear_;
		twist.z += a_scale_ * angular_;
		geometry_msgs::Point32 msg = twist;

		if(dirty == true)
		{
			twist_pub_.publish(msg);
			dirty = false;
		}
	}

	return 0;
}

