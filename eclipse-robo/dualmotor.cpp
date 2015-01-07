#include "dualmotor.h"
#include "motor.h"

DualMotor::DualMotor(Motor* left, Motor* right)
: p_left(left), p_right(right)
{
	p_left->enable(true);
	p_right->enable(true);
}

DualMotor::~DualMotor() {
	p_left->enable(false);
	p_right->enable(false);
}

void
DualMotor::update(int speed, int angular) {
	Motor* inner = p_right;
	Motor* outer = p_left;
	if (angular < 0) { inner = p_left; outer = p_right; angular = -angular; }

	if (speed > 255)  speed = 255;
	if (speed < -255) speed = -255;
	outer->setSpeed(speed);

	if (angular > speed + 255) angular = speed + 255;
	inner->setSpeed(speed - angular);
}
