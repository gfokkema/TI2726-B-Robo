#include "Arduino.h"
#include "dualmotor.h"
#include "motor.h"

extern DualMotor motor;

ISR(TIMER1_OVF_vect) {
	motor.disable();
}

DualMotor::DualMotor(Motor* left, Motor* right)
: p_left(left), p_right(right)
{
	// WGM1   = 0000 (normal)
	// CS1    =  100 (256 prescaler)
	// TIMSK1 = 0001 (interrupt on TOIE)
	// overflow frequency: 16 MHz / 256 / 65536 = 1 Hz
	noInterrupts();           // disable all interrupts
	TCCR1A = 0;
	TCCR1B = _BV(CS12);
	TIMSK1 = _BV(TOIE1);
	interrupts();             // enable all interrupts
}

void
DualMotor::disable()
{
	p_left->disable();
	p_right->disable();
}

void
DualMotor::enable()
{
	p_left->enable();
	p_right->enable();
}

void
DualMotor::update(int speed, int angular)
{
	enable();

	Motor* inner = p_right;
	Motor* outer = p_left;
	if (angular < 0) { inner = p_left; outer = p_right; angular = -angular; }

	// TODO: These checks are probably unnecessary
	// if (speed > 255)  speed = 255;
	// if (speed < -255) speed = -255;
	outer->setSpeed(speed);

	// if (angular > speed + 255) angular = speed + 255;
	inner->setSpeed(speed - angular);
}
