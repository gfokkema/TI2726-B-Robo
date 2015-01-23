#include "Arduino.h"
#include "dualmotor.h"
#include "motor.h"

extern DualMotor* motor;

ISR(TIMER1_OVF_vect) {
	motor->set(0, 0);
}

DualMotor::DualMotor(Motor* left, Motor* right)
: p_left(left), p_right(right), m_angular(0), m_speed(0), m_speedcap(1), m_dirty(false)
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

	p_left->enable();
	p_right->enable();
}

bool
DualMotor::dirty()
{
	return m_dirty;
}

void
DualMotor::resettimer()
{
	TCNT1 = 0;
}

void
DualMotor::set(int speed, int angular)
{
	noInterrupts();
	m_speed = speed;
	m_angular = angular;
	interrupts();

	m_dirty = true;
}

void
DualMotor::setSpeedcap(int distance)
{
	// Determine a speed multiplier based on sensor input
	// 0    --> input < 10 cm	0%
	// 0.25 --> input = 20 cm	25%
	// 0.5  --> input = 30 cm	50%
	// 0.75 --> input = 40 cm	75%
	// 1.0  --> input > 50 cm	full speed
	double ratio = (distance - 10) / 40.f;
	// FIXME potential bug: does max / min support floating point values?
	m_speedcap = max(0, min(1, ratio));

	m_dirty = true;
}

void
DualMotor::update()
{
	noInterrupts();
	int speed = m_speed;
	int angular = m_angular;
	interrupts();

	// Positive angular velocity means turning to the left
	Motor* inner = p_left;
	Motor* outer = p_right;
	if (angular < 0) { inner = p_right; outer = p_left; angular = -angular; }

	// Clamp speed and angular to sensible ranges
	// Only apply the speedcap when going forward (there is no sensor on the back)
	speed =   max(-255, min(255,   speed)) * (speed < 0 ? 1 : m_speedcap);
	angular =           min(511, angular)  * (speed < 0 ? 1 : m_speedcap);

	outer->setSpeed(speed);
	inner->setSpeed(speed - angular);

	m_dirty = false;
}
