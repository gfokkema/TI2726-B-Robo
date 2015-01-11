#include "Arduino.h"
#include "motor.h"

Motor::Motor(int rev, int en, int fwd)
: m_rev(rev), m_en(en), m_fwd(fwd), m_speed(0)
{
	pinMode(rev, OUTPUT);
	pinMode(en,  OUTPUT);
	pinMode(fwd, OUTPUT);
}

void
Motor::enable()
{
	digitalWrite(m_en, HIGH);
}

void
Motor::disable()
{
	digitalWrite(m_en, LOW);
}

void
Motor::setSpeed(int speed)
{
	if (speed > 255) speed = 255;
	if (speed < -255) speed = -255;

	if (speed > 0) {
		digitalWrite(m_rev, LOW);
		analogWrite( m_fwd, speed);
	} else {
		digitalWrite(m_fwd, LOW);
		analogWrite( m_rev, -speed);
	}

	m_speed = speed;
}

