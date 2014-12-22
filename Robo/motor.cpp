#include "Arduino.h"
#include "motor.h"

Motor::Motor(int rev, int en, int fwd)
: m_rev(rev), m_en(en), m_fwd(fwd), m_speed(0)
{
	pinMode(rev, OUTPUT);
	pinMode(en,  OUTPUT);
	pinMode(fwd, OUTPUT);
}

void Motor::enable(bool toggle)
{
	if (toggle) digitalWrite(m_en, HIGH);
	else        digitalWrite(m_en, LOW);
}

void Motor::setSpeed(int speed)
{
	if (abs(speed) > 255) return;

	if (speed > 0) {
		digitalWrite(m_rev, LOW);
		analogWrite( m_fwd, speed);
	} else {
		digitalWrite(m_fwd, LOW);
		analogWrite( m_rev, -speed);
	}

	m_speed = speed;
}
