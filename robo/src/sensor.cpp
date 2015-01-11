#include "Arduino.h"
#include "sensor.h"

extern Sensor sensor;

/*******************
 * DEBUG VARIABLES *
 *******************/
long timer_start;
bool timer_start_dirty;
/*******************/

ISR(TIMER5_OVF_vect) {
	sensor.trigger();
}

Sensor::Sensor(int trigger, int echo)
: m_trigger(trigger), m_echo(echo)
{
	pinMode(trigger, OUTPUT);
	pinMode(echo,     INPUT);

	// WGM5   = 0000 (normal)
	// CS5    =  011 (64 prescaler)
	// TIMSK5 = 0005 (interrupt on TOIE)
	// overflow frequency: 16 MHz / 64 / 65536 = 1 Hz
	noInterrupts();           // disable all interrupts
	TCCR5A = 0;
	TCCR5B = _BV(CS51) | _BV(CS50);
	TIMSK5 = _BV(TOIE5);
	interrupts();             // enable all interrupts
}

int
Sensor::read()
{
	if (!digitalRead(m_echo)) return -1;

	// Measure the width of the pulse as accurately as possible
	noInterrupts();
	long start = micros();
	while (digitalRead(m_echo) && micros() - start < MAX_PULSE)
		delayMicroseconds(1);
	long end = micros();
	interrupts();

	// Speed of sound is 340 m/s or 29 cm/microsecond
	// The pulse travels back and forth, so we divide this by 2
	int dt = end - start;
	int distance = dt / 29 / 2;

	return distance;
}

void
Sensor::trigger()
{
	digitalWrite(m_trigger, HIGH);
	delayMicroseconds(10);
	digitalWrite(m_trigger, LOW);
}

