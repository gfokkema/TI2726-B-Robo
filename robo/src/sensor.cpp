#include "Arduino.h"
#include "sensor.h"

extern Sensor* sensor;

ISR(TIMER5_OVF_vect) {
	sensor->trigger();
}

/**
 * Sets up the sensor and a timer at 4 Hz
 */
Sensor::Sensor(int trigger, int echo)
: m_trigger(trigger), m_echo(echo), m_distance(0)
{
	pinMode(trigger, OUTPUT);
	pinMode(echo,     INPUT);

	// WGM5   = 0000 (normal)
	// CS5    =  011 (64 prescaler)
	// TIMSK5 = 0001 (interrupt on TOIE)
	// overflow frequency: 16 MHz / 64 / 65536 = 4 Hz
	noInterrupts();           // disable all interrupts
	TCCR5A = 0;
	TCCR5B = _BV(CS51) | _BV(CS50);
	TIMSK5 = _BV(TOIE5);
	interrupts();             // enable all interrupts
}

/**
 * loop
 */
bool
Sensor::dirty()
{
	long start = micros();
	while (!digitalRead(m_echo) && micros() - start < MAX_PULSE)
		delayMicroseconds(1);
	if (!digitalRead(m_echo)) return false;

	// Measure the width of the pulse as accurately as possible
	noInterrupts();
	start = micros();
	while (digitalRead(m_echo) && micros() - start < MAX_PULSE)
		delayMicroseconds(1);
	long end = micros();
	interrupts();

	// Speed of sound is 340 m/s or 29 cm/microsecond
	// The pulse travels back and forth, so we divide this by 2
	m_distance = (end - start) / 29;

	return true;
}

/**
 * loop
 */
int
Sensor::read()
{
	return m_distance;
}

/**
 * ISR
 */
void
Sensor::trigger()
{
	digitalWrite(m_trigger, HIGH);
	delayMicroseconds(10);
	digitalWrite(m_trigger, LOW);
}

