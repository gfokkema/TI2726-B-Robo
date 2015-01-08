#include "Arduino.h"
#include "sensor.h"

extern Sensor sensor;

/*******************
 * DEBUG VARIABLES *
 *******************/
long timer_start, timer_end;
bool timer_start_dirty, timer_end_dirty;
/*******************/
long pulse_start, pulse_end;
bool bdirty = false;

ISR(TIMER1_COMPA_vect)
{
	// TODO: This should not be necessary!
	// TCNT1 = 0;
	
	digitalWrite(sensor.m_trigger, LOW);

	// DEBUG
	timer_end = micros();
	timer_end_dirty = true;
}

ISR(TIMER1_COMPB_vect)
{
	digitalWrite(sensor.m_trigger, HIGH);

	// DEBUG
	timer_start = micros();
	timer_start_dirty = true;
}

void
pulse_start_cb()
{
	pulse_start = micros();
}

void
pulse_end_cb()
{
	pulse_end = micros();
	bdirty = true;
}

Sensor::Sensor(int trigger, int echo)
: m_trigger(trigger), m_echo(echo)
{
	pinMode(trigger, OUTPUT);
	pinMode(echo,     INPUT);

	// WGM1   = 0100 (ctc)
	// CS1    =  101 (1024 prescaler)
	// TIMSK1 = 0110 (interrupt on OCR1A, interrupt on OCR1B)
	// overflow frequency: 16 MHz / 1024 / 4096 = 4 Hz
	// desired width:      16 ms per 256 ms --> 4096 / 16 = 256
	noInterrupts();           // disable all interrupts
	OCR1A = 4095;
	OCR1B = 3839;
	TCCR1A = _BV(WGM12);
	TCCR1B = _BV(CS12) | _BV(CS10);
	TIMSK1 = _BV(OCIE1A) | _BV(OCIE1B);
	interrupts();             // enable all interrupts

	attachInterrupt(echo, pulse_start_cb, RISING);
	attachInterrupt(echo, pulse_end_cb,  FALLING);
}

bool
Sensor::dirty()
{
	return bdirty;
}

int
Sensor::read()
{
	// Speed of sound is 340 m/s or 29 cm/microsecond
	// The pulse travels back and forth, so we divide this by 2
	long pulse_dt = pulse_end - pulse_start;
	int pulse_distance = pulse_dt / 29 / 2;

	bdirty = false;
	return pulse_distance;
}
