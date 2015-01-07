#include "Arduino.h"
#include "sensor.h"

ISR(TIMER1_COMPA_vect)
{
	sensor.end_trigger();
}

ISR(TIMER1_COMPB_vect)
{
	sensor.start_trigger();
}

void pulse_start_cb()
{
	sensor.start_pulse();
}

void pulse_end_cb()
{
	sensor.end_pulse();
}

Sensor::Sensor(int trigger, int echo)
: m_trigger(trigger), m_echo(echo), m_dirty(false)
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

void Sensor::start_trigger()
{
	digitalWrite(m_trigger, HIGH);

	// DEBUG
	m_timer_start = micros();
	m_timer_start_dirty = true;
}

void Sensor::end_trigger()
{
	// TODO: This should not be necessary!
	// TCNT1 = 0;
	
	digitalWrite(m_trigger, LOW);

	// DEBUG
	m_timer_end = micros();
	m_timer_end_dirty = true;
}

void Sensor::start_pulse()
{
	m_pulse_start = micros();
}

void Sensor::end_pulse()
{
	m_pulse_end = micros();
	m_dirty = true;
}

bool Sensor::dirty()
{
	return m_dirty;
}

int Sensor::read()
{
	// Speed of sound is 340 m/s or 29 cm/microsecond
	// The pulse travels back and forth, so we divide this by 2
	long pulse_dt = m_pulse_end - m_pulse_start;
	int pulse_distance = pulse_dt / 29 / 2;

	m_dirty = false;
	return pulse_distance;
}
