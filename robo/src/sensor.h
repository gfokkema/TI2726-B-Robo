#ifndef SENSOR_H_
#define SENSOR_H_

/**
 * This class abstracts and controls a proximity sensor.
 */
class Sensor
{
public:
	/**
	 * Create a sensor using the specified pins
	 */
	Sensor(int trigger, int echo);

	/**
	 * Trigger ultrasonic emission
	 */
	void start_trigger();
	void end_trigger();

	/**
	 * Measure ultrasonic echo
	 */
	void start_pulse();
	void end_pulse();

	/**
	 * Read the echo measurements
	 */
	bool dirty();
	int read();

	/*******************
	 * DEBUG VARIABLES *
	 *******************/
	long m_timer_start, m_timer_end;
	bool m_timer_start_dirty, m_timer_end_dirty;
	/*******************/
private:
	int m_trigger, m_echo;
	long m_pulse_start, m_pulse_end;
	bool m_dirty;
};

extern Sensor sensor;

#endif /* SENSOR_H_ */

