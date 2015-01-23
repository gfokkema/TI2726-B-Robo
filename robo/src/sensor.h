#ifndef SENSOR_H_
#define SENSOR_H_

#define MAX_PULSE 5000 // 100 cm * 58 us ~ 5000 us

/**
 * This class abstracts and controls a proximity sensor.
 */
class Sensor {
public:
	/**
	 * Create a sensor using the specified pins.
	 * Also set up a timer that will overflow 4 times a second.
	 */
	Sensor(int trigger, int echo);

	/** Called from loop
	 * Check whether the sensor is dirty (ECHO is high).
	 * If this is not the case, wait for MAX_PULSE microseconds.
	 * If ECHO is still low, stop blocking and return false.
	 * If ECHO is high, measure pulse width, block until ECHO is low and return true.
	 */
	bool dirty();

	/** Called from loop
	 * Read the last measured pulse width.
	 */
	int read();

	/** Called from ISR
	 * Send a 10 us signal to the TRIGGER pin.
	 */
	void trigger();

private:
	const int m_trigger, m_echo;
	int m_distance;
};

#endif /* SENSOR_H_ */

