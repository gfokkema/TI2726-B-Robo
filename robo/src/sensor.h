#ifndef SENSOR_H_
#define SENSOR_H_

/**
 * This class abstracts and controls a proximity sensor.
 */
class Sensor {
public:
	/**
	 * Create a sensor using the specified pins
	 */
	Sensor(int trigger, int echo);

	/**
	 * Read the echo measurements
	 */
	bool dirty();
	int read();
private:
	int m_trigger, m_echo;
};

#endif /* SENSOR_H_ */

