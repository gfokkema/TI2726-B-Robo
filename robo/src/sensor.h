#ifndef SENSOR_H_
#define SENSOR_H_

#define MAX_PULSE 5000 // 100 cm * 58 us ~ 5000 us FIXME

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
	void trigger();

private:
	const int m_trigger, m_echo;
	int m_distance;
};

#endif /* SENSOR_H_ */

