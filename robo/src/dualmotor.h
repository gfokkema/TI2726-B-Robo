#ifndef DUALMOTOR_H_
#define DUALMOTOR_H_

class Motor;

/**
 * This class abstracts and controls two DC-motors.
 * Furthermore we can pass it a linear and angular speed,
 * which it will use to individually control both DC-motors
 * so that it steers in the given direction.
 * It also controls a timer so that it stops based on an interrupt
 * when there is no input from ROS for more than 1 second.
 */
class DualMotor {
public:
	/**
	 * Create a dualmotor using the specified DC-motors.
	 * Also set up a timer that will overflow every second.
	 */
	DualMotor(Motor* left, Motor* right);

	/** Called from loop
	 * Check whether parameters of the dualmotor have changed.
	 * Parameters that can change are:
	 * - speedcap
	 * - linear / angular speed
	 */
	bool dirty();

	/** Called from callback
	 * Reset the timer count to 0.
	 * Should be called when motor speed is updated from ROS.
	 */
	void resettimer();

	/** Called from callback / ISR **
	 * Set the linear and angular speed of this dualmotor.
	 */
	void set(int speed, int angular);

	/** Called from loop **
	 * Set the speedcap of this dualmotor based on input from the Sensor class.
	 */
	void setSpeedcap(int distance);

	/** Called from loop **
	 * Process all updated values in this dualmotor and
	 * change the speed of both DC-motors accordingly.
	 */
	void update();
private:
	// Current linear and angular velocity
	volatile int m_angular, m_speed;

	// Pointers to the left and right DC-motors
	Motor* const p_left;
	Motor* const p_right;

	// Speed multiplier based on sensor distance
	double m_speedcap;
	// Tracks whether attributes of this dualmotor have changed
	bool m_dirty;
};

#endif /* DUALMOTOR_H_ */
