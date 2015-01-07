#ifndef MOTOR_H_
#define MOTOR_H_

/**
 * This class abstracts and controls a single DC-motor.
 */
class Motor {
public:
	/**
	 * Create a motor using the specified pins
	 */
	Motor(int rev, int en, int fwd);

	/**
	 * Enable or disable this motor
	 */
	void enable(bool toggle);

	/**
	 * Set the speed of this motor
	 */
	void setSpeed(int speed);
private:
	int m_rev, m_en, m_fwd;
	int m_speed;
};

#endif /* MOTOR_H_ */

