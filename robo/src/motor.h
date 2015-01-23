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
	void enable();
	void disable();

	/**
	 * Set the speed of this motor.
	 * Speed goes from -255 (backwards) to 255 (forwards).
	 */
	void setSpeed(int speed);
private:
	const int m_rev, m_en, m_fwd;
};

#endif /* MOTOR_H_ */

