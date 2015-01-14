#ifndef DUALMOTOR_H_
#define DUALMOTOR_H_

class Motor;

class DualMotor {
public:
	DualMotor(Motor* left, Motor* right);

	void resettimer();
	void set(int speed, int angular);
	void update();
private:
	volatile int m_angular, m_speed;
	Motor* const p_left;
	Motor* const p_right;
	bool m_dirty;
};

#endif /* DUALMOTOR_H_ */
