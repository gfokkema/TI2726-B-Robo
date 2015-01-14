#ifndef DUALMOTOR_H_
#define DUALMOTOR_H_

class Motor;

class DualMotor {
public:
	DualMotor(Motor* left, Motor* right);

	bool dirty();
	void resettimer();
	void set(int speed, int angular);
	void setSpeedcap(int distance);
	void update();
private:
	volatile int m_angular, m_speed;
	Motor* const p_left;
	Motor* const p_right;
	double m_speedcap;
	bool m_dirty;
};

#endif /* DUALMOTOR_H_ */
