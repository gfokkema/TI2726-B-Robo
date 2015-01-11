#ifndef DUALMOTOR_H_
#define DUALMOTOR_H_

class Motor;

class DualMotor {
public:
	DualMotor(Motor* left, Motor* right);

	void enable();
	void disable();
	void update(int speed, int angular);
private:
	Motor* const p_left;
	Motor* const p_right;
};

#endif /* DUALMOTOR_H_ */
