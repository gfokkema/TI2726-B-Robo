#ifndef DUALMOTOR_H_
#define DUALMOTOR_H_

class Motor;
class DualMotor {
public:
	DualMotor(Motor* left, Motor* right);
	~DualMotor();

	void update(int speed, int angular);
private:
	Motor* p_left;
	Motor* p_right;
};

#endif /* DUALMOTOR_H_ */
