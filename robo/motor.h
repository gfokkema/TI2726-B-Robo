#ifndef MOTOR_H_
#define MOTOR_H_

class Motor
{
public:
  Motor(int rev, int en, int fwd);
  void setSpeed(int speed);
private:
  int rev, en, fwd;
  int speed;
};

#endif /* MOTOR_H_ */

