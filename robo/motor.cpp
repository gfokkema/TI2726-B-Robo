#include "Arduino.h"
#include "motor.h"

Motor::Motor(int rev, int en, int fwd)
: rev(rev), en(en), fwd(fwd), speed(0)
{
  pinMode(rev, OUTPUT);
  pinMode(en, OUTPUT);
  pinMode(fwd, OUTPUT);
}

void Motor::setSpeed(int speed)
{
  if (abs(speed) > 255) return;
  
  if (speed > 0) {
    digitalWrite(rev, LOW);
    analogWrite(fwd, speed);
  } else {
    digitalWrite(fwd, LOW);
    analogWrite(rev, -speed);
  }
}

