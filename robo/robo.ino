#include "motor.h"

#define LEFTREV   7
#define LEFTEN	 24
#define LEFTFWD   6

#define RIGHTREV  3
#define RIGHTEN  25
#define RIGHTFWD  2

Motor m1(LEFTREV, LEFTEN, LEFTFWD);
Motor m2(RIGHTREV, RIGHTEN, RIGHTFWD);

void setup()
{
  Serial1.begin(57600);
  m1.enable(true);
  m2.enable(true);
  
  m1.setSpeed(100);
  m2.setSpeed(100);
}

void loop()
{
  if (Serial1.available() > 0) {
  }
}
