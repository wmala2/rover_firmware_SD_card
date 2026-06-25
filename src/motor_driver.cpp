#include "motor_driver.h"

void robot_setup()
{
  pinMode(LEFT_MOTOR_FWD, OUTPUT);
  pinMode(LEFT_MOTOR_REV, OUTPUT);
  pinMode(RIGHT_MOTOR_FWD, OUTPUT);
  pinMode(RIGHT_MOTOR_REV, OUTPUT);

  analogWrite(LEFT_MOTOR_FWD, 0);
  analogWrite(LEFT_MOTOR_REV, 0);
  analogWrite(RIGHT_MOTOR_FWD, 0);
  analogWrite(RIGHT_MOTOR_REV, 0);
}

void setMotorSpeed(int i, int spd)
{
  unsigned char reverse = 0;
  if (spd < 0){ spd = -spd; reverse = 1; }
  if (spd > 255) spd = 255;

  if (i == LEFT){
    if (reverse == 0){ analogWrite(LEFT_MOTOR_FWD, spd); analogWrite(LEFT_MOTOR_REV, 0); }
    else { analogWrite(LEFT_MOTOR_FWD, 0); analogWrite(LEFT_MOTOR_REV, spd); }
  } else { // RIGHT
    if (reverse == 0){ analogWrite(RIGHT_MOTOR_FWD, spd); analogWrite(RIGHT_MOTOR_REV, 0); }
    else { analogWrite(RIGHT_MOTOR_FWD, 0); analogWrite(RIGHT_MOTOR_REV, spd); }
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
  setMotorSpeed(RIGHT, rightSpeed);
  setMotorSpeed(LEFT, leftSpeed);
}
