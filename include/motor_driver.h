#pragma once
#include <Arduino.h>
#include "hardware_config.h"

void robot_setup();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
