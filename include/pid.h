#pragma once
#include <Arduino.h>
#include "hardware_config.h"
#include "motor_driver.h"
#include "encoder.h"

struct WheelPID {
  const char* nm = "";
  volatile long TargetTicksPerFrame = 0;
  long PrevEnc   = 0;
  long LastDelta = 0;
  float Kp = FEEDBACK_KP;
  float Ki = FEEDBACK_KI;
  float Kd = FEEDBACK_KD;
  float integral = 0.0f;
  float lastErr  = 0.0f;
  int   PWM      = 0;
  // new
  // distance per rev = pi * WHEEL_DIAMETER_M
  // rev per frame = TargetTicksPerFrame / ENCODER_CPR_WHEEL
  // dist per frame = distance per rev * rev per frame
  // distance per second = dist per frame * frames / sec
  // notional counts per tick 400 per interval at .4 m/s

  float kV = FEEDFORWARD_VEL_CONST;
  float kA = FEEDFORWARD_ACC_CONST;
};

extern WheelPID leftPID;
extern WheelPID rightPID;
extern volatile int  moving;
extern volatile long nextPID;

void resetPID();
void pidStepOne(WheelPID& w, int side);
void updatePID();

// helpers
int clamp_pwm_no_floor(int out);
int apply_pwm_limits_floor(int out);
