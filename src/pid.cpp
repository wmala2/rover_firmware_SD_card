#include "pid.h"
#include "control_server.h"  // for ws_logf
#include <math.h>

// interrupt masking addition
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
// ---------- NEW: tiny critical-section mux and snapshot ----------
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

struct EncSnapshot {
  long L;
  long R;
};

// Copy both encoder counts atomically (very short mask window)
static inline EncSnapshot readEncodersAtomic() {
  EncSnapshot s;
  portENTER_CRITICAL(&encMux);
  s.L = readEncoder(LEFT);
  s.R = readEncoder(RIGHT);
  portEXIT_CRITICAL(&encMux);
  return s;
}

WheelPID leftPID;
WheelPID rightPID;
volatile int  moving = 0;
volatile long nextPID = 0;

int clamp_pwm_no_floor(int out) {
  if (out >  PWM_MAX) out =  PWM_MAX;
  if (out < -PWM_MAX) out = -PWM_MAX;
  return out;
}
int apply_pwm_limits_floor(int out) {
  if (out >  PWM_MAX) out =  PWM_MAX;
  if (out < -PWM_MAX) out = -PWM_MAX;
  if (out != 0 && abs(out) < PWM_MIN) out = (out > 0 ? PWM_MIN : -PWM_MIN);
  return out;
}

void resetPID() {
  leftPID.integral = rightPID.integral = 0.0f;
  leftPID.lastErr  = rightPID.lastErr  = 0.0f;
  leftPID.PWM      = rightPID.PWM      = 0;

  // --- NEW: seed PrevEnc from a single atomic snapshot ---
  EncSnapshot s = readEncodersAtomic();
  leftPID.PrevEnc  = s.L;
  rightPID.PrevEnc = s.R;
  // -------------------------------------------------------

  leftPID.LastDelta = rightPID.LastDelta = 0;
}


// Single-wheel step with anti-windup and conditional floor
// ---------- CHANGED: pidStepOne now takes an encoder reading ----------
static inline void pidStepOne_encNow(WheelPID& w, long encNow) {
  long delta  = encNow - w.PrevEnc;
  w.PrevEnc   = encNow;
  w.LastDelta = delta;

  const float err   = (float)w.TargetTicksPerFrame - (float)delta;
  const float deriv = err - w.lastErr;

  // PID with negative Kd on delta-of-error as in your current code
  const float u_lin  = w.Kp * err + w.Ki * w.integral - w.Kd * deriv;

  // feed-forward on target ticks/frame
  const float u_ff   = w.kV * w.TargetTicksPerFrame;

  const int out_lin  = (int)lroundf(u_lin) + (int)lroundf(u_ff);

  // DEBUG (optional): comment out in production
  // Serial.println("out_lin: " + String(out_lin) + " err:" + String(err) +
  //                " uff:" + String(u_ff) + " target ticks:" + String(w.TargetTicksPerFrame));

  const bool enable_floor = (abs(w.TargetTicksPerFrame) >= FLOOR_ENABLE_TICKS);
  const int out_sat = enable_floor ? apply_pwm_limits_floor(out_lin)
                                   : clamp_pwm_no_floor(out_lin);

  const bool saturated = (out_sat != out_lin);

  if (!saturated) {
    w.integral += err;
  } else {
    if ((out_sat > 0 && err < 0) || (out_sat < 0 && err > 0)) {
      w.integral += err;
    } else {
      w.integral *= AW_DECAY;
    }
  }

  const float ki = (w.Ki == 0.0f ? 1e-6f : w.Ki);
  const float I_CLAMP = (float)PWM_MAX / ki;
  if (w.integral >  I_CLAMP) w.integral =  I_CLAMP;
  if (w.integral < -I_CLAMP) w.integral = -I_CLAMP;

  w.lastErr = err;
  w.PWM     = out_sat;
}
// ----------------------------------------------------------------------


void updatePID() {
  if (leftPID.TargetTicksPerFrame == 0 && rightPID.TargetTicksPerFrame == 0) {
    leftPID.integral  *= 0.9f;
    rightPID.integral *= 0.9f;
    leftPID.lastErr = rightPID.lastErr = 0.0f;

    // --- NEW: re-seed from a snapshot when idle so next move starts clean ---
    EncSnapshot s_idle = readEncodersAtomic();
    leftPID.PrevEnc  = s_idle.L;
    rightPID.PrevEnc = s_idle.R;
    // -----------------------------------------------------------------------

    leftPID.LastDelta = rightPID.LastDelta = 0;

    leftPID.PWM = rightPID.PWM = 0;
    setMotorSpeeds(0, 0);
    moving = 0;
    return;
  }

  // --- NEW: single atomic snapshot feeds both PIDs for this tick ---
  EncSnapshot s = readEncodersAtomic();
  pidStepOne_encNow(leftPID,  s.L);
  pidStepOne_encNow(rightPID, s.R);
  // -----------------------------------------------------------------

  // Log using the same snapshot so counts, deltas and errors are consistent
  ws_logf("ENC L=%ld R=%ld dL=%ld dR=%ld pwmL=%d pwmR=%d tgtL=%ld tgtR=%ld lLastErr=%.0f rLastErr=%.0f\n",
          s.L, s.R,
          leftPID.LastDelta, rightPID.LastDelta,
          leftPID.PWM, rightPID.PWM,
          leftPID.TargetTicksPerFrame, rightPID.TargetTicksPerFrame,
          (double)leftPID.lastErr, (double)rightPID.lastErr);

  moving = 1;
  setMotorSpeeds(leftPID.PWM, rightPID.PWM);
}
