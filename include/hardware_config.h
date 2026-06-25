#pragma once
#include <Arduino.h>

// -------------------- MOTOR PINS --------------------
#ifndef LEFT_MOTOR_FWD
  #define LEFT_MOTOR_FWD 12
#endif
#ifndef LEFT_MOTOR_REV
  #define LEFT_MOTOR_REV 13
#endif
#ifndef RIGHT_MOTOR_FWD
  #define RIGHT_MOTOR_FWD 26 // - DAC pin do not use with PWM 
  // #define RIGHT_MOTOR_FWD 14
#endif
#ifndef RIGHT_MOTOR_REV
  #define RIGHT_MOTOR_REV 27
#endif

// -------------------- ENCODER PINS --------------------
#ifndef encoder0PinA
  #define encoder0PinA 21      // left B original
#endif
#ifndef encoder0PinB
  #define encoder0PinB 19      // left A original
#endif
#ifndef encoder1PinA
  #define encoder1PinA 23      // right B original
#endif
#ifndef encoder1PinB
  #define encoder1PinB 22      // right A original
#endif

// Wheel side indices
#ifndef LEFT
  // #define LEFT  0
  #define LEFT  1
#endif
#ifndef RIGHT
  // #define RIGHT 1
  #define RIGHT 0
#endif

// -------------------- ENCODER / WHEEL --------------------
#ifndef WHEEL_DIAMETER_M
  #define WHEEL_DIAMETER_M 0.070f
#endif

// ---- Motor profile selection -------------------------------------------------
// Choose via build flag: -D MOTOR_PROFILE=130  or  -D MOTOR_PROFILE=500
#ifndef MOTOR_PROFILE
  // #define MOTOR_PROFILE 130   // default
  #define MOTOR_PROFILE 500   // default
#endif

// You can override these per profile below with your calibrated values:
#if MOTOR_PROFILE == 130
  // EXAMPLE values — replace with your measured constants:
  #undef PWM_MIN
  #define PWM_MIN     50      // floor needed to start moving 130 RPM motors

  #undef ENCODER_CPR_WHEEL
  #define ENCODER_CPR_WHEEL 1980.0f   // counts per wheel revolution - measured with scope

  #undef FEEDFORWARD_VEL_CONST
  #define FEEDFORWARD_VEL_CONST 0.5f   // counts per wheel revolution

    #undef FEEDFORWARD_ACC_CONST
  #define FEEDFORWARD_ACC_CONST 0.0f   // counts per wheel revolution

  // -------------------- PID / CONTROL --------------------
  #ifndef PID_RATE
    #define PID_RATE 20 // Hz
  #endif
  #ifndef PID_INTERVAL
    #define PID_INTERVAL 50 // ms - TODO consolodate iterval and rate
  #endif
  #ifndef PWM_MAX
    #define PWM_MAX 255
  #endif
  #ifndef PWM_MIN
    #define PWM_MIN 50
  #endif
  #ifndef AW_DECAY
    #define AW_DECAY 0.95f
  #endif
  #ifndef FLOOR_ENABLE_TICKS
    #define FLOOR_ENABLE_TICKS 2
  #endif

  #ifndef FEEDBACK_KP
      #define FEEDBACK_KP 1.0
  #endif
  #ifndef FEEDBACK_KI
      #define FEEDBACK_KI 0.16
  #endif
  #ifndef FEEDBACK_KD
      #define FEEDBACK_KD 0.0
  #endif

#elif MOTOR_PROFILE == 500
  // EXAMPLE values — replace with your measured constants:
  #ifndef PWM_MIN
    #define PWM_MIN 40          // usually lower floor for higher-RPM motors
  #endif

  #ifndef ENCODER_CPR_WHEEL
    #define ENCODER_CPR_WHEEL 680.0f    // counts per wheel revolution - measured with scope
  #endif

  #ifndef FEEDFORWARD_VEL_CONST
    #define FEEDFORWARD_VEL_CONST 0.5f   // counts per wheel revolution
  #endif

  #ifndef FEEDFORWARD_ACC_CONST
    #define FEEDFORWARD_ACC_CONST 0.0f   // counts per wheel revolution
  #endif

    // -------------------- PID / CONTROL --------------------
  #ifndef PID_RATE
    #define PID_RATE 20 // Hz
  #endif
  #ifndef PID_INTERVAL
    #define PID_INTERVAL 50 // ms - TODO consolodate iterval and rate
  #endif
  #ifndef PWM_MAX
    #define PWM_MAX 255
  #endif
  #ifndef PWM_MIN
    #define PWM_MIN 40
  #endif
  #ifndef AW_DECAY
    #define AW_DECAY 0.95f
  #endif
  #ifndef FLOOR_ENABLE_TICKS
    #define FLOOR_ENABLE_TICKS 2
  #endif

  #ifndef FEEDBACK_KP
      #define FEEDBACK_KP 3.0
  #endif
  #ifndef FEEDBACK_KI
      #define FEEDBACK_KI 0.3
  #endif
  #ifndef FEEDBACK_KD
      #define FEEDBACK_KD 1.0
  #endif

#else
  #error "Unsupported MOTOR_PROFILE value (use 130 or 500)"
#endif

// -------------------- RUNTIME / LOGGING --------------------
#ifndef HEARTBEAT_MS
  #define HEARTBEAT_MS 1000
#endif
#ifndef DEFAULT_CMD_RATE_HZ
  #define DEFAULT_CMD_RATE_HZ 10 //TODO change back
#endif
#ifndef QUEUE_SECONDS
  #define QUEUE_SECONDS 3
#endif

// -------------------- HTTP / UDP PORTS --------------------
#ifndef HTTP_PORT
  #define HTTP_PORT 8080
#endif
#ifndef UDP_PORT
  #define UDP_PORT 9000
#endif
#ifndef ENCODER_REPLY_PORT
  #define ENCODER_REPLY_PORT 9001
#endif

// -------------------- TRIMS --------------------
#ifndef TRIM_LEFT
  #define TRIM_LEFT  1.00f
#endif
#ifndef TRIM_RIGHT
  #define TRIM_RIGHT 1.00f
#endif

// -------------------- HTTP/LOG defaults --------------------
#ifndef START_HTTP_ENABLED
  #define START_HTTP_ENABLED 1
#endif
#ifndef START_LOGGING_ENABLED
  #define START_LOGGING_ENABLED 1
#endif
