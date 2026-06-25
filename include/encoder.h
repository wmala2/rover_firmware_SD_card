#pragma once
#include <Arduino.h>
#include "hardware_config.h"
/*
  Encoder interface (symmetric, robot-centric)

  - Uses identical quadrature rules for LEFT and RIGHT.
  - Provides per-wheel polarity so "robot forward" yields positive counts on BOTH wheels.
  - Exposes normalized reads via readEncoder(side) and raw reads via readEncoderRaw(side).
  - Keeps legacy globals encoder0Pos / encoder1Pos updated so existing code that inspects them still works.
*/

// ---------------------------
// Side identifiers
// ---------------------------
#ifndef LEFT
#define LEFT  0
#endif
#ifndef RIGHT
#define RIGHT 1
#endif

// ---------------------------
// PIN ASSIGNMENTS (adjust to your wiring)
// If your pins are defined elsewhere before including this header, these won't override them.
// ---------------------------
// #ifndef ENC0_PIN_A
// #define ENC0_PIN_A  19   // LEFT encoder channel A
// #endif
// #ifndef ENC0_PIN_B
// #define ENC0_PIN_B  21   // LEFT encoder channel B
// #endif
// #ifndef ENC1_PIN_A
// #define ENC1_PIN_A  22   // RIGHT encoder channel A
// #endif
// #ifndef ENC1_PIN_B
// #define ENC1_PIN_B  23   // RIGHT encoder channel B
// #endif
#ifndef ENC0_PIN_A
#define ENC0_PIN_A  22   // LEFT encoder channel A
#endif
#ifndef ENC0_PIN_B
#define ENC0_PIN_B  23   // LEFT encoder channel B
#endif
#ifndef ENC1_PIN_A
#define ENC1_PIN_A  19   // RIGHT encoder channel A
#endif
#ifndef ENC1_PIN_B
#define ENC1_PIN_B  21   // RIGHT encoder channel B
#endif

// ---------------------------
// Robot-centric encoder polarity
// Set so that pushing the ROBOT FORWARD by hand makes BOTH sides count UP.
// If one side runs negative when pushing forward, flip that side to -1.
// ---------------------------
#ifndef ENCODER_POLARITY_LEFT
#define ENCODER_POLARITY_LEFT   (-1)
#endif
#ifndef ENCODER_POLARITY_RIGHT
#define ENCODER_POLARITY_RIGHT  (-1)   // flip to (-1) if your right reads backward
#endif

// Public API
void init_encoders();
void resetEncoders();

// Normalized counts (forward = + on both wheels)
long readEncoder(int side);

// Raw, unnormalized counters (for debugging / compatibility)
long readEncoderRaw(int side);

// Legacy globals retained for compatibility (updated by ISRs)
extern volatile long encoder0Pos;  // LEFT raw count
extern volatile long encoder1Pos;  // RIGHT raw count
