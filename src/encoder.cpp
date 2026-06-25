#include "encoder.h"


volatile long encoder0Pos = 0;    // left
volatile long encoder1Pos = 0;    // right

void doEncoderA(){
  if (digitalRead(encoder0PinA) == HIGH) {
    if (digitalRead(encoder0PinB) == LOW) { encoder0Pos = encoder0Pos + 1; }
    else { encoder0Pos = encoder0Pos - 1; }
  } else {
    if (digitalRead(encoder0PinB) == HIGH) { encoder0Pos = encoder0Pos + 1; }
    else { encoder0Pos = encoder0Pos - 1; }
  }
}

void doEncoderB(){
  if (digitalRead(encoder0PinB) == HIGH) {
    if (digitalRead(encoder0PinA) == HIGH) { encoder0Pos = encoder0Pos + 1; }
    else { encoder0Pos = encoder0Pos - 1; }
  } else {
    if (digitalRead(encoder0PinA) == LOW) { encoder0Pos = encoder0Pos + 1; }
    else { encoder0Pos = encoder0Pos - 1; }
  }
}

void doEncoderC(){
  if (digitalRead(encoder1PinA) == HIGH) {
    if (digitalRead(encoder1PinB) == LOW) { encoder1Pos = encoder1Pos - 1; }
    else { encoder1Pos = encoder1Pos + 1; }
  } else {
    if (digitalRead(encoder1PinB) == HIGH) { encoder1Pos = encoder1Pos - 1; }
    else { encoder1Pos = encoder1Pos + 1; }
  }
}

void doEncoderD(){
  if (digitalRead(encoder1PinB) == HIGH) {
    if (digitalRead(encoder1PinA) == HIGH) { encoder1Pos = encoder1Pos - 1; }
    else { encoder1Pos = encoder1Pos + 1; }
  } else {
    if (digitalRead(encoder1PinA) == LOW) { encoder1Pos = encoder1Pos - 1; }
    else { encoder1Pos = encoder1Pos + 1; }
  }
}

void init_encoders(){
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);

  attachInterrupt(encoder0PinA, doEncoderA, CHANGE);
  attachInterrupt(encoder0PinB, doEncoderB, CHANGE);
  attachInterrupt(encoder1PinA, doEncoderC, CHANGE);
  attachInterrupt(encoder1PinB, doEncoderD, CHANGE);
}

long readEncoder(int i) { return (i == LEFT) ? encoder0Pos : encoder1Pos; }
void resetEncoder(int i) { if (i == LEFT) encoder0Pos=0L; else encoder1Pos=0L; }
void resetEncoders() { resetEncoder(LEFT); resetEncoder(RIGHT); }
