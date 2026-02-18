#include <Arduino.h>
#include "LX16A-bus.h"

const uint8_t SERVO_ID = 1;
LX16A servo(SERVO_ID, Serial);

const int MIN_ANG = 0;
const int MAX_ANG = 240;

const int STEP_DEG = 1;                 // smaller = smoother
const unsigned long STEP_MS = 30;       // lower = faster sweep

int angleDeg = MIN_ANG;
int dir = +1;

unsigned long lastStep = 0;
unsigned long lastPrint = 0;

void setup() {
  // Serial is BOTH: servo bus + Serial Monitor on UNO
  Serial.begin(115200);
  servo.initialize(115200);

  servo.enableTorque();
  servo.setServoMode();
}

void loop() {
  unsigned long now = millis();

  // Move through full range continuously
  if (now - lastStep >= STEP_MS) {
    lastStep = now;

    angleDeg += dir * STEP_DEG;
    if (angleDeg >= MAX_ANG) { angleDeg = MAX_ANG; dir = -1; }
    if (angleDeg <= MIN_ANG) { angleDeg = MIN_ANG; dir = +1; }

    servo.move(angleDeg, (uint16_t)STEP_MS);
  }

  // Output angle 1 time per second: DIGITS ONLY
  if (now - lastPrint >= 1000) {
    lastPrint = now;
    Serial.println(angleDeg);   // prints only digits + newline
  }
}
