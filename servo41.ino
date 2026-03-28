#include <Arduino.h>
#include "LX16A-bus.h"

const uint8_t SERVO_ID = 1;
LX16A servo(SERVO_ID, Serial2);

const int MIN_ANG = 0;
const int MAX_ANG = 40;

int STEP_DEG = 1;
unsigned long STEP_MS = 30;   // <-- adjustable speed

int angleDeg = MIN_ANG;
int dir = +1;

unsigned long lastStep = 0;
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  servo.initialize(115200);
  servo.enableTorque();
  servo.setServoMode();

  Serial.println("Type a number (ms) to change speed. Example: 10 = fast, 100 = slow");
}

void loop() {
  unsigned long now = millis();

  // 🔧 Read speed from Serial
  if (Serial.available()) {
    int newSpeed = Serial.parseInt();
    if (newSpeed > 0) {
      STEP_MS = newSpeed;
      Serial.print("New STEP_MS: ");
      Serial.println(STEP_MS);
    }
  }

  // Sweep motion
  if (now - lastStep >= STEP_MS) {
    lastStep = now;

    angleDeg += dir * STEP_DEG;

    if (angleDeg >= MAX_ANG) {
      angleDeg = MAX_ANG;
      dir = -1;
    }
    if (angleDeg <= MIN_ANG) {
      angleDeg = MIN_ANG;
      dir = +1;
    }

    servo.move(angleDeg, (uint16_t)STEP_MS);
  }

  // Debug print
  if (now - lastPrint >= 1000) {
    lastPrint = now;
    Serial.print("Angle: ");
    Serial.print(angleDeg);
    Serial.print(" | Speed(ms): ");
    Serial.println(STEP_MS);
  }
}
