#include <AlfredoCRSF.h>
#include "LX16A-bus.h"
#include <Servo.h>
#include <math.h>

AlfredoCRSF crsf;

// ============================================================
// PIN MAP
// ============================================================

// LX16A arm bus servo
// Use Serial5 TX only: pin 20 -> servo signal
// Pin 21 (RX) is not wired / not used
const uint8_t ARM_BUS_SERVO_ID = 1;
LX16A armBusServo(ARM_BUS_SERVO_ID, Serial5);

// PWM servos
const int ROTATION_SERVO_PIN = 11;
const int BUCKET_SERVO_PIN   = 12;

// Drive motor driver pins
// IN1 = right back, IN2 = right forward, IN3 = left back, IN4 = left forward
const int RB_PIN = 4;   // IN1
const int RF_PIN = 5;   // IN2
const int LB_PIN = 2;   // IN3
const int LF_PIN = 3;   // IN4

// Arm lift actuator pins
const int ARM_UP_PIN   = 22;
const int ARM_DOWN_PIN = 23;

// Claw actuator pins
const int CLAW_OPEN_PIN  = 8;
const int CLAW_CLOSE_PIN = 9;

// ============================================================
// CRSF CHANNEL MAP
// ============================================================

const uint8_t STEERING_CRSF_CH       = 1;
const uint8_t THROTTLE_CRSF_CH       = 2;
const uint8_t ARM_LIFT_CRSF_CH       = 3;
const uint8_t ROTATION_CRSF_CH       = 4;
const uint8_t DRIVE_OVERRIDE_CRSF_CH = 5;   // CH5 scroll wheel
const uint8_t ARM_BUS_SWITCH_CRSF_CH = 6;
const uint8_t CLAW_CRSF_CH           = 7;
const uint8_t BUCKET_CRSF_CH         = 8;

// Flip these if any directions are reversed
const bool THROTTLE_INVERTED = false;
const bool STEERING_INVERTED = true;
const bool ARM_LIFT_INVERTED = false;

const uint16_t SW_LOW_MAX  = 1300;
const uint16_t SW_HIGH_MIN = 1700;

// ============================================================
// LX16A ARM BUS SERVO SETTINGS
// ============================================================

const int ARM_BUS_MIN  = 0;
const int ARM_BUS_MAX  = 240;
const int ARM_BUS_STEP = 1;
const unsigned long ARM_BUS_MS = 40;

int armBusAngle = 0;
unsigned long lastArmBusStep = 0;

// ============================================================
// PWM SERVO SETTINGS
// ============================================================

Servo rotationServo;
Servo bucketServo;

bool rotationServoAttached = false;
bool bucketServoAttached   = false;
bool bucketEnabled         = false;   // bucket ignored until CH8 pressed once

const int ROTATION_MIN = 0;
const int ROTATION_MAX = 180;
int rotationAngle = 90;

const int BUCKET_MIN       = 0;
const int BUCKET_MAX       = 180;
const int BUCKET_START_POS = 165;  // hold/rest

// 10 evenly spaced dump positions from 150 down to 90
const int BUCKET_DROP_POSITIONS[10] = {
  150, 143, 137, 130, 123,
  117, 110, 103,  97,  90
};

int bucketProfileIndex = 0; // 0..9
int bucketAngle = BUCKET_START_POS;

// ============================================================
// DRIVE OVERRIDE SETTINGS
// ============================================================

const float CH5_OVERRIDE_SPEED = 1.0f;

// 0 = normal stick control
// 1 = forced forward
// 2 = forced reverse
// 3 = pivot left
// 4 = pivot right
int readDriveOverrideModeFromCH5() {
  uint16_t raw = crsf.getChannel(DRIVE_OVERRIDE_CRSF_CH);
  if (raw == 0) return 0;

  const float RAW_MIN = 989.0f;
  const float RAW_MAX = 2012.0f;
  const float RAW_CENTER = (RAW_MIN + RAW_MAX) / 2.0f;
  const float HALF_RANGE = (RAW_MAX - RAW_MIN) / 2.0f;

  float norm = (raw - RAW_CENTER) / HALF_RANGE;
  norm = constrain(norm, -1.0f, 1.0f);

  // CH5 <= 0 => normal controller mode
  if (norm <= 0.0f) return 0;

  // Convert positive half (0..1) into 0..100 percent
  float pct = norm * 100.0f;

  if (pct <= 25.0f) return 1;   // forward
  if (pct <= 50.0f) return 2;   // reverse
  if (pct <= 75.0f) return 3;   // left pivot
  return 4;                     // right pivot
}

// ============================================================
// HELPERS
// ============================================================

void ensureRotationServoAttached() {
  if (!rotationServoAttached) {
    pinMode(ROTATION_SERVO_PIN, OUTPUT);
    rotationServo.attach(ROTATION_SERVO_PIN);
    rotationServoAttached = true;
  }
}

void ensureBucketServoAttached() {
  if (!bucketServoAttached) {
    pinMode(BUCKET_SERVO_PIN, OUTPUT);
    bucketServo.attach(BUCKET_SERVO_PIN);
    bucketServoAttached = true;
  }
}

float channelNorm(int ch) {
  uint16_t raw = crsf.getChannel(ch);

  if (raw == 0) return 0.0f;

  const float RAW_MIN = 989.0f;
  const float RAW_MAX = 2012.0f;
  const float RAW_CENTER = (RAW_MIN + RAW_MAX) / 2.0f;
  const float HALF_RANGE = (RAW_MAX - RAW_MIN) / 2.0f;

  float norm = (raw - RAW_CENTER) / HALF_RANGE;
  norm = constrain(norm, -1.0f, 1.0f);

  // 8% deadband for general controls
  if (fabs(norm) < 0.08f) return 0.0f;

  return norm;
}

// Returns:
//  1  = positive/high
//  0  = neutral/middle
// -1  = negative/low
int read3PosChannel(uint8_t ch) {
  uint16_t raw = crsf.getChannel(ch);

  if (raw == 0) return 0;
  if (raw < SW_LOW_MAX)  return -1;
  if (raw > SW_HIGH_MIN) return 1;
  return 0;
}

// For stick/axis channels used as bidirectional actuator control
int readSignedAxisChannel(uint8_t ch, bool invert = false) {
  float norm = channelNorm(ch);
  if (invert) norm = -norm;

  if (norm > 0.2f)  return 1;
  if (norm < -0.2f) return -1;
  return 0;
}

// Map CH5 raw CRSF value into 10 even bands: 0..9
int read10BandChannelRaw(uint8_t ch) {
  uint16_t raw = crsf.getChannel(ch);

  if (raw == 0) return 5;

  const int RAW_MIN = 989;
  const int RAW_MAX = 2012;

  int clamped = constrain((int)raw, RAW_MIN, RAW_MAX);
  long scaled = (long)(clamped - RAW_MIN) * 10L / (RAW_MAX - RAW_MIN + 1);

  if (scaled < 0) scaled = 0;
  if (scaled > 9) scaled = 9;

  return (int)scaled;
}

void driveMotor(int forwardPin, int backwardPin, float norm) {
  float speed = fabs(norm);
  if (speed < 0.02f) speed = 0.0f;

  uint16_t pwm = (uint16_t)(speed * 1023.0f);

  if (speed == 0.0f) {
    analogWrite(forwardPin, 0);
    analogWrite(backwardPin, 0);
    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, LOW);
  } else if (norm > 0.0f) {
    digitalWrite(backwardPin, LOW);
    analogWrite(forwardPin, pwm);
    analogWrite(backwardPin, 0);
  } else {
    digitalWrite(forwardPin, LOW);
    analogWrite(forwardPin, 0);
    analogWrite(backwardPin, pwm);
  }
}

void driveActuator(int posPin, int negPin, int state) {
  if (state == 1) {
    digitalWrite(posPin, HIGH);
    digitalWrite(negPin, LOW);
  } else if (state == -1) {
    digitalWrite(posPin, LOW);
    digitalWrite(negPin, HIGH);
  } else {
    digitalWrite(posPin, LOW);
    digitalWrite(negPin, LOW);
  }
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println("Teensy 4.1 + CRSF main control");

  // Output pins
  pinMode(LF_PIN, OUTPUT);
  pinMode(RF_PIN, OUTPUT);
  pinMode(LB_PIN, OUTPUT);
  pinMode(RB_PIN, OUTPUT);

  pinMode(ARM_UP_PIN, OUTPUT);
  pinMode(ARM_DOWN_PIN, OUTPUT);

  pinMode(CLAW_OPEN_PIN, OUTPUT);
  pinMode(CLAW_CLOSE_PIN, OUTPUT);

  // Start safe
  digitalWrite(LF_PIN, LOW);
  digitalWrite(RF_PIN, LOW);
  digitalWrite(LB_PIN, LOW);
  digitalWrite(RB_PIN, LOW);

  digitalWrite(ARM_UP_PIN, LOW);
  digitalWrite(ARM_DOWN_PIN, LOW);

  digitalWrite(CLAW_OPEN_PIN, LOW);
  digitalWrite(CLAW_CLOSE_PIN, LOW);

  // Motor PWM
  analogWriteResolution(10);
  analogWriteFrequency(LF_PIN, 20000);
  analogWriteFrequency(RF_PIN, 20000);
  analogWriteFrequency(LB_PIN, 20000);
  analogWriteFrequency(RB_PIN, 20000);

  // CRSF receiver
  Serial1.begin(CRSF_BAUDRATE);
  crsf.begin(Serial1);

  // LX16A bus servo
  Serial5.begin(115200);
  armBusServo.initialize(115200);
  armBusServo.enableTorque();
  armBusServo.setServoMode();

  // Rotation servo only
  ensureRotationServoAttached();

  // IMPORTANT:
  // Do NOT attach bucket servo in setup.
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  crsf.update();

  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'H' || cmd == 'h') {
      static bool toggled = false;
      int testAngle = toggled ? 70 : 90;
      toggled = !toggled;
      armBusServo.move(testAngle, 700);
      armBusAngle = testAngle;
      Serial.printf("[DEBUG] H - arm bus servo to %d deg\n", testAngle);
    }
    else if (cmd == 'X' || cmd == 'x') {
      driveMotor(LF_PIN, LB_PIN, 0.0f);
      driveMotor(RF_PIN, RB_PIN, 0.0f);
      Serial.println("[DEBUG] X - motors stopped");
    }
  }

  bucketProfileIndex = read10BandChannelRaw(DRIVE_OVERRIDE_CRSF_CH);

  int rawArmBusSw = crsf.getChannel(ARM_BUS_SWITCH_CRSF_CH);

  if (rawArmBusSw != 0) {
    if (rawArmBusSw < (int)SW_LOW_MAX) {
      unsigned long now = millis();
      if (now - lastArmBusStep >= ARM_BUS_MS) {
        lastArmBusStep = now;
        armBusAngle += ARM_BUS_STEP;
        armBusAngle = constrain(armBusAngle, ARM_BUS_MIN, ARM_BUS_MAX);
        armBusServo.move(armBusAngle, (uint16_t)ARM_BUS_MS);
      }
    } else if (rawArmBusSw > (int)SW_HIGH_MIN) {
      unsigned long now = millis();
      if (now - lastArmBusStep >= ARM_BUS_MS) {
        lastArmBusStep = now;
        armBusAngle -= ARM_BUS_STEP;
        armBusAngle = constrain(armBusAngle, ARM_BUS_MIN, ARM_BUS_MAX);
        armBusServo.move(armBusAngle, (uint16_t)ARM_BUS_MS);
      }
    }
  }

  uint16_t rawRot = crsf.getChannel(ROTATION_CRSF_CH);
  float rotNorm = 0.0f;

  if (rawRot != 0) {
    const float RAW_MIN = 989.0f;
    const float RAW_MAX = 2012.0f;
    const float RAW_CENTER = (RAW_MIN + RAW_MAX) / 2.0f;
    const float HALF_RANGE = (RAW_MAX - RAW_MIN) / 2.0f;

    rotNorm = (rawRot - RAW_CENTER) / HALF_RANGE;
    rotNorm = constrain(rotNorm, -1.0f, 1.0f);

    if (fabs(rotNorm) < 0.15f) rotNorm = 0.0f;
  }

  int targetRotationAngle = rotationAngle;

  if (rotNorm != 0.0f) {
    targetRotationAngle = (int)((rotNorm + 1.0f) * 0.5f * 180.0f);
    targetRotationAngle = constrain(targetRotationAngle, ROTATION_MIN, ROTATION_MAX);
  }

  static unsigned long lastRotUpdate = 0;
  if (millis() - lastRotUpdate > 40) {
    lastRotUpdate = millis();
    if (rotationAngle < targetRotationAngle) rotationAngle++;
    else if (rotationAngle > targetRotationAngle) rotationAngle--;
    rotationServo.write(rotationAngle);
  }

  int bucketSwState = read3PosChannel(BUCKET_CRSF_CH);

  if (!bucketEnabled) {
    if (bucketSwState == 1) {
      bucketEnabled = true;
      ensureBucketServoAttached();
      bucketServo.write(bucketAngle);
    }
  }

  if (bucketEnabled) {
    int targetBucketAngle =
      (bucketSwState == 1) ? BUCKET_DROP_POSITIONS[bucketProfileIndex] : BUCKET_START_POS;

    static unsigned long lastBucketUpdate = 0;
    if (millis() - lastBucketUpdate > 15) {
      lastBucketUpdate = millis();

      if (bucketAngle < targetBucketAngle) {
        bucketAngle = min(bucketAngle + 1, targetBucketAngle);
      } else if (bucketAngle > targetBucketAngle) {
        bucketAngle = max(bucketAngle - 1, targetBucketAngle);
      }

      bucketServo.write(bucketAngle);
    }
  }

  float left = 0.0f;
  float right = 0.0f;

  int driveOverrideMode = readDriveOverrideModeFromCH5();

  if (driveOverrideMode == 0) {
    float throttle = channelNorm(THROTTLE_CRSF_CH);
    if (THROTTLE_INVERTED) throttle = -throttle;

    float steering = channelNorm(STEERING_CRSF_CH);
    if (STEERING_INVERTED) steering = -steering;

    left  = throttle + steering;
    right = throttle - steering;

    left  = constrain(left,  -1.0f, 1.0f);
    right = constrain(right, -1.0f, 1.0f);
  } else if (driveOverrideMode == 1) {
    left = CH5_OVERRIDE_SPEED;
    right = CH5_OVERRIDE_SPEED;
  } else if (driveOverrideMode == 2) {
    left = -CH5_OVERRIDE_SPEED;
    right = -CH5_OVERRIDE_SPEED;
  } else if (driveOverrideMode == 3) {
    left = -CH5_OVERRIDE_SPEED;
    right = CH5_OVERRIDE_SPEED;
  } else if (driveOverrideMode == 4) {
    left = CH5_OVERRIDE_SPEED;
    right = -CH5_OVERRIDE_SPEED;
  }

  driveMotor(LF_PIN, LB_PIN, left);
  driveMotor(RF_PIN, RB_PIN, right);

  int armLiftState = readSignedAxisChannel(ARM_LIFT_CRSF_CH, ARM_LIFT_INVERTED);
  driveActuator(ARM_UP_PIN, ARM_DOWN_PIN, armLiftState);

  int clawState = read3PosChannel(CLAW_CRSF_CH);
  driveActuator(CLAW_OPEN_PIN, CLAW_CLOSE_PIN, clawState);
}
