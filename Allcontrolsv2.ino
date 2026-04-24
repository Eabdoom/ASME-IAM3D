#include <AlfredoCRSF.h>
#include "LX16A-bus.h"
#include <Servo.h>
#include <math.h>

AlfredoCRSF crsf;

// ============================================================
// PIN MAP
// ============================================================

// LX16A bus servo on Serial5: TX=20, RX=21
const uint8_t ARM_BUS_SERVO_ID = 1;
LX16A armBusServo(ARM_BUS_SERVO_ID, Serial5);

// PWM servos
const int ROTATION_SERVO_PIN = 11;
const int BUCKET_SERVO_PIN   = 12;

// Drive motor driver pins
const int LF_PIN = 2;
const int RF_PIN = 3;
const int LB_PIN = 4;
const int RB_PIN = 5;

// Arm lift actuator pins
const int ARM_UP_PIN   = 7;
const int ARM_DOWN_PIN = 8;

// Claw actuator pins
const int CLAW_OPEN_PIN  = 9;
const int CLAW_CLOSE_PIN = 10;

// ============================================================
// CRSF CHANNEL MAP
// ============================================================

// CH3 = throttle (left stick vertical)
// CH4 = steering (left stick horizontal)
// If robot goes backward when pushing forward, flip this to true
const bool THROTTLE_INVERTED = false;

const uint8_t ARM_BUS_SWITCH_CRSF_CH = 9;
const uint8_t ARM_LIFT_CRSF_CH       = 6;
const uint8_t CLAW_CRSF_CH           = 6;

const uint16_t SW_LOW_MAX  = 1300;
const uint16_t SW_HIGH_MIN = 1700;

// ============================================================
// LX16A ARM BUS SERVO SETTINGS
// ============================================================

const int ARM_BUS_MIN  = 0;
const int ARM_BUS_MAX  = 240;
const int ARM_BUS_STEP = 1;
const unsigned long ARM_BUS_MS = 60;

int armBusAngle = 0;
unsigned long lastArmBusStep = 0;

// ============================================================
// PWM SERVO SETTINGS
// ============================================================

Servo rotationServo;
Servo bucketServo;

bool rotationServoAttached = false;
bool bucketServoAttached   = false;

const int ROTATION_MIN = 0;
const int ROTATION_MAX = 180;
int rotationAngle = 90;

const int BUCKET_MIN = 0;
const int BUCKET_MAX = 180;
const int BUCKET_START_POS = 170;
int bucketAngle = BUCKET_START_POS;

const int STARTUP_STEP_DELAY_MS = 45;

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

  // Failsafe: if CRSF connection has not established, return 0.0 (Stop)
  if (raw == 0) return 0.0f;

  const float RAW_MIN = 989.0f;
  const float RAW_MAX = 2012.0f;
  const float RAW_CENTER = (RAW_MIN + RAW_MAX) / 2.0f;
  const float HALF_RANGE = (RAW_MAX - RAW_MIN) / 2.0f;

  float norm = (raw - RAW_CENTER) / HALF_RANGE;
  norm = constrain(norm, -1.0f, 1.0f);
  
  // 4% Deadband to eliminate stick jitter and servo twitch
  if (fabs(norm) < 0.04f) {
    return 0.0f;
  }
  return norm;
}

int read3PosChannel(uint8_t ch) {
  uint16_t raw = crsf.getChannel(ch);

  // Failsafe: if CRSF hasn't connected, return STOP logic (1)
  if (raw == 0) return 1;

  if (raw < SW_LOW_MAX) return 0;
  if (raw < SW_HIGH_MIN) return 1;
  return 2;
}

void driveMotor(int forwardPin, int backwardPin, float norm) {
  float speed = fabs(norm);
  if (speed < 0.02f) speed = 0.0f;

  uint16_t pwm = (uint16_t)(speed * 1023.0f);

  if (norm >= 0.0f) {
    analogWrite(forwardPin, pwm);
    analogWrite(backwardPin, 0);
  } else {
    analogWrite(forwardPin, 0);
    analogWrite(backwardPin, pwm);
  }
}

void driveActuator(int pinA, int pinB, int state) {
  if (state == 0) {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
  } else if (state == 2) {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
  } else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
  }
}

void slowMovePWMServo(Servo &s, int fromAngle, int toAngle, int stepDelayMs) {
  fromAngle = constrain(fromAngle, 0, 180);
  toAngle   = constrain(toAngle, 0, 180);

  if (fromAngle < toAngle) {
    for (int a = fromAngle; a <= toAngle; a++) {
      s.write(a);
      delay(stepDelayMs);
    }
  } else {
    for (int a = fromAngle; a >= toAngle; a--) {
      s.write(a);
      delay(stepDelayMs);
    }
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

  // Start outputs safe
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
  // Removed forced move to ARM_BUS_MIN here to prevent violent jumping on startup

  // PWM servos back on
  ensureRotationServoAttached();     // attach only, no startup move
  ensureBucketServoAttached();       // attach and move bucket slowly to rest
  slowMovePWMServo(bucketServo, 90, BUCKET_START_POS, STARTUP_STEP_DELAY_MS);
  bucketAngle = BUCKET_START_POS;

  // ---- PIN MAP TEST ----
  // Fires each motor pin alone for 1.5s so you can see which motor/direction it is.
  // Order: pin 2 → pin 3 → pin 4 → pin 5
  // Watch which motor moves at each step and note it down.
  Serial.println("=== PIN MAP TEST: pin 2 ===");
  analogWrite(2, 512); delay(1500); analogWrite(2, 0); delay(500);

  Serial.println("=== PIN MAP TEST: pin 3 ===");
  analogWrite(3, 512); delay(1500); analogWrite(3, 0); delay(500);

  Serial.println("=== PIN MAP TEST: pin 4 ===");
  analogWrite(4, 512); delay(1500); analogWrite(4, 0); delay(500);

  Serial.println("=== PIN MAP TEST: pin 5 ===");
  analogWrite(5, 512); delay(1500); analogWrite(5, 0); delay(500);

  Serial.println("=== PIN MAP TEST DONE — entering normal operation ===");
  // ---- END PIN MAP TEST ----
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  crsf.update();

  // Serial debug commands
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

    // Individual pin tests — fire one pin at a time to map which pin = which motor/direction
    // Stop with X between each test.
    else if (cmd == '1') {
      analogWrite(2, 512); analogWrite(3, 0); analogWrite(4, 0); analogWrite(5, 0);
      Serial.println("[DEBUG] 1 - pin 2 only at 50%");
    }
    else if (cmd == '2') {
      analogWrite(2, 0); analogWrite(3, 512); analogWrite(4, 0); analogWrite(5, 0);
      Serial.println("[DEBUG] 2 - pin 3 only at 50%");
    }
    else if (cmd == '3') {
      analogWrite(2, 0); analogWrite(3, 0); analogWrite(4, 512); analogWrite(5, 0);
      Serial.println("[DEBUG] 3 - pin 4 only at 50%");
    }
    else if (cmd == '4') {
      analogWrite(2, 0); analogWrite(3, 0); analogWrite(4, 0); analogWrite(5, 512);
      Serial.println("[DEBUG] 4 - pin 5 only at 50%");
    }
    else if (cmd == 'X' || cmd == 'x') {
      analogWrite(2, 0); analogWrite(3, 0); analogWrite(4, 0); analogWrite(5, 0);
      Serial.println("[DEBUG] X - all pins stopped");
    }
    // Print all 16 CRSF channels
    else if (cmd == 'P' || cmd == 'p') {
      Serial.print("[DEBUG] CH: ");
      for (int i = 1; i <= 16; i++) {
        Serial.printf("%2d=%4d  ", i, crsf.getChannel(i));
      }
      Serial.println();
    }
  }

  // ------------------------------------------------------------
  // LX16A ARM BUS SERVO CONTROL (CH6, 3-position)
  // ------------------------------------------------------------
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
  // middle = hold

  // ------------------------------------------------------------
  // ROTATION PWM SERVO (CH11)
  // ------------------------------------------------------------
  float ch11Norm = channelNorm(11);
  int targetRotationAngle = (int)((ch11Norm + 1.0f) * 0.5f * 180.0f);
  targetRotationAngle = constrain(targetRotationAngle, ROTATION_MIN, ROTATION_MAX);

  static unsigned long lastRotUpdate = 0;
  if (millis() - lastRotUpdate > 15) { // Adjust 15ms higher for slower rotation
    lastRotUpdate = millis();
    if (rotationAngle < targetRotationAngle) rotationAngle++;
    else if (rotationAngle > targetRotationAngle) rotationAngle--;
    rotationServo.write(rotationAngle);
  }

  // ------------------------------------------------------------
  // BUCKET PWM SERVO
  // ------------------------------------------------------------
  // Still held at startup angle until you assign a channel to it
  bucketServo.write(bucketAngle);

  // ------------------------------------------------------------
  // DRIVE MOTORS
  // ------------------------------------------------------------
  float throttle = channelNorm(3);
  if (THROTTLE_INVERTED) throttle = -throttle;
  float steering = channelNorm(4);

  float left  = throttle + steering;
  float right = throttle - steering;

  left  = constrain(left,  -1.0f, 1.0f);
  right = constrain(right, -1.0f, 1.0f);

  driveMotor(LF_PIN, LB_PIN, left);
  driveMotor(RF_PIN, RB_PIN, right);

  // ------------------------------------------------------------
  // ARM LIFT ACTUATOR (CH7)
  // ------------------------------------------------------------
  int armLiftState = read3PosChannel(ARM_LIFT_CRSF_CH);
  driveActuator(ARM_UP_PIN, ARM_DOWN_PIN, armLiftState);

  // ------------------------------------------------------------
  // CLAW ACTUATOR (CH6)
  // ------------------------------------------------------------
  int rawClaw = crsf.getChannel(CLAW_CRSF_CH);
  int clawState = 1; // STOP
  if (rawClaw != 0) {
    if (rawClaw < SW_LOW_MAX) clawState = 2;         // Negative -> Close
    else if (rawClaw > SW_HIGH_MIN) clawState = 0;   // Positive -> Open
  }
  driveActuator(CLAW_OPEN_PIN, CLAW_CLOSE_PIN, clawState);

  // ------------------------------------------------------------
  // SERIAL STATUS
  // ------------------------------------------------------------
  static uint32_t last = 0;
  if (millis() - last > 250) {
    last = millis();

    const char* armLiftStr = (armLiftState == 0) ? "UP"
                             : (armLiftState == 2) ? "DOWN"
                                                   : "STOP";

    const char* clawStr = (clawState == 0) ? "OPEN"
                          : (clawState == 2) ? "CLOSE"
                                             : "STOP";

    const char* armBusStr = (rawArmBusSw < (int)SW_LOW_MAX) ? "LOW"
                            : (rawArmBusSw > (int)SW_HIGH_MIN) ? "HIGH"
                                                               : "MID";

    int busCh = crsf.getChannel(ARM_BUS_SWITCH_CRSF_CH);
    int liftCh = crsf.getChannel(ARM_LIFT_CRSF_CH);
    int clawCh = crsf.getChannel(CLAW_CRSF_CH);
    int rawCh3 = crsf.getChannel(3);
    int rawCh4 = crsf.getChannel(4);

    Serial.printf("[DRIVE] CH3raw=%4d T=%6.2f  CH4raw=%4d S=%6.2f  L=%6.2f R=%6.2f  |  [ARM_BUS ch%d]%-4s raw=%4d arm=%3d deg  [ROT]%3d deg [BUCKET]%3d deg  [ARM_LIFT]%s  [CLAW]%s\n",
                  rawCh3, throttle, rawCh4, steering, left, right,
                  ARM_BUS_SWITCH_CRSF_CH, armBusStr, rawArmBusSw, armBusAngle,
                  rotationAngle, bucketAngle,
                  armLiftStr, clawStr);
  }
}
