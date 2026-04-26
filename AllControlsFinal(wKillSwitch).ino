#include <AlfredoCRSF.h>
#include "LX16A-bus.h"
#include <Servo.h>
#include <math.h>

AlfredoCRSF crsf;

// ============================================================
// PIN MAP
// ============================================================

// LX16A arm bus servo
const uint8_t ARM_BUS_SERVO_ID = 1;
LX16A armBusServo(ARM_BUS_SERVO_ID, Serial5);

// PWM servos
const int ROTATION_SERVO_PIN = 11;
const int BUCKET_SERVO_PIN   = 12;

// Drive motor driver pins
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
const uint8_t ARM_DISARM_CRSF_CH     = 5;   // Moved Arm/Disarm Kill Switch to CH5
const uint8_t ARM_BUS_SWITCH_CRSF_CH = 6;
const uint8_t CLAW_CRSF_CH           = 7;
const uint8_t BUCKET_CRSF_CH         = 8;

// Flip these if any directions are reversed
const bool THROTTLE_INVERTED = false;
const bool STEERING_INVERTED = true;
const bool ARM_LIFT_INVERTED = false;

const uint16_t SW_LOW_MAX  = 1300;
const uint16_t SW_HIGH_MIN = 1700;

bool isArmed = false; // Tracks if the robot is allowed to move

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
bool bucketEnabled         = false;   

const int ROTATION_MIN = 0;
const int ROTATION_MAX = 180;
int rotationAngle = 90;

const int BUCKET_MIN       = 0;
const int BUCKET_MAX       = 180;
const int BUCKET_START_POS = 165;  

const int BUCKET_DROP_POSITIONS[10] = {
  150, 143, 137, 130, 123,
  117, 110, 103,  97,  90
};

int bucketProfileIndex = 0; // Defaulting to the max drop (150 degrees)
int bucketAngle = BUCKET_START_POS;


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

  if (fabs(norm) < 0.08f) return 0.0f;
  return norm;
}

int read3PosChannel(uint8_t ch) {
  uint16_t raw = crsf.getChannel(ch);
  if (raw == 0) return 0;
  if (raw < SW_LOW_MAX)  return -1;
  if (raw > SW_HIGH_MIN) return 1;
  return 0;
}

int readSignedAxisChannel(uint8_t ch, bool invert = false) {
  float norm = channelNorm(ch);
  if (invert) norm = -norm;

  if (norm > 0.2f)  return 1;
  if (norm < -0.2f) return -1;
  return 0;
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

// ------------------------------------------------------------
// FAILSAFE: Instantly kills all heavy movement
// ------------------------------------------------------------
void stopEverything() {
  driveMotor(LF_PIN, LB_PIN, 0.0f);
  driveMotor(RF_PIN, RB_PIN, 0.0f);
  driveActuator(ARM_UP_PIN, ARM_DOWN_PIN, 0);
  driveActuator(CLAW_OPEN_PIN, CLAW_CLOSE_PIN, 0);
  // Note: Servos are left where they are to hold position, 
  // rather than dropping a heavy load instantly.
}


// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println("Teensy 4.1 + CRSF main control (ARM/DISARM ENABLED on CH5)");

  pinMode(LF_PIN, OUTPUT);
  pinMode(RF_PIN, OUTPUT);
  pinMode(LB_PIN, OUTPUT);
  pinMode(RB_PIN, OUTPUT);

  pinMode(ARM_UP_PIN, OUTPUT);
  pinMode(ARM_DOWN_PIN, OUTPUT);

  pinMode(CLAW_OPEN_PIN, OUTPUT);
  pinMode(CLAW_CLOSE_PIN, OUTPUT);

  // Start safe
  stopEverything();

  analogWriteResolution(10);
  analogWriteFrequency(LF_PIN, 20000);
  analogWriteFrequency(RF_PIN, 20000);
  analogWriteFrequency(LB_PIN, 20000);
  analogWriteFrequency(RB_PIN, 20000);

  Serial1.begin(CRSF_BAUDRATE);
  crsf.begin(Serial1);

  Serial5.begin(115200);
  armBusServo.initialize(115200);
  armBusServo.enableTorque();
  armBusServo.setServoMode();

  ensureRotationServoAttached();
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  crsf.update();

  // ------------------------------------------------------------
  // FAILSAFE CHECK (Channel 5)
  // ------------------------------------------------------------
  uint16_t rawArmSwitch = crsf.getChannel(ARM_DISARM_CRSF_CH);
  
  // If the switch is flipped up (> 1500), we are ARMED.
  // If the switch is down (< 1500) OR the controller disconnects (raw == 0), DISARM.
  isArmed = (rawArmSwitch > 1500);

  if (!isArmed) {
    stopEverything(); // Cut power to all heavy hardware

    // Only print disarm status every 1 second to avoid flooding monitor
    static unsigned long lastDisarmPrint = 0;
    if (millis() - lastDisarmPrint > 1000) {
      Serial.println("[WARNING] ROBOT DISARMED - Flipping CH5 Switch UP to arm.");
      lastDisarmPrint = millis();
    }
    return; // SKIP THE REST OF THE LOOP until armed!
  }

  // ============================================================
  // ALL CODE BELOW ONLY RUNS WHEN ROBOT IS ARMED
  // ============================================================

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

  // Standard Tank Mixing
  float throttle = channelNorm(THROTTLE_CRSF_CH);
  if (THROTTLE_INVERTED) throttle = -throttle;

  float steering = channelNorm(STEERING_CRSF_CH);
  if (STEERING_INVERTED) steering = -steering;

  left  = throttle + steering;
  right = throttle - steering;

  left  = constrain(left,  -1.0f, 1.0f);
  right = constrain(right, -1.0f, 1.0f);

  driveMotor(LF_PIN, LB_PIN, left);
  driveMotor(RF_PIN, RB_PIN, right);

  int armLiftState = readSignedAxisChannel(ARM_LIFT_CRSF_CH, ARM_LIFT_INVERTED);
  driveActuator(ARM_UP_PIN, ARM_DOWN_PIN, armLiftState);

  int clawState = read3PosChannel(CLAW_CRSF_CH);
  driveActuator(CLAW_OPEN_PIN, CLAW_CLOSE_PIN, clawState);
}
