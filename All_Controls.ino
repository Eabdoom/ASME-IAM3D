#include <AlfredoCRSF.h>
#include "LX16A-bus.h"

AlfredoCRSF crsf;

// LX16A servo setup
// servo1 (ID 1) — arm servo   — SB button (CH6) sweeps 0–240°
// servo2 (ID 2) — sweep servo — right stick L/R (CH1) maps to 0–40°
const uint8_t SERVO_ID  = 1;
const uint8_t SERVO2_ID = 2;
LX16A servo1(SERVO_ID,  Serial5);   // Serial5: TX=pin20, RX=pin21
LX16A servo2(SERVO2_ID, Serial5);

// --- Arm servo (servo1) state ---
// SB switch (CH6): up=move up, middle=stop, down=move down
const int   ARM_MIN  = 0;
const int   ARM_MAX  = 240;
const int   ARM_STEP = 2;         // degrees per step
const unsigned long ARM_MS = 20;  // ms between steps

int  armAngle = 0;                // start at 0 to match park position
unsigned long lastArmStep = 0;

// --- Servo2 state ---
const int SERVO2_MIN = 0;
const int SERVO2_MAX = 40;
int  lastServo2Angle = -1;        // track last sent angle to avoid flooding bus
unsigned long lastServo2Step = 0;
const unsigned long SERVO2_MS = 30;  // min ms between servo2 commands


// Normalize CRSF values into -1 … +1
float channelNorm(int ch) {
  uint16_t raw = crsf.getChannel(ch);  // typically 989–2012

  const float RAW_MIN = 989.0f;
  const float RAW_MAX = 2012.0f;
  const float RAW_CENTER = (RAW_MIN + RAW_MAX) / 2.0f;
  const float HALF_RANGE = (RAW_MAX - RAW_MIN) / 2.0f;

  float norm = (raw - RAW_CENTER) / HALF_RANGE;
  return constrain(norm, -1.0f, 1.0f);
}


// Drive an H-bridge motor using normalized RC input
void driveMotor(int in1, int in2, float norm) {
  float speed = fabs(norm);
  if (speed < 0.02f) speed = 0;

  uint16_t pwm = speed * 1023;

  if (norm >= 0) {
    analogWrite(in1, pwm);
    analogWrite(in2, 0);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, pwm);
  }
}


// -------- ACTUATOR CODE --------

int readSC() {
  uint16_t raw = crsf.getChannel(7);

  if (raw < 1200) return 0;
  if (raw < 1700) return 1;
  return 2;
}

void driveActuator(int in1, int in2, int state)
{
  if(state == 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if(state == 2) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

// ------------------------------------


void setup() {

  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println("Teensy 4.1 + CRSF + H-Bridge Motor Control");

  Serial1.begin(CRSF_BAUDRATE);
  crsf.begin(Serial1);

  // LX16A servos on shared Serial5 half-duplex bus (TX=pin20, RX=pin21)
  Serial5.begin(115200);

  // servo1 — arm servo (ID 1)
  servo1.initialize(115200);
  servo1.enableTorque();
  servo1.setServoMode();
  servo1.move(ARM_MIN, 100);    // park at 0° on startup

  // servo2 — right stick servo (ID 2)
  servo2.initialize(115200);
  servo2.enableTorque();
  servo2.setServoMode();
  servo2.move(SERVO2_MIN, 100);  // park at 0° on startup

  analogWriteResolution(10);

  analogWriteFrequency(8, 20000);
  analogWriteFrequency(10, 20000);
  analogWriteFrequency(14, 20000);
  analogWriteFrequency(15, 20000);

  // actuator pins
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
}


void loop() {

  crsf.update();

  // --- Serial debug command: type 'H' to test arm servo (70 ↔ 90°) ---
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'H' || cmd == 'h') {
      static bool toggled = false;
      int testAngle = toggled ? 70 : 90;
      toggled = !toggled;
      servo1.move(testAngle, 500);
      armAngle = testAngle;
      Serial.printf("[DEBUG] H received — moving arm servo to %d°\n", testAngle);
    }
  }

  // actuator control
  int sc = readSC();
  driveActuator(2,3,sc);

  // --- SB Switch (CH6) — arm servo (servo1) up/stop/down ---
  uint16_t rawCH6 = crsf.getChannel(6);

  if (rawCH6 < 1200) {
    // Switch DOWN — move arm down
    unsigned long now = millis();
    if (now - lastArmStep >= ARM_MS) {
      lastArmStep = now;
      armAngle -= ARM_STEP;
      armAngle = constrain(armAngle, ARM_MIN, ARM_MAX);
      servo1.move(armAngle, (uint16_t)ARM_MS);
    }
  } else if (rawCH6 > 1700) {
    // Switch UP — move arm up
    unsigned long now = millis();
    if (now - lastArmStep >= ARM_MS) {
      lastArmStep = now;
      armAngle += ARM_STEP;
      armAngle = constrain(armAngle, ARM_MIN, ARM_MAX);
      servo1.move(armAngle, (uint16_t)ARM_MS);
    }
  }
  // Switch MIDDLE (1200–1700) — hold position, send nothing

  // --- servo2: right stick L/R (CH1) → position 0–40° ---
  // Rate-limited to avoid flooding the shared Serial4 bus
  unsigned long nowS2 = millis();
  float ch1Norm   = channelNorm(1);
  int servo2Angle = (int)((ch1Norm + 1.0f) * 0.5f * SERVO2_MAX);
  servo2Angle     = constrain(servo2Angle, SERVO2_MIN, SERVO2_MAX);
  if (nowS2 - lastServo2Step >= SERVO2_MS) {
    lastServo2Step = nowS2;
    servo2.move(servo2Angle, 30);
    lastServo2Angle = servo2Angle;
  }

  // --- Read stick inputs ---
  float throttle = channelNorm(2);   // forward/back stick
  float steering = channelNorm(1);   // left/right stick


  // --- Tank mixing ---
  float left  = throttle + steering;
  float right = throttle - steering;

  left  = constrain(left,  -1.0f, 1.0f);
  right = constrain(right, -1.0f, 1.0f);


  // --- Drive motors ---
  driveMotor(8, 10, left);
  driveMotor(14, 15, right);


  // --- Serial Monitor status (every 250ms) ---
  static uint32_t last = 0;
  if (millis() - last > 250) {
    last = millis();

    // Actuator state
    const char* actStr = (sc == 0) ? "EXTEND" : (sc == 2) ? "RETRACT" : "STOP";

    // SB switch state
    const char* sbStr  = (rawCH6 < 1200) ? "DOWN" : (rawCH6 > 1700) ? "UP" : "MID";

    Serial.printf("[ACT]%-7s [SB]%-4s arm=%3d°  [SV2]%2d°  [DRIVE] T=%.2f S=%.2f L=%.2f R=%.2f\n",
                  actStr, sbStr, armAngle,
                  servo2Angle,
                  throttle, steering, left, right);
  }
}