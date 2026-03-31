#include <AlfredoCRSF.h>
#include "LX16A-bus.h"

AlfredoCRSF crsf;

// LX16A servo setup (replaces Servo.h usage)
const uint8_t SERVO_ID = 1;
LX16A servo1(SERVO_ID, Serial2);


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

  // LX16A servo init (NEW)
  Serial2.begin(115200);
  servo1.initialize(115200);
  servo1.enableTorque();
  servo1.setServoMode();

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

  // actuator control
  int sc = readSC();
  driveActuator(2,3,sc);

  // --- LX16A Servo control (CH4) ---
  float servoInput = channelNorm(4);

  // map -1..1 → 0..240 degrees (LX16A range)
  int servoAngle = (servoInput + 1.0f) * 120.0f;

  servo1.move(servoAngle, 30);


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


  // Debug
  static uint32_t last = 0;
  if (millis() - last > 200) {
    last = millis();
    Serial.printf("T=%.2f S=%.2f  L=%.2f R=%.2f\n",
                   throttle, steering, left, right);
  }
}