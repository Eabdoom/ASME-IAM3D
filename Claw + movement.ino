#include <AlfredoCRSF.h>

AlfredoCRSF crsf;

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


// -------- ADDED ACTUATOR CODE --------

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

  Serial.println("Teensy 4.0 + CRSF + H-Bridge Motor Control");

  Serial1.begin(CRSF_BAUDRATE);
  crsf.begin(Serial1);

  analogWriteResolution(10);

  analogWriteFrequency(8, 20000);
  analogWriteFrequency(9, 20000);
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

  // --- Read stick inputs ---
  float throttle = channelNorm(2);   // forward/back stick
  float steering = channelNorm(1);   // left/right stick

  // --- Tank mixing ---
  float left  = throttle + steering;
  float right = throttle - steering;

  left  = constrain(left,  -1.0f, 1.0f);
  right = constrain(right, -1.0f, 1.0f);

  // --- Drive motors ---
  // Motor A
  driveMotor(8, 9, left);

  // Motor B
  driveMotor(14, 15, right);

  // Debug
  static uint32_t last = 0;
  if (millis() - last > 200) {
    last = millis();
    Serial.printf("T=%.2f S=%.2f  L=%.2f R=%.2f\n",
                   throttle, steering, left, right);
  }

}
