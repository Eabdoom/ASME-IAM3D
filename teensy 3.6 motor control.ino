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
  float speed = fabs(norm);      // 0…1
  if (speed < 0.02f) speed = 0;  // deadband

  uint16_t pwm = speed * 1023;   // 10-bit PWM (0–1023)

  if (norm >= 0) {
    // Forward
    analogWrite(in1, pwm);
    analogWrite(in2, 0);
  } else {
    // Reverse
    analogWrite(in1, 0);
    analogWrite(in2, pwm);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println("Teensy 4.0 + CRSF + H-Bridge Motor Control");

  Serial1.begin(CRSF_BAUDRATE);
  crsf.begin(Serial1);

  // PWM setup
  analogWriteResolution(10);  // 0–1023 resolution

  // 20 kHz PWM on all motor pins
  analogWriteFrequency(8, 20000);
  analogWriteFrequency(9, 20000);
  analogWriteFrequency(14, 20000);
  analogWriteFrequency(15, 20000);
}

void loop() {
  crsf.update();

  // Read normalized RC channels (−1…+1)
  float n1 = channelNorm(1);
  float n2 = channelNorm(2);

  // --- Control motors ---
  // Motor A on pins 8 (IN1), 9 (IN2)
  driveMotor(8, 9, n1);

  // Motor B on pins 10 (IN1), 11 (IN2)
  driveMotor(14, 15, n2);

  // Debug (optional)
  static uint32_t last = 0;
  if (millis() - last > 200) {
    last = millis();
    Serial.printf("CH1=%.2f  CH2=%.2f\n",n1,n2);
}
}
