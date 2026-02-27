#include <AlfredoCRSF.h>
#include <SoftwareSerial.h>

// RX on Pin 2, TX on Pin 4
SoftwareSerial mySerial(2, 4); 
AlfredoCRSF crsf;

float kb_n1 = 0; 
float kb_n2 = 0; 

float channelNorm(int ch) {
  uint16_t raw = crsf.getChannel(ch);
  const float RAW_MIN = 989.0;
  const float RAW_MAX = 2012.0;
  const float RAW_CENTER = 1500.5;
  const float HALF_RANGE = 511.5;

  float norm = (raw - RAW_CENTER) / HALF_RANGE;
  return constrain(norm, -1.0, 1.0);
}

void driveMotor(int in1, int in2, float norm) {
  float speed = fabs(norm);
  if (speed < 0.05) speed = 0; 

  // CHANGED FOR R3: PWM is 0-255 (8-bit)
  uint8_t pwm = (uint8_t)(speed * 255);

  if (norm >= 0) {
    analogWrite(in1, pwm);
    analogWrite(in2, 0);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, pwm);
  }
}

void setup() {
  Serial.begin(115200);
  
  // Note: SoftwareSerial on R3 struggles above 57600 baud.
  // Ensure your CRSF receiver is not set to 400k baud!
  mySerial.begin(57600); 
  crsf.begin(mySerial);

  // analogWriteResolution(10) REMOVED - Not supported on R3

  pinMode(11, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(3, OUTPUT);

  Serial.println(F("Uno R3 Ready."));
  Serial.println(F("Controls: w,a,s,d,x + Enter"));
}

void loop() {
  crsf.update();

  if (Serial.available() > 0) {
    char key = Serial.read();
    switch (key) {
      case 'w': kb_n1 = 0.8;  kb_n2 = 0.8;  break;
      case 's': kb_n1 = -0.8; kb_n2 = -0.8; break;
      case 'a': kb_n1 = -0.5; kb_n2 = 0.5;  break;
      case 'd': kb_n1 = 0.5;  kb_n2 = -0.5; break;
      case 'x': kb_n1 = 0;    kb_n2 = 0;    break;
    }
  }

  float final_n1 = 0;
  float final_n2 = 0;

  if (crsf.isLinkUp()) {
    final_n1 = channelNorm(1);
    final_n2 = channelNorm(2);
  } else {
    final_n1 = kb_n1;
    final_n2 = kb_n2;
  }

  driveMotor(11, 9, final_n1); 
  driveMotor(5, 3, final_n2);  

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.print(crsf.isLinkUp() ? "REMOTE | " : "KEYBOARD | ");
    Serial.print("M1: "); Serial.print(final_n1);
    Serial.print(" M2: "); Serial.println(final_n2);
  }
}
