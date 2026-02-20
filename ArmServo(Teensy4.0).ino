#include <AlfredoCRSF.h>
#include <LX16A-bus.h> // Matches the library in your GitHub link

// --- 1. SETUP OBJECTS ---
AlfredoCRSF crsf;

// Constructor: LX16A(ID, SerialPort)
// We use ID 1 and Serial4 (Pins 18 & 19)
LX16A armServo(1, Serial4);

// --- 2. PIN DEFINITIONS ---
const int L_IN1 = 8;  const int L_IN2 = 9;  
const int R_IN1 = 14; const int R_IN2 = 15; 

void setup() {
  Serial.begin(115200);
  
  // Remote Setup
  Serial1.begin(CRSF_BAUDRATE); 
  crsf.begin(Serial1);

  // --- SERVO SETUP (Specific to alecxcode library) ---
  // 1. Start the hardware serial first
  Serial4.begin(115200); 
  
  // 2. Initialize the library logic
  armServo.initialize(); 
  
  // 3. Lock the servo and set to Angle Mode
  armServo.setServoMode(); 
  armServo.enableTorque(); 

  // --- MOTOR SETUP ---
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  
  // Silent Mode (20kHz)
  analogWriteFrequency(L_IN1, 20000);
  analogWriteFrequency(L_IN2, 20000);
  analogWriteFrequency(R_IN1, 20000);
  analogWriteFrequency(R_IN2, 20000);

  Serial.println("System Ready: LX16A-bus Active");
}

void driveMotors(float left, float right) {
  int lSpeed = constrain(abs(left) * 1023, 0, 1023);
  if (left > 0) { analogWrite(L_IN1, lSpeed); analogWrite(L_IN2, 0); }
  else          { analogWrite(L_IN1, 0); analogWrite(L_IN2, lSpeed); }

  int rSpeed = constrain(abs(right) * 1023, 0, 1023);
  if (right > 0) { analogWrite(R_IN1, rSpeed); analogWrite(R_IN2, 0); }
  else           { analogWrite(R_IN1, 0); analogWrite(R_IN2, rSpeed); }
}

void loop() {
  crsf.update(); 

  // --- WHEEL CONTROL ---
  float throttle = (crsf.getChannel(3) - 1500) / 500.0;
  float rawTurn  = (crsf.getChannel(1) - 1500) / 500.0;
  float turn = rawTurn * rawTurn * rawTurn; 
  
  driveMotors(throttle + turn, throttle - turn);

// --- EXCAVATOR ARM LOGIC ---
  int stickInput = crsf.getChannel(2); 

  // Map stick to Degrees (0 to 240 for this library)
  float targetAngle = map(stickInput, 1000, 2000, 0, 240);
  targetAngle = constrain(targetAngle, 0, 240);

  // THE FIX: Only give it the Angle and the Time (in milliseconds)
  armServo.move(targetAngle, 50);
  
  // --- DEBUG PRINTS ---
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.print("Turn (Ch1): "); Serial.print(crsf.getChannel(1));
    Serial.print(" | Drive (Ch3): "); Serial.print(crsf.getChannel(3));
    Serial.print(" | Arm (Ch2): "); Serial.println(crsf.getChannel(2));
  }
}
