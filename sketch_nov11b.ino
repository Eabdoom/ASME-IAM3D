#include <AlfredoCRSF.h>

AlfredoCRSF crsf;

// pick the serial port connected to your ELRS receiver
// for Arduino Mega, Leonardo, or similar, use Serial1.
// if you only have Serial (e.g. Uno), it probably won’t work well at 420000 baud.
#define CRSF_PORT Serial1

void setup() {
  Serial.begin(115200);   // USB serial for debugging
  CRSF_PORT.begin(420000);
  crsf.begin(&CRSF_PORT);
  Serial.println("Starting AlfredoCRSF test...");
}

void loop() {
  // keep feeding data to the library
  crsf.update();

  // if any packet was received, print a message
  if (crsf.packetReceived()) {       // function name may vary; check examples
    Serial.println("Got a CRSF packet!");
  }

  // small delay so we don't flood the serial monitor
  delay(10);
}
