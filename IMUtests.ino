#include <SoftwareSerial.h>

SoftwareSerial mySerial(3, 2); // RX=3, TX=2

const long bauds[] = {4800, 9600, 19200, 38400, 57600, 115200};
const int numBauds = sizeof(bauds) / sizeof(bauds[0]);
int currentBaud = 0;
unsigned long lastSwitch = 0;
const unsigned long SWITCH_INTERVAL = 4000; // 4 sec per baud rate

void setup() {
  Serial.begin(115200);
  Serial.println("Baud auto-scan starting...");
  mySerial.begin(bauds[currentBaud]);
  Serial.print("\n--- Trying baud: ");
  Serial.println(bauds[currentBaud]);
  lastSwitch = millis();
}

void loop() {
  // Switch baud rate periodically
  if (millis() - lastSwitch > SWITCH_INTERVAL) {
    currentBaud = (currentBaud + 1) % numBauds;
    mySerial.end();
    mySerial.begin(bauds[currentBaud]);
    Serial.print("\n--- Trying baud: ");
    Serial.println(bauds[currentBaud]);
    lastSwitch = millis();
  }

  // Buffer bytes, print in a batch to reduce overhead
  static byte buf[16];
  static int idx = 0;

  while (mySerial.available() && idx < 16) {
    buf[idx++] = mySerial.read();
  }

  if (idx > 0) {
    for (int i = 0; i < idx; i++) {
      if (buf[i] < 16) Serial.print("0");
      Serial.print(buf[i], HEX);
      Serial.print(" ");
      if (buf[i] == 0x55) Serial.print("<-- HEADER? ");
    }
    idx = 0;
  }
}