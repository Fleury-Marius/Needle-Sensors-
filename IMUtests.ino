#include <SoftwareSerial.h>

SoftwareSerial imuSerial(10, 11);

#define PACKET_SIZE 32
#define HEADER_1 0xA4
#define HEADER_2 0x03

byte buffer[PACKET_SIZE];
int idx = 0;

enum State { WAIT_H1, WAIT_H2, COLLECT };
State state = WAIT_H1;

// --- TENTATIVE offsets, verify these against real sensor motion! ---
const int OFFSET_ACC_X = 4;
const int OFFSET_ACC_Y = 6;
const int OFFSET_ACC_Z = 8;
const int OFFSET_GYRO_X = 10;
const int OFFSET_GYRO_Y = 12;
const int OFFSET_GYRO_Z = 14;
const int OFFSET_ROLL   = 22;
const int OFFSET_PITCH  = 24;
const int OFFSET_YAW    = 26;

void setup() {
  Serial.begin(115200);
  imuSerial.begin(115200);
  Serial.println("GY-95T PACKET READER - axis decode (UNVERIFIED offsets)");
}

int16_t readInt16(int offset) {
  return (int16_t)(buffer[offset] | (buffer[offset + 1] << 8));
}

void loop() {
  while (imuSerial.available()) {
    byte b = imuSerial.read();

    switch (state) {
      case WAIT_H1:
        if (b == HEADER_1) { buffer[0] = b; idx = 1; state = WAIT_H2; }
        break;
      case WAIT_H2:
        if (b == HEADER_2) { buffer[1] = b; idx = 2; state = COLLECT; }
        else if (b == HEADER_1) { buffer[0] = b; idx = 1; }
        else state = WAIT_H1;
        break;
      case COLLECT:
        buffer[idx++] = b;
        if (idx >= PACKET_SIZE) {
          printAxes();
          state = WAIT_H1;
        }
        break;
    }
  }
}

void printAxes() {
  int16_t accX = readInt16(OFFSET_ACC_X);
  int16_t accY = readInt16(OFFSET_ACC_Y);
  int16_t accZ = readInt16(OFFSET_ACC_Z);

  int16_t gyroX = readInt16(OFFSET_GYRO_X);
  int16_t gyroY = readInt16(OFFSET_GYRO_Y);
  int16_t gyroZ = readInt16(OFFSET_GYRO_Z);

  int16_t roll  = readInt16(OFFSET_ROLL);
  int16_t pitch = readInt16(OFFSET_PITCH);
  int16_t yaw   = readInt16(OFFSET_YAW);

  Serial.print("Acc  X: "); Serial.print(accX);
  Serial.print("  Y: "); Serial.print(accY);
  Serial.print("  Z: "); Serial.println(accZ);

  Serial.print("Gyro X: "); Serial.print(gyroX);
  Serial.print("  Y: "); Serial.print(gyroY);
  Serial.print("  Z: "); Serial.println(gyroZ);

  Serial.print("Roll: "); Serial.print(roll);
  Serial.print("  Pitch: "); Serial.print(pitch);
  Serial.print("  Yaw: "); Serial.println(yaw);
  Serial.println("---");
}
