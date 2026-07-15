#include <Wire.h>
#include "VL6180X.h"

//////////////////// GY-95T ////////////////////
#define uint16_t unsigned int
#define iic_add  0xA4 >> 1

typedef struct
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    uint8_t leve;
    int16_t temp;
} gy;

gy my_95Q;
uint16_t delay_t = 20;
byte ready_Ok = 0;

//////////////////// VL6180X ////////////////////
VL6180X sensor;

//////////////////// SETUP ////////////////////
void setup() {
  Serial.begin(9600);
  Wire.begin();

  delay(100);

  // ---- GY-95T INIT ----
  byte td = 1;
  iic_read(0x02, &td, 1);

  switch (td) {
    case 0: delay_t = 100; break;
    case 1: delay_t = 20; break;
    case 2: delay_t = 10; break;
    case 3: delay_t = 5; break;
  }

  delay(delay_t);
  attachInterrupt(0, Exti, RISING); // INT pin → D2

  // ---- VL6180X INIT ----
  sensor.init();
  sensor.configureDefault();
  sensor.setTimeout(500);

  Serial.println("System Ready (GY-95T + VL6180X)");
}

//////////////////// LOOP ////////////////////
void loop() {
  unsigned char data[16] = {0};

  // ---- Read GY-95T ----
  iic_read(0x14, data, 9);
  memcpy(&my_95Q, data, 9);

  Serial.print("Roll: ");
  Serial.print(my_95Q.roll / 100.0);
  Serial.print(" | Pitch: ");
  Serial.print(my_95Q.pitch / 100.0);
  Serial.print(" | Yaw: ");
  Serial.print(my_95Q.yaw / 100.0);

  // ---- Read Distance ----
  int distance = sensor.readRangeSingleMillimeters();

  Serial.print(" | Distance (mm): ");
  Serial.print(distance);

  if (sensor.timeoutOccurred()) {
    Serial.print(" (TIMEOUT)");
  }

  Serial.println();

  delay(200);
}

//////////////////// INTERRUPT ////////////////////
void Exti() {
  if (!ready_Ok)
    ready_Ok = 1;
}

//////////////////// I2C FUNCTIONS ////////////////////
void iic_read(unsigned char add, unsigned char *data, unsigned char len) {
  Wire.beginTransmission(iic_add);
  Wire.write(add);
  Wire.endTransmission(false);
  Wire.requestFrom(iic_add, (int)len);

  for (byte j = 0; j < len; j++) {
    *data++ = Wire.read();
  }
}

void iic_write(char add, unsigned char data) {
  Wire.beginTransmission(iic_add);
  Wire.write((uint8_t)add);
  Wire.write((uint8_t)data);
  Wire.endTransmission();
}