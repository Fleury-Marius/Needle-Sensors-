#include <Wire.h>
#include "VL6180X.h"

/////////////////////////////////
/*
GY95T-----mini            VL6180X
VCC----VCC                VCC----VCC (or 3V3, check your board)
SCL----A5                 SCL----A5 (shared I2C bus)
SDA----A4                 SDA----A4 (shared I2C bus)
INT----2
PS-----GND
GND--GND                  GND----GND

Both sensors share the same I2C bus (A4/A5 on Uno/Nano),
so only ONE Wire.begin() call is needed.
*/
/////////////////////////////////

#define iic_add  0xa4 >> 1

// Struct order MUST match the register order 0x08 -> 0x22 exactly,
// since we memcpy() the raw I2C bytes straight into it (L byte then H byte
// for every 16-bit value, per the datasheet).
typedef struct __attribute__((packed))
{
    int16_t acc_x;   // 0x08/0x09
    int16_t acc_y;   // 0x0A/0x0B
    int16_t acc_z;   // 0x0C/0x0D

    int16_t gyro_x;  // 0x0E/0x0F
    int16_t gyro_y;  // 0x10/0x11
    int16_t gyro_z;  // 0x12/0x13

    int16_t roll;    // 0x14/0x15
    int16_t pitch;   // 0x16/0x17
    int16_t yaw;     // 0x18/0x19

    uint8_t leve;    // 0x1A

    int16_t temp;    // 0x1B/0x1C

    int16_t mag_x;   // 0x1D/0x1E
    int16_t mag_y;   // 0x1F/0x20
    int16_t mag_z;   // 0x21/0x22
} gy;

unsigned char Re_buf;
unsigned char sign = 0;
gy my_95Q;
unsigned int delay_t = 0;
byte ready_Ok = 0;

VL6180X sensor;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    delay(100);

    // --- GY95T setup ---
    byte td = 1;
    iic_read(0x02, &td, 1);
    switch (td)
    {
        case 0: delay_t = 100; break;
        case 1: delay_t = 20;  break;
        case 2: delay_t = 10;  break;
        case 3: delay_t = 5;   break;
    }
    delay(delay_t);
    attachInterrupt(0, Exti, RISING);

    // --- VL6180X setup ---
    sensor.init();
    sensor.configureDefault();
    sensor.setTimeout(500);
}

void loop() {
    unsigned char data[27] = {0};

    // --- Read GY95T (accel/gyro/mag) ---
    iic_read(0x08, data, 27);
    memcpy(&my_95Q, data, 27);

    Serial.print("acc_x: ");  Serial.print(my_95Q.acc_x);
    Serial.print(",acc_y: "); Serial.print(my_95Q.acc_y);
    Serial.print(",acc_z: "); Serial.print(my_95Q.acc_z);

    Serial.print(",roll: ");  Serial.print(my_95Q.roll / 100.0);
    Serial.print(",pitch: "); Serial.print(my_95Q.pitch / 100.0);
    Serial.print(",yaw: ");   Serial.print(my_95Q.yaw / 100.0);

    Serial.print(",temp: ");  Serial.print(my_95Q.temp / 100.0);
    Serial.print(",leve: ");  Serial.print((float)my_95Q.leve);

    Serial.print(",mag_x: "); Serial.print(my_95Q.mag_x);
    Serial.print(",mag_y: "); Serial.print(my_95Q.mag_y);
    Serial.print(",mag_z: "); Serial.print(my_95Q.mag_z);

    ready_Ok = 0;

    // --- Read VL6180X (distance) ---
    Serial.print(",distance_mm: ");
    Serial.println(sensor.readRangeSingleMillimeters());

    delay(delay_t);
}

void Exti()
{
    if (!ready_Ok)
        ready_Ok = 1; // data-update flag
}

void iic_read(unsigned char add, unsigned char *data, unsigned char len)
{
    byte j = 0;
    Wire.beginTransmission(iic_add);
    Wire.write(add);
    Wire.endTransmission(false);
    Wire.requestFrom(iic_add, (int)len);
    for (j = 0; j < len; j++)
        *data++ = Wire.read();
}

void iic_write(char add, unsigned char data)
{
    Wire.beginTransmission(iic_add);
    Wire.write((uint8_t)add);
    Wire.write((uint8_t)data);
    Wire.endTransmission();
}
