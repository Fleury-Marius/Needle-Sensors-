#include <Wire.h>

/////////////////////////////////
/*
GY95T-----mini
VCC----VCC
SCL----A5
SDA----A4
INT----2
PS-----GND
GND--GND
*/
/////////////////////////////////

//////////////////////////////////
#define uint16_t unsigned int
#define iic_add  0xa4>>1
typedef struct
{
    
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    uint8_t leve;
    int16_t temp;
   
} gy;
unsigned char Re_buf;
unsigned char sign=0;
gy my_95Q;
uint16_t delay_t=0;
byte ready_Ok=0;
void setup() {
      byte td=1;
       Serial.begin(9600);
       Wire.begin();
       delay(100); 
       iic_read(0x02,&td,1);
       switch(td)
       {
         case 0:delay_t=100;break;
         case 1:delay_t=20;break;
         case 2:delay_t=10;break;
         case 3:delay_t=5;break;
       }
        delay(delay_t); 
        attachInterrupt(0, Exti, RISING);
}
void loop() {
  unsigned char data[16]={0};

 
     
   //   if(ready_Ok)
      {
   iic_read(0x14,data,9);
   memcpy(&my_95Q,data,9);
    Serial.print("roll: ");
   Serial.print(my_95Q.roll/100);
   Serial.print(",pitch: ");
   Serial.print( my_95Q.pitch/100);
   Serial.print(",yaw:");
   Serial.print( my_95Q.yaw/100);
   Serial.print(",temp:");
   Serial.print(my_95Q.temp/100);
   Serial.print(",leve:");
   Serial.println((float)my_95Q.leve);
    ready_Ok=0;
     }
 
   delay(delay_t); 
}
void Exti()
{
 if(!ready_Ok) 
  ready_Ok=1;//数据更新标志
}
void iic_read(unsigned char add,unsigned char *data,unsigned char len)
{
  byte j=0;
   Wire.beginTransmission(iic_add);
   Wire.write(add);
   Wire.endTransmission(false); 
   Wire.requestFrom(iic_add,(int)len);
   for(j=0;j<len;j++)
   *data++=Wire.read();
}
void iic_write( char add,unsigned char data)
{
  Wire.beginTransmission(iic_add);
  Wire.write((uint8_t)add);
  Wire.write((uint8_t)data);
  Wire.endTransmission();
}



