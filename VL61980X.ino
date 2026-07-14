// Makerguides. (n.d.). VL6180X distance sensor with Arduino from 
//https://www.makerguides.com/vl6180x-distance-sensor-with-arduino/
//Designed for running the distance sensor VL6180X
#include "Wire.h"
#include "VL6180X.h"

VL6180X sensor;

void setup() {
  Serial.begin(9600);
  Wire.begin();  
  sensor.init();
  sensor.configureDefault();
  sensor.setTimeout(500);
}

void loop() { 
  Serial.println(sensor.readRangeSingleMillimeters()); 
  delay(1000);
}
