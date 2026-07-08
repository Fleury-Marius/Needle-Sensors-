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