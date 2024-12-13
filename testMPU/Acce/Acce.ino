#include <MPU6050.h>
#include <Wire.h>
#include <math.h>

MPU6050 mpu;
void setup() {
  Serial.begin(9600);
  mpu.initialize();

}

void loop() {

}
