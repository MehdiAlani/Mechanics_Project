#include <Wire.h>
#include <MPU6050.h>
#include <Arduino.h>


MPU6050 mpu;

double acce_x,acce_y,acce_z;
double pitch_me,roll_me;
double pitch_world,roll_world;
bool verify_maths = true;
#define ACCEL_OFFSET_X 0
#define ACCEL_OFFSET_Y 0
#define ACCEL_OFFSET_Z 0

#define EPSELON 0.01
void accel_update(){
  acce_x = mpu.getAccelerationX()  / 8192.0 + ACCEL_OFFSET_X;
  acce_y = mpu.getAccelerationY() / 8192.0 + ACCEL_OFFSET_Y;
  acce_z = mpu.getAccelerationZ() / 8192.0 + ACCEL_OFFSET_Z;

  pitch_world = atan(-acce_y / sqrt(acce_x * acce_x + acce_z * acce_z)) * 180.0 / PI;
  roll_world = atan(acce_x / sqrt(acce_y * acce_y + acce_z * acce_z)) * 180.0 / PI;
  pitch_me = atan(acce_z / acce_y) * 180.0 / PI;
  roll_me = atan(acce_z / acce_x) * 180.0 / PI;
}
void print_accel_linear(){
  Serial.print("acce_x: ");
  Serial.print(acce_x);
  Serial.print(" acce_y: ");
  Serial.print(acce_y);
  Serial.print(" acce_z: ");
  Serial.println(acce_z);
}

void print_accel_angular(){
  Serial.print("pitch: ");
  Serial.print(pitch_me);
  Serial.print(" pitch_world: ");
  Serial.print(pitch_world);
  Serial.print("\t\troll_me: ");
  Serial.print(roll_me);
  Serial.print(" roll_world: ");
  Serial.println(roll_world);
}
void print_differience(){
  /*


  verify_maths = abs(pitch_world- pitch_me) >= EPSELON && abs(roll_world - roll_me) >= EPSELON;
  if(verify_maths == false){
    print_accel_angular();
    while(1);
  }


  */
  Serial.print(pitch_world + pitch_me);
  Serial.print("\t\t");
  Serial.println(roll_world + roll_me);
}


void setup() {

  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  if(!mpu.testConnection()){
    Serial.println("Not Connected ");
    while(1);
  }
  else{
    Serial.println("Connected ");
  }
}
void loop(){
  accel_update();
  print_differience();
  delay(1);
}
