
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

int16_t ax, ay, az; 
int16_t gx, gy, gz;  
double gyroPitch;
double gyroYaw;
double angle;


#define OFFSET_X 1.28
#define OFFSET_Z 0.51

void get_mpu_values(){
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  gyroPitch = gx / 131.0 + OFFSET_X;     
  gyroYaw = gz / 131.0 + OFFSET_Z;
}

#define MAX_SAMPLE 99999999
void determine_OFFSET(){
  Serial.print("Start Sampling The Max is ");
  Serial.println(MAX_SAMPLE);
  double avrage_pitch = 0;
  double avrage_yaw = 0;
  unsigned long count = 0; 
  while(count < MAX_SAMPLE && Serial.available() == 0){
    get_mpu_values();
    avrage_pitch += gyroPitch;
    avrage_yaw += gyroYaw;
    count++;
    delay(10);
  }
  avrage_pitch = avrage_pitch / count;
  avrage_yaw = avrage_yaw / count;

  Serial.println();
  Serial.print("OFFSET_X: ");
  Serial.print(avrage_pitch);
  Serial.print(" OFFSET_Z: ");
  Serial.println(avrage_yaw);
}


#define STEP_TIME_US 10
#define PRINT_TIME_MS 500
void get_angle_with_gyro(){
  Serial.println("Integration Begins :)))))");
  double angle_x = 0;
  double angle_z = 0;
  unsigned long startTime_ms = millis();
  unsigned long startTime_us = micros();
  int sample;
  while(Serial.available() == 0){
    if(micros() - startTime_us >= STEP_TIME_US){
      get_mpu_values();
      angle_x += (gyroPitch / 1000.0) * ((micros() - startTime_us) / 1000.0);
      angle_z += (gyroYaw / 1000.0) * ((micros() - startTime_us) / 1000.0);
      if(angle_x > 180) angle_x = angle_x - 180;
      else if(angle_x < 0) angle_x = 180 + angle_x; 
      if(angle_z >= 360) angle_z = angle_z - 360;
      else if(angle_z < 0) angle_z = 360 + angle_z;
      startTime_us = micros();
    }
    if(millis() - startTime_ms >= PRINT_TIME_MS){
      Serial.print("angle_x: ");
      Serial.print(angle_x);
      Serial.print(" speed_x: ");
      Serial.print(gyroPitch);
      Serial.print(" angle_z: ");
      Serial.print(angle_z);
      Serial.print(" speed_z: ");
      Serial.println(gyroYaw);
      startTime_ms = millis();
    }
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed.");
    while (1);
  }
  Serial.println("MPU6050 connection successful.");
  //determine_OFFSET();
  get_angle_with_gyro();
}

void loop() {

  get_mpu_values();
  Serial.print("speed_x: ");
  Serial.print(gyroPitch);
  Serial.print(" speed_z: ");
  Serial.println(gyroYaw);
  delay(300);


}
