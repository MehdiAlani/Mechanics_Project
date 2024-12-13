#include <Wire.h>
#include <Arduino.h>
#include <math.h>

#define MPU_ADDR 0x68
#define CONF_FILTER 0x1A  


#define ACCE_ADDR 0x3B
#define GYRO_ADDR 0x43
#define TEMP_ADDR 0x41 


#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define CONF_GYRO 0x1B 
#define CONF_ACCE 0x1C
#define FIFO_ENABLE 0x23 


int temp,rgyro_x,rgyro_y,rgyro_z,racce_x,racce_y,racce_z;


void config_mpu(uint8_t config,uint8_t v){
    if (config <= 3){
        Wire.beginTransmission(MPU_ADDR);
        switch(v){
            case 0:
                Wire.write(CONF_GYRO);
                break;
            case 1:
                Wire.write(CONF_ACCE);
                break;
            default:
                break;
        }
        Wire.write(config << 3);
        Wire.endTransmission();
    }
    return;
}

uint16_t recivie_mpu_byte(uint8_t addr){
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(addr);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR,2);
    uint16_t lowbyte;
    uint16_t highbyte;
    if(Wire.available() == 2){
        highbyte = Wire.read();
        lowbyte = Wire.read();
        Wire.endTransmission();
        return (highbyte << 8) | lowbyte;
    }
    
    return 1;
}
void load_all_mpu(){
    temp = recivie_mpu_byte(TEMP_ADDR);
    rgyro_x = recivie_mpu_byte(GYRO_ADDR);
    rgyro_y = recivie_mpu_byte(GYRO_ADDR + 1);
    rgyro_z = recivie_mpu_byte(GYRO_ADDR + 2);
    racce_x = recivie_mpu_byte(ACCE_ADDR);
    racce_y = recivie_mpu_byte(ACCE_ADDR + 1);
    racce_z = recivie_mpu_byte(ACCE_ADDR + 2);
}

void setup() {
    Serial.begin(9600);
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0);
    Wire.endTransmission();
    config_mpu(3,0);
}
void loop() {
    load_all_mpu();
    float gyro_x,gyro_y,gyro_z;
    float acce_x,acce_y,acce_z;
    float tempreture = temp / 340.0 + 36.53;
    gyro_x = rgyro_x / 16.4;
    gyro_y = rgyro_y / 16.4;
    gyro_z = rgyro_z / 16.4;
    //acce_x = racce_x / 

    Serial.print("Gyro: x: ");
    Serial.print(gyro_x);
    Serial.print(" Gyro: y: ");
    Serial.print(gyro_y);
    Serial.print(" Gyro: z: ");
    Serial.print(gyro_z);
    Serial.println();
    delay(1000);
} 
