#include <Arduino.h>
#include <MPU6050.h>
#include <math.h> 
#include <Wire.h>


int16_t gx,gy,gz,accex,accey,accez;
double gyro_x,gyro_y,gyro_z,acce_x,acce_y,acce_z;
double pitch=0,yaw=0,raw_yaw=0;
double azimuth,elevation;
bool integrate = true;
MPU6050 mpu;

// Here u have your offsets :))
#define GYRO_X_OFFSET -1.406477
#define GYRO_Y_OFFSET 0.680898
#define GYRO_Z_OFFSET -0.429791
#define GYRO_CONST 131.0
void setup_gyro(){
    gyro_x = gx / GYRO_CONST - GYRO_X_OFFSET;
    gyro_y = gy / GYRO_CONST - GYRO_Y_OFFSET;
    gyro_z = gz / GYRO_CONST - GYRO_Z_OFFSET;
}


#define ACCE_X_OFFSET 0.018353
#define ACCE_Y_OFFSET -0.009344
#define ACCE_Z_OFFSET 0.000292

#define ACCE_CONST 8192.0
void setup_acce(){
    acce_x = accex / ACCE_CONST - ACCE_X_OFFSET;
    acce_y = accey / ACCE_CONST - ACCE_Y_OFFSET;
    acce_z = accez / ACCE_CONST - ACCE_Z_OFFSET;
}

void print_all_raw(){
    Serial.print("gx: ");
    Serial.print(gx);
    Serial.print(" gy: ");
    Serial.print(gy);
    Serial.print(" gz: ");
    Serial.print(gx);


    Serial.print(" accex: ");
    Serial.print(accex);
    Serial.print(" accey: ");
    Serial.print(accey);
    Serial.print(" accez: ");
    Serial.println(accez);
}
void print_all(){
    Serial.print("gyro_x: ");
    Serial.print(gyro_x);
    Serial.print(" gyro_y: ");
    Serial.print(gyro_y);
    Serial.print(" gyro_z: ");
    Serial.print(gyro_z);

    
    Serial.print(" acce_x: ");
    Serial.print(acce_x);
    Serial.print(" acce_y: ");
    Serial.print(acce_y);
    Serial.print(" acce_z: ");
    Serial.println(acce_z);
}

#define FLOATING_POINT_NUMBER 6
#define SAMPLE_TIME_US 10
#define PRINT_TIME_MS 500
#define MAX_SAMPLES 9999999
void calibrate_gyro(){
    Serial.println("Put the IMU in a bread Board DONT MOVE IT KEEP IT IT STATIONARY ON A FLAT TABLE !!!!!! ;) ");
    Serial.print("Starting with number of Samples: ");
    Serial.println(MAX_SAMPLES);
    double sum_gx = 0,sum_gy = 0,sum_gz = 0,sum_accex = 0,sum_accey = 0,sum_accez = 0;
    unsigned long startTime_ms = millis();
    unsigned long startTime_us = micros();
    unsigned long i = 1;
    // Getting avrages Values.. 
    while(i < MAX_SAMPLES){
        if(micros() - startTime_us >= SAMPLE_TIME_US){
            mpu.getMotion6(&accex,&accey,&accez,&gx,&gy,&gz);
            setup_gyro();
            setup_acce();
            sum_gx += gyro_x;
            sum_gy += gyro_y;
            sum_gz += gyro_z;
            sum_accex += acce_x;
            sum_accey += acce_y;
            sum_accez += acce_z;
            startTime_us = micros();
            i++;
        }
        if(millis() - startTime_ms >= PRINT_TIME_MS){
            Serial.print("offgx: ");
            Serial.print(sum_gx / i,FLOATING_POINT_NUMBER);
            Serial.print(" offgy: ");
            Serial.print(sum_gy / i,FLOATING_POINT_NUMBER);
            Serial.print(" offgz: ");
            Serial.print(sum_gz / i,FLOATING_POINT_NUMBER);
            Serial.print(" oaccx: ");
            Serial.print(sum_accex / i,FLOATING_POINT_NUMBER);
            Serial.print(" oaccy: ");
            Serial.print(sum_accey / i,FLOATING_POINT_NUMBER);
            Serial.print(" oaccz: ");
            Serial.println(sum_accez / i - 1,FLOATING_POINT_NUMBER);
            startTime_ms = millis();
        }
    }   
}

void show_raw(){
    mpu.getMotion6(&accex,&accey,&accez,&gx,&gy,&gz);
    setup_gyro();
    setup_acce();
    Serial.println("=================================================================Calculated===================================================================");
    print_all();
    Serial.println("====================================================================RAW=======================================================================");
    print_all_raw();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    delay(100);
}
#define SENSOR_FUSION_ALPHA 0.98
#define EPSELON 0.058415
#define MIN_STEP_TIME_US 10
#define PHASE_DEGGRES 60
double calibrate_pitch(double v){
    if(v > 180) v = v - 180;
    else if(v < 0) v = 180 + v; 
    return v;
}
double calibrate_yaw(double v){
    if(v >= 360) v = v - 360;
    else if(v < 0) v = 360 + v;
    return v;
}
void print_telescope(){
    azimuth = yaw;
    elevation = pitch;
    
    Serial.print("azimuth: ");
    Serial.print(azimuth);
    Serial.print(" elevation: "); 
    Serial.println(elevation);
}
void show_angle(){
    double yaw_sum = 0;
    uint64_t count = 0;
    unsigned long startTime_ms = millis();
    unsigned long startTime_us = micros();
    while(integrate == true){
        mpu.getMotion6(&accex,&accey,&accez,&gx,&gy,&gz);
        setup_gyro();
        setup_acce();
        if(micros() - startTime_us >= MIN_STEP_TIME_US){
            // Sensor Fusion 
            /*
            pitch_gyro += (gyro_x / 1000) * ((micros() - startTime_us) / 1000.0) ;
            pitch = (SENSOR_FUSION_ALPHA * pitch_acce) + (1 - SENSOR_FUSION_ALPHA) * pitch_gyro;
            */
            //Serial.println(gyro_z);  
            yaw_sum += gyro_z;
            count++;

            if(true) raw_yaw -= (gyro_z / 1000) * ((micros() - startTime_us) / 1000.0); 
            if(raw_yaw >= (360 - PHASE_DEGGRES)) raw_yaw = raw_yaw - (360 - PHASE_DEGGRES);// Calibrate this alone... it is raw data
            else if(raw_yaw < 0) raw_yaw = (360 - PHASE_DEGGRES)  + raw_yaw;


            pitch = calibrate_pitch((atan(acce_z / acce_y)) * 180 / PI - 90);
            yaw = calibrate_yaw(raw_yaw + raw_yaw * PHASE_DEGGRES /(360.0 - PHASE_DEGGRES));
            startTime_us = micros();
        }
        if(millis() - startTime_ms >= PRINT_TIME_MS){

            print_telescope();
            /*
            Serial.print("pitch_gyro: ");
            Serial.print(pitch_gyro);
            Serial.print(" pitch_acce: ");
            Serial.print(pitch_acce);
            Serial.print("yaw_avrage: ");
            Serial.print(yaw_sum / count,FLOATING_POINT_NUMBER);
            Serial.print(" pitch: ");
            Serial.print(pitch,FLOATING_POINT_NUMBER);
            Serial.print(" real_yaw: ");
            Serial.print(real_yaw,FLOATING_POINT_NUMBER);
            Serial.print(" raw_yaw: ");
            Serial.println(yaw,FLOATING_POINT_NUMBER);
            */
           startTime_ms = millis();
        }
    }
}

void setup(){

    Wire.begin();
    Serial.begin(9600);
    mpu.initialize();
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    if(!mpu.testConnection()){
        Serial.println("Connection Failed");
        while(1);
    }
    Serial.println("Connection Successful");
    show_angle();
}

void loop(){


}