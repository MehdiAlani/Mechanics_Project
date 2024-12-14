#include <Arduino.h>
#include <Stepper.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050.h>


#define PIN_TEST_ADC 26
#define PIN_3V3 25
#define DELAY_TIME_TEST 5000
#define PIN_NEG_2 2

__uint32_t print_time_ms = 0;
__uint32_t stepper_time_ms = 0;
__uint32_t servo_time_ms = 0;
__uint32_t motion_time_ms = 0;
__uint32_t integrate_time_yaw_us = 0;
__uint32_t integrate_time_pitch_us = 0;
__uint32_t pitch_detect_ms = 0;
__uint32_t yaw_detect_ms = 0;


bool FLAG_AZIMUTH_CHANGE = false;
bool FLAG_ELEVATION_CHANGE = false; 
bool FLAG_GET_ELEVATION = false;
bool FLAG_GET_AZIMUTH = false;
MPU6050 mpu;
int16_t gx,gy,gz,accex,accey,accez;


double gyro_x,gyro_y,gyro_z,acce_x,acce_y,acce_z;
double pitch,yaw,raw_yaw,system_pitch;
double azimuth;
double elevation;
double CurrentPos;

// History Functions 

/*

    void random_move(){
    int x_angle = random(180);
    int z_steps = random(STEPS_PER_REV) / 10 - random(STEPS_PER_REV) / 50;
    stepper.step(z_steps);
    stepper_stop();
    delay(500);
    servo.write(x_angle);
    delay(500);
    Serial.print("x_angle: ");
    Serial.print(x_angle);
    Serial.print(" Z_steps: ");
    Serial.println(z_steps);
    delay(DELAY_TIME_TEST);
    }

  // This Try Binary search Method :)))
    mpu.getMotion6(&accex,&accey,&accez,&gx,&gy,&gz);
    setup_gyro();
    setup_acce();
    update_pitch();

    if(Serial.available() > 0){
        get_user_serial();
    }
    difference_elevation = pitch - elevation;
    /*
    if(millis() - print_time_ms > PRINT_ANGLE_TIME_MS){
        Serial.print("pitch: ");
        Serial.print(pitch);
        Serial.print(" elevation: ");
        Serial.print(elevation);
        Serial.print(" Difference: ");
        Serial.println(abs(difference_elevation));
        print_time_ms = millis();
    }

    if(abs(difference_elevation) > EPSELON){
      FLAG_ELEVATION_CHANGE = true;
      if(millis() - servo_time_ms > SERVO_TIME_FEEDBACK_MS){
        int servo_angle;
        if(difference_elevation > 0) {
           servo_angle = (int)(pitch + (difference_elevation) / 2);
        }
        else{
            servo_angle = (int)(pitch + abs(difference_elevation) / 2);
        }

        Serial.print("pitch: ");
        Serial.print(pitch);
        Serial.print(" elevation: ");
        Serial.print(elevation);
        Serial.print(" Difference: ");
        Serial.print(difference_elevation);
        Serial.print(" Servo_angle: ");
        Serial.println(servo_angle);

        servo.write(servo_angle);
        servo_time_ms = millis();
      }
    }
    else{
      FLAG_ELEVATION_CHANGE = false;
    }


// This for detecting motion with accelerometerthe problem is this for acceleration not speed
bool detect_motion(double motion_vect){
    return sqrt(acce_x * acce_x + acce_y * acce_y + acce_z * acce_z) > motion_vect;
}

#define MIN_STEP 0.000001
void find_min_vector(){
    for(double i = 1; i < 5; i += MIN_STEP){
        while(!detect_motion(i)){
            mpu.getMotion6(&accex,&accey,&accez,&gx,&gy,&gz);
            setup_acce();
            Serial.print("Vect now is: ");
            Serial.print(sqrt(acce_x * acce_x + acce_y * acce_y + acce_z * acce_z),FLOATING_POINT_NUMBER);
            Serial.print(" Max Vect is: ");
            Serial.println(i,FLOATING_POINT_NUMBER);
            delayMicroseconds(20);
        }
    }
    return;
}
*/



//=================================================Serial Functions==============================================================================================  
#define SERIAL_BUFFER_SIZE 200
String Serial_Buffer;
bool isNumeric(char * s){
    int i = 0;
    while(s[i] != '\0' && s[i] <= '9' && s[i] >= '0'){
        i++;
    }
    return (s[i] == (char)'\0');
}

void Clear_Serial(){
    while(Serial.available() > 0) Serial.read();
}
void ReadSerialString(char * s,char terminator1,char terminator2 ,int maxSize){
    // Wait For the First Character ... NOPEEEE 
    //while(Serial.available() <= 0);

    // Begin 
    int i = 0;
    char c = Serial.read();
    while (c != terminator1 && c != terminator2 && i < maxSize - 1){
      s[i] = c;
      //while(Serial.available() <= 0); u can use it if u have a real-time serial monitor not like the arduino ide...
      c = Serial.read();
      i++;
    }
    s[i] = '\0';
    if(i >= maxSize - 1){
      Serial.println("U have Reached The Max Size");
      return;
    }
}
void get_user_serial(){
    char serial_msg[SERIAL_BUFFER_SIZE];
    
    if(FLAG_GET_AZIMUTH == false){
        ReadSerialString(serial_msg,'\n',' ',SERIAL_BUFFER_SIZE);
        if(!isNumeric(serial_msg)) return;
        Serial_Buffer = String(serial_msg);
        double azimuth_buffer = Serial_Buffer.toDouble();
        if(azimuth_buffer > 360 || azimuth_buffer < 0){
            Serial.println("Give  azimuth(0/360): ");
            return; 
        }
        else{
            azimuth = azimuth_buffer;
            FLAG_GET_AZIMUTH = true;
        }
    }
        if(FLAG_GET_ELEVATION == false && FLAG_GET_AZIMUTH == true && Serial.available() > 0){
            ReadSerialString(serial_msg,'\n',' ',SERIAL_BUFFER_SIZE);
            if(!isNumeric(serial_msg)) return;
            Serial_Buffer = String(serial_msg);
            double elevation_buffer =  Serial_Buffer.toDouble();
            if (elevation_buffer > 90 || elevation_buffer < 0){
                Serial.println("Give elevation(-90/90): ");
                return;
            }
            else{
                elevation = elevation_buffer;
                FLAG_GET_AZIMUTH = false;
                FLAG_GET_ELEVATION = false;
            }
        }

}




//=================================================MPU Functions==============================================================================================  

#define GYRO_X_OFFSET -1.422841
#define GYRO_Y_OFFSET  0.690009
#define GYRO_Z_OFFSET  -0.533911
#define GYRO_CONST 131.0
void setup_gyro(){
    gyro_x = gx / GYRO_CONST - GYRO_X_OFFSET;
    gyro_y = gy / GYRO_CONST - GYRO_Y_OFFSET;
    gyro_z = gz / GYRO_CONST - GYRO_Z_OFFSET;
}


#define ACCE_X_OFFSET 0.057769 
#define ACCE_Y_OFFSET 0.009872
#define ACCE_Z_OFFSET -0.000869

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
void show_all_types(){
    Serial.println("=================================================================Calculated===================================================================");
    print_all();
    Serial.println("====================================================================RAW===================================================================");
    print_all_raw();
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
}

#define FLOATING_POINT_NUMBER 6
#define PRINT_TIME_MS 500
#define SENSOR_FUSION_ALPHA 0.98
#define EPSELON 0.058415
#define SAMPLE_TIME_US 10
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

#define MIN_YAW 0.5
bool detect_yaw(double min){
    return abs(gyro_z) > min;
}
#define MIN_PITCH 0.5
bool detect_pitch(double min){
    return abs(gyro_x) > min;
}
#define MAX_SPEED 99999
#define MIN_STEP 0.000001
void find_min_yaw(){
    for(double i = 0; i < MAX_SPEED; i += MIN_STEP){
        while(!detect_yaw(i)){
            mpu.getMotion6(&accex,&accey,&accez,&gx,&gy,&gz);
            setup_gyro();
            Serial.print("Speed now is: ");
            Serial.print(gyro_z,FLOATING_POINT_NUMBER);
            Serial.print(" Max Speed is: ");
            Serial.println(i,FLOATING_POINT_NUMBER);
            delayMicroseconds(20);
        }
    }
    return;
}

void find_min_pitch(){
    for(double i = 0; i < MAX_SPEED; i += MIN_STEP){
        while(!detect_pitch(i)){
            mpu.getMotion6(&accex,&accey,&accez,&gx,&gy,&gz);
            setup_gyro();
            Serial.print("Speed now is: ");
            Serial.print(gyro_x,FLOATING_POINT_NUMBER);
            Serial.print(" Max Speed is: ");
            Serial.println(i,FLOATING_POINT_NUMBER);
            delayMicroseconds(20);
        }
    }
    return;
}



double calibrate_yaw(double v){
    if(v >= 360) v = v - 360;
    else if(v < 0) v = 360 + v;
    return v;
}

void update_yaw(){
    if( micros() - integrate_time_yaw_us > SAMPLE_TIME_US ){
        yaw -= (gyro_z / 1000) * ((micros() - integrate_time_yaw_us) / 1000.0);
        yaw = calibrate_yaw(yaw);
        integrate_time_yaw_us = micros();
    }
}

#define ALPHA_FUSION  0.98
void update_pitch(){
    double sample;
    while(acce_x == 0 && acce_z == 0){
        mpu.getMotion6(&accex,&accey,&accez,&gx,&gy,&gz);
        setup_gyro();
        setup_acce();
    }
    if(micros() - integrate_time_pitch_us > SAMPLE_TIME_US){
        sample = (gyro_x / 1000) * ((micros() - integrate_time_pitch_us) / 1000.0);
        integrate_time_pitch_us = micros();
    }
    pitch = (ALPHA_FUSION ) * (pitch + sample) + (1 - ALPHA_FUSION) *(atan2(acce_y,sqrt(acce_z * acce_z + acce_x * acce_x)) * 180.0 / PI );
}



//=================================================Motors_functions==============================================================================================
#define COIL1 13
#define COIL2 12
#define COIL3 14
#define COIL4 27
#define SERVO_PIN 33 
#define STEPER_SPEED_RPM 10
#define STEPS_PER_REV 2048.0
#define DEGREE_STEP 360 / STEPS_PER_REV
Stepper stepper(STEPS_PER_REV,COIL1,COIL3,COIL2,COIL4);
Servo servo;
double motor_current_angle;


void stepper_stop(){ // this because an issuse of the library ... kinda of 
    digitalWrite(COIL1,0);
    digitalWrite(COIL2,0);
    digitalWrite(COIL3,0);
    digitalWrite(COIL4,0);
}


#define ERROR_AZIMUTH 2
void stepper_move(){
    double full_pos = yaw;
    full_pos = calibrate_yaw(full_pos);
    double CurrentPos_Mirror =  full_pos + 180;
    CurrentPos_Mirror = calibrate_yaw(CurrentPos_Mirror);
    double CurrentPos_Azimuth =  azimuth + 180;
    CurrentPos_Azimuth = calibrate_yaw(CurrentPos_Azimuth);
    double diff = azimuth - full_pos; 
    if(abs(diff) > ERROR_AZIMUTH || abs(CurrentPos_Azimuth - CurrentPos_Mirror) > ERROR_AZIMUTH){
         FLAG_ELEVATION_CHANGE = false; 
        if( (0 <= diff && diff <= 180) || (-180 > diff && diff > -360) ){
            CurrentPos +=  DEGREE_STEP;
            stepper.step(1);
        }
        else{
            CurrentPos -=  DEGREE_STEP;
            stepper.step(-1);
        }
        CurrentPos = calibrate_yaw(CurrentPos);
    }
    else{
      stepper_stop();
      FLAG_ELEVATION_CHANGE = true; 
    }
}
#define ERROR_ELEVATION 2
void servo_move(){
    if(abs(elevation - pitch) > ERROR_ELEVATION){
        servo.write((int) (elevation + pitch));
    }
    FLAG_ELEVATION_CHANGE = false;
}


//==================================================================INIT_function==============================================================================================

void setup(){
    servo.attach(SERVO_PIN);
    stepper.setSpeed(STEPER_SPEED_RPM);
    Serial.begin(115200);
    Wire.begin();  
    mpu.initialize();
    if(!mpu.testConnection()){
        Serial.println("MPU NOT CONNECTED ...");
        while(1);
    }
    Serial.println("MPU STARTING");
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    pinMode(COIL1,OUTPUT);
    pinMode(COIL2,OUTPUT);
    pinMode(COIL3,OUTPUT);
    pinMode(COIL4,OUTPUT);
    pinMode(PIN_NEG_2,OUTPUT);

    digitalWrite(PIN_NEG_2,LOW);
    servo.write(0);
    delay(500);
    mpu.getMotion6(&accex,&accey,&accez,&gx,&gy,&gz);
    setup_gyro();
    setup_acce();
    update_pitch();
    update_yaw();


    FLAG_ELEVATION_CHANGE = false;
    CurrentPos = yaw;
    servo_time_ms = millis();
    stepper_time_ms = millis();
    print_time_ms = millis();
    motion_time_ms = millis();
    pitch_detect_ms = millis();
    yaw_detect_ms = millis();
    integrate_time_pitch_us = micros();
    integrate_time_yaw_us = micros();


    //calibrate_gyro();
    //find_min_yaw();
    //find_min_pitch();
}

//==================================================================LOOP_function==============================================================================================

#define STEPPER_TIME_FEEDBACK_MS 5
#define SERVO_TIME_FEEDBACK_MS 200
#define MOTOR_TIME_OUT_MS 500
#define PRINT_ANGLE_TIME_MS 1000
#define MOTION_TIMEOUT 500
#define EPSELON 1 
void loop(){
  
    mpu.getMotion6(&accex,&accey,&accez,&gx,&gy,&gz);
    setup_gyro();
    setup_acce();
    // This is for Pitch 
    if(detect_pitch(MIN_PITCH)){
        update_pitch();
        pitch_detect_ms = millis();
    }
    else{
        if(millis() - pitch_detect_ms < MOTION_TIMEOUT){
            update_pitch();
        }
        else{
            integrate_time_pitch_us = micros();
        }
    }
    // This is for Yaw 
    if(detect_yaw(MIN_YAW)){
        update_yaw();
        yaw_detect_ms = millis();
    }
    else{
        if(millis() - yaw_detect_ms < MOTION_TIMEOUT){
            update_yaw();
        }
        else{
            integrate_time_yaw_us = micros();
        }
    }




    if(Serial.available() > 0){
        get_user_serial();
        Clear_Serial();
    }
    if(millis() - print_time_ms > PRINT_ANGLE_TIME_MS){

        Serial.print("pitch: ");
        Serial.print(pitch);
        Serial.print(" yaw: ");
        Serial.print(yaw);
        Serial.print(" Current Pos: ");
        Serial.print(CurrentPos);
        Serial.print(" azimuth: ");
        Serial.print(azimuth);
        Serial.print(" elevation: ");
        Serial.println(elevation);
        print_time_ms = millis();

    }


    if(millis() - stepper_time_ms > STEPPER_TIME_FEEDBACK_MS){
        stepper_move();
        stepper_time_ms = millis();
    }
    if(millis() - servo_time_ms > SERVO_TIME_FEEDBACK_MS){
        servo_move();
        servo_time_ms = millis();
    }
    
    
    
}

