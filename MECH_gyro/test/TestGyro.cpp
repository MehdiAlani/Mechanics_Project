#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Math.h>  // For the atan2 function

// Create an instance of the Adafruit_MPU6050 class
Adafruit_MPU6050 mpu;

// Variables for storing sensor data
float gx, gy, gz,ax,ay,az;  // Gyroscope data (angular velocity in degrees/sec)
float yaw = 0;      // Yaw angle in degrees (initial value is 0)
float pitch = 0;    // Pitch angle in degrees (initial value is 0)
unsigned long previousMillis = 0;  // Variable to store last time for calculating delta time
float dt = 0.01;    // Time interval for each loop, 10ms is a good starting point

void setup() {
  // Start the serial communication
  Serial.begin(115200);

  // Initialize I2C
  Wire.begin();

  // Initialize the MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to initialize MPU6050");
    while (1);
  }

  // Set the gyroscope range using setGyroRange() method (±250 degrees per second)
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // Set the accelerometer range (optional)
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

  // Set the filter bandwidth (optional, to reduce noise)
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 initialized successfully!");
}

void loop() {
  // Read the sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Gyroscope data (angular velocity in degrees/sec)
  gx = g.gyro.x;  // Gyroscope X-axis angular velocity (not used here)
  gy = g.gyro.y;  // Gyroscope Y-axis angular velocity (not used here)
  gz = g.gyro.z;  // Gyroscope Z-axis angular velocity (used for yaw)


  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;

  // Calculate pitch and roll angles in degrees (based only on accelerometer data)
  pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180 / PI;
  // Calculate the yaw angle in degrees
  // Integrate the gyroscope Z-axis data over time to get the yaw angle
  unsigned long currentMillis = millis();
  dt = (currentMillis - previousMillis) / 1000.0;  // Time interval in seconds
  previousMillis = currentMillis;

  yaw += (gz * dt) * 180.0 / PI;  // Convert radians to degrees

  // Limit the yaw to keep it between -180 and 180 degrees
  if (yaw > 180) yaw -= 360;
  if (yaw < -180) yaw += 360;

  // Calculate the pitch angle in degrees using accelerometer data
  // Use the accelerometer X and Z data for pitch calculation
  pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180 / PI;
  // Print the yaw and pitch values to the serial monitor
  Serial.print("Yaw: ");
  Serial.print(yaw);
  Serial.print("°  ");
  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.println("°");

  delay(10);  // Delay for 10ms (100Hz loop rate)
}
