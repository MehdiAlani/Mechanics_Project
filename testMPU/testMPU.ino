#include <Wire.h>
#include <MPU6050.h>

// Create an instance of the MPU6050 class
MPU6050 mpu;

// Calibration values for the accelerometer and gyroscope
float ax, ay, az;  // Accelerometer readings
float gx, gy, gz;  // Gyroscope readings
float pitch, roll;  // Pitch and Roll angles
float dt = 0.01;    // Time interval (for gyro integration)

unsigned long lastTime = 0;

// Complementary filter parameters
float alpha = 0.98;

void setup() {
  // Start Serial Monitor
  Serial.begin(115200);

  // Initialize the MPU6050
  Wire.begin();
  mpu.initialize();

  // Check if MPU6050 is connected
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed.");
    while (1);
  }
  Serial.println("MPU6050 connection successful.");
}

void loop() {
  // Read accelerometer and gyroscope data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert accelerometer data to g (1g = 9.8 m/s^2)
  ax = ax / 16384.0;
  ay = ay / 16384.0;
  az = az / 16384.0;

  // Calculate pitch and roll angles using accelerometer data
  float accelPitch = atan2(ay, az) * 180.0 / PI;  // Pitch angle from accelerometer
  float accelRoll = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;  // Roll angle from accelerometer

  // Get the current time
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;  // Time interval in seconds
  lastTime = currentTime;

  // Calculate angular velocity in degrees per second
  float gyroPitch = gx / 131.0;  // Gyroscope X-axis (pitch)
  float gyroRoll = gy / 131.0;   // Gyroscope Y-axis (roll)

  // Apply complementary filter to combine accelerometer and gyroscope data
  pitch = alpha * (pitch + gyroPitch * dt) + (1 - alpha) * accelPitch;
  roll = alpha * (roll + gyroRoll * dt) + (1 - alpha) * accelRoll;

  // Print the calculated pitch and roll angles
  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print(" | Roll: ");
  Serial.println(roll);

  // Add a small delay before the next reading
  delay(100);
}
