#include <ESP32Servo.h>

// Pin definitions
const int potPin = 34;   // Analog pin connected to the potentiometer
const int servoPin = 25; // PWM pin connected to the servo signal

Servo myServo;

void setup() {
  // Attach the servo to the specified pin
  myServo.attach(servoPin);

  // Begin serial communication for debugging
  Serial.begin(115200);
}

void loop() {
  // Read the potentiometer value (0 to 4095 on ESP32)
  int potValue = analogRead(potPin);

  // Map potentiometer value to servo angle (0 to 180 degrees)
  int servoAngle = map(potValue, 0, 4095, 0, 360);

  // Write the angle to the servo motor
  myServo.write(servoAngle);

  // Debug output to the serial monitor
  Serial.print("Potentiometer Value: ");
  Serial.print(potValue);
  Serial.print(" -> Servo Angle: ");
  Serial.println(servoAngle);

  // Small delay to stabilize readings
  delay(20);
}
