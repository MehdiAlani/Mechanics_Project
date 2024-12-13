#include <Stepper.h>

// Define stepper motor connections and interface type
#define IN1 13 // GPIO for IN1
#define IN2 12 // GPIO for IN2
#define IN3 14 // GPIO for IN3
#define IN4 27// GPIO for IN4
#define RPM 10


const int step_per_rev = 2048;
Stepper stepper(step_per_rev,IN1,IN3,IN2,IN4);

void setup() {
  stepper.setSpeed(RPM);
  
  Serial.begin(9600);
}

void loop() {
  Serial.println("clockwise");
  stepper.step(step_per_rev);
  delay(1000);
  Serial.println("contreclockwise");
  stepper.step(-step_per_rev);
  delay(1000);
}
