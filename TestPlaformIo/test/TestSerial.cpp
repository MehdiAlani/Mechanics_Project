#include <Wire.h>
#include <Arduino.h>
#include <math.h>

bool isNumeric(char * s){
    int n;
    for(n = 0; s[n] != '\0';n++);
    int i = 0;
    while(s[i] <= '9' && s[i] >= '0' && i < n)i++;
    return i == n;
}
void ReadSerialString(char * s,char terminator ,int maxSize){
    int i = 0;
    char c = Serial.read();
    while (c != terminator && i < maxSize - 1){
      s[i] = c;
      while(Serial.available() <= 0);
      c = Serial.read();
      i++;
    }
    s[i] = '\0';
    if(i >= maxSize - 1){
      Serial.println("U have Reached The Max Size");
      return;
    }
}
#define SERIAL_BUFFER_SIZE 200
char Serial_Buffer[SERIAL_BUFFER_SIZE];

void setup() {
  Serial.begin(9600);

}

void loop() {
  if(Serial.available() > 0){
    ReadSerialString(Serial_Buffer,'\n',SERIAL_BUFFER_SIZE);
    Serial.println(isNumeric(Serial_Buffer));
  }
  delay(100);
} 
