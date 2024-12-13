#include <Wire.h>
#include <Arduino.h>
#include <math.h>

bool isNumeric(char * s){
    int i = 0;
    while(s[i] != '\0' && s[i] <= '9' && s[i] >= '0'){
        i++;
    }
    return (s[i] == (char)'\0');
}
void ReadSerialString(char * s,char terminator ,int maxSize){
    int i = 0;
    char c = Serial.read();
    Serial.write(c);
    while (c != terminator && i < maxSize - 1){
      s[i] = c;
      while(Serial.available() <= 0);
      c = Serial.read();
      Serial.write(c);
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
