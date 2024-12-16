#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <ESP8266HTTPClient.h>

#define CELESTIAL_BODY_NUMBER 11 // Well 11 with the Earth ....

typedef struct CelestialBody {
  String name;
  int elevation;
  int azimuth;
}CelestialBody ;

CelestialBody SolarSystemPlanets[CELESTIAL_BODY_NUMBER];

// WiFi Credentials
const char* ssid = "ORANGE_9E88_Ext";
const char* password = "383LrjBv";

// Server Info
String newTime = "01%3A00%3A00";
String latitude = "36.844547";
String longitude = "10.196042";
String elevation = "10";
const char * host = "api.astronomyapi.com";
String url = "/api/v2/bodies/positions?latitude="+ latitude +"&longitude="+ longitude +"&"+ elevation +"=10&from_date=2024-12-16&to_date=2024-12-17&time=" + newTime;
const char* authorization = "Basic ZjY1OWU3NjQtZGNjNi00ZmM1LWI3OTUtZTg5N2NkZWVkNzgwOjZlNmFlNjU3NjZmMjg5M2E2YWE3NTBhM2M1ODU1NmE5MTlhMGM3ZDNhZmZjNTMxOTE0N2JjZGJkODg0MDg5YzQxYzEyNDgwN2Y3ZWRhM2UxMzk4YWIxNmE0MmRkYzI3NGIxYTFkNjRmMWY5Y2RkYWM4YzI1YTdkZTFkNThhMjU3NWUzOWU4ZDFjN2ExNjM2NjQyZDIyZDg4ODlkYTVmYWIwNDFkOWVhOTZjMDdkNWY4NTYzN2MzMmY2ODc5Y2M2ZTA3MjAyYTNiNjViZjZjMjk1OThkMGRjZDMxNjcwM2Qw";
WiFiClientSecure client;



void Display_CelestialBody(){
  Serial.println("CelestialBody Are :");
  for(int i = 0; i < CELESTIAL_BODY_NUMBER;i++){
    Serial.print("name: ");
    Serial.print(SolarSystemPlanets[i].name);
    Serial.print(" elevation: ");
    Serial.print(SolarSystemPlanets[i].elevation);
    Serial.print(" azimuth: ");
    Serial.println(SolarSystemPlanets[i].azimuth);
  }
}


int fetchAstronomyData() {
  String FullResponseJSON = "";
  client.setInsecure();
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  if (!client.connect(host, 443)) {
    Serial.println("Connection failed!");
    return -1;
  }
  Serial.println("Connected to server");

  // Send HTTPS GET request
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "User-Agent: python-requests/2.32.3\r\n" +
               "Accept-Encoding: gzip, deflate\r\n" +
               "Accept: */*\r\n" +
               "Connection: keep-alive\r\n" +
               "Authorization: " + authorization + "\r\n\r\n");

  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") break;
    Serial.println(line);    
  }
 
  Serial.print("Response Body: ");
  while (client.available()) {
    char c = client.read();
    Serial.print(c);
    FullResponseJSON += c;
  }
  const int n = FullResponseJSON.length();

  Serial.print("\nTotal Length : ");
  Serial.println(n);

  const int size_buffer = 25; 
  String string_buffer;

  String entry = "entry";
  String name = "name";
  String altitude = "altitude";
  String azimuth = "azimuth";


  String doublepoint = ":";
  String culybrace = "}";
  String point = ".";

  bool FLAG_ENTRY = false;
  bool FLAG_NAME = false;
  bool FLAG_AZIMUTH = false;
  bool FLAG_ALTITUDE = false;
  int index_cb = 0;
  int index;
  String buff;

  for(int i = 0; i < n && index_cb < CELESTIAL_BODY_NUMBER ; i++){
    
    string_buffer = FullResponseJSON.substring(i,size_buffer+i);


    if(FLAG_ENTRY == false){
      index = string_buffer.indexOf(entry);
      if(index != -1){
        FLAG_ENTRY = true;
        i = i + index + entry.length();
      }
    }
    else if(FLAG_NAME == false){
      index = string_buffer.indexOf(name);
      if(index != -1){
        FLAG_NAME = true;
        i = i + index + name.length();
        string_buffer = FullResponseJSON.substring(i,size_buffer+i);
        buff = string_buffer.substring(string_buffer.indexOf(doublepoint) + 2,string_buffer.indexOf(culybrace) - 1);
        SolarSystemPlanets[index_cb].name = buff;
      }
    }
    else if(FLAG_ALTITUDE == false){
      index = string_buffer.indexOf(altitude);
      if(index != -1){
        FLAG_ALTITUDE = true;
        i = i + index + name.length() + 7;
        string_buffer = FullResponseJSON.substring(i,size_buffer+i);
        buff = string_buffer.substring(string_buffer.indexOf(doublepoint) + 2,string_buffer.indexOf(point));
        SolarSystemPlanets[index_cb].elevation = buff.toInt();
      }
    }
    else if(FLAG_AZIMUTH == false){
      index = string_buffer.indexOf(azimuth);
      if(index != -1){
        i = i + index + name.length() + 7;
        string_buffer = FullResponseJSON.substring(i,size_buffer+i);
        buff = string_buffer.substring(string_buffer.indexOf(doublepoint) + 2,string_buffer.indexOf(point));
        SolarSystemPlanets[index_cb].azimuth = buff.toInt();

        FLAG_AZIMUTH = false;
        FLAG_ALTITUDE = false;
        FLAG_ENTRY = false;
        FLAG_NAME = false;
        index_cb++;
      }
    }
    
  }
  Display_CelestialBody();

  Serial.print("\nTotal Length : ");
  Serial.println(n);
  Serial.println("Disconnected from WiFi/Server");
  client.stop();
  WiFi.disconnect();
  return 0;
}


void setup() {
  Serial.begin(115200);
  fetchAstronomyData();
}


void loop() {
  // Nothing here
}


