#include <map>
#include <string>
#include <WiFi.h>

using namespace std;
// IMPORTANT = ROBOT 1 code uses different pin configuration

const char* ssid = "Giriirig";             // Replace with your WiFi SSID
const char* password = "milesmyrandi";  // Replace with your WiFi password

WiFiServer wifiServer(80); 

int m1pin2 = 25; 
int m1pin1 = 26; 
int m2pin1 = 13; 
int m2pin2 = 27; 
int ledPin = 2;

void setup() {

  Serial.begin(115200);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  
  wifiServer.begin();


  pinMode(m1pin1, OUTPUT);
  pinMode(m1pin2, OUTPUT);
  pinMode(m2pin1, OUTPUT);
  pinMode(m2pin2, OUTPUT);
  pinMode(ledPin, OUTPUT);
}

void move(char command) {
  if (command == 'W') {
    digitalWrite(m1pin2, LOW);
    digitalWrite(m1pin1, HIGH);
    digitalWrite(m2pin2, LOW);
    digitalWrite(m2pin1, HIGH);
    delay(1300);
    digitalWrite(m1pin1, LOW);
    digitalWrite(m2pin1, LOW);
  } 
  else if (command == 'S') {
    digitalWrite(m1pin1, LOW);
    digitalWrite(m1pin2, HIGH);
    digitalWrite(m2pin1, LOW);
    digitalWrite(m2pin2, HIGH);
    delay(1300);
    digitalWrite(m1pin2, LOW);
    digitalWrite(m2pin2, LOW);
  } 
  else if (command == 'D') {
    digitalWrite(m1pin1, HIGH);
    digitalWrite(m1pin2, LOW);
    digitalWrite(m2pin1, LOW);
    digitalWrite(m2pin2, HIGH);
    delay(725);
    digitalWrite(m1pin1, LOW);
    digitalWrite(m2pin2, LOW);
  } 
  else if (command == 'A') {
    digitalWrite(m1pin2, HIGH);
    digitalWrite(m1pin1, LOW);
    digitalWrite(m2pin2, LOW);
    digitalWrite(m2pin1, HIGH);
    delay(725);
    digitalWrite(m1pin2, LOW);
    digitalWrite(m2pin1, LOW);
  }
}


void loop() {
  WiFiClient client = wifiServer.available();
  if (client) {
    while (client.connected()) {
      if (client.available()) {
        String received = client.readStringUntil('\n');
        received.trim();

        if (received.length() == 1) {
          char newCommand = received.charAt(0);
          digitalWrite(ledPin, HIGH);
          delay(200);
          digitalWrite(ledPin, LOW);
          move(newCommand);
        }
      }
    }
    client.stop();
  }
}
