#include <map>
#include <string>
#include <WiFi.h>

using namespace std;
// IMPORTANT = ROBOT 1 code uses different pin configuration
typedef std::map<char, string> InnerMap; // Map char to string
typedef std::map<char, InnerMap> NestedMap; // Map char to InnerMap

const char* ssid = "Giriirig";             // Replace with your WiFi SSID
const char* password = "milesmyrandi";  // Replace with your WiFi password

String commands = "";            // Dynamic command storage initialized with some default commands

WiFiServer wifiServer(80); 

NestedMap translate;
int m1pin2 = 25; 
int m1pin1 = 26; 
int m2pin1 = 13; 
int m2pin2 = 27; 
char Curr_orientation = 'W';
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

  // translate[curr_orientation][command] = "executable command"
  // set the curr_orientation to the command after moving
  translate['W']['W'] = "W";
  translate['W']['A'] = "AW";
  translate['W']['S'] = "S";
  translate['W']['D'] = "DW";
  translate['A']['W'] = "DW";
  translate['A']['A'] = "W";
  translate['A']['S'] = "AW";
  translate['A']['D'] = "S";
  translate['S']['W'] = "S";
  translate['S']['A'] = "DW";
  translate['S']['S'] = "W";
  translate['S']['D'] = "AW";
  translate['D']['W'] = "AW";
  translate['D']['A'] = "S";
  translate['D']['S'] = "DW";
  translate['D']['D'] = "W";


}

void move(char command) {
  if (command == 'W') {
    digitalWrite(m1pin1, LOW);
    digitalWrite(m1pin2, HIGH);
    digitalWrite(m2pin1, LOW);
    digitalWrite(m2pin2, HIGH);
    delay(1300);
    digitalWrite(m1pin2, LOW);
    digitalWrite(m2pin2, LOW);
  } 
  else if (command == 'S') {
    digitalWrite(m1pin2, LOW);
    digitalWrite(m1pin1, HIGH);
    digitalWrite(m2pin2, LOW);
    digitalWrite(m2pin1, HIGH);
    delay(1300);
    digitalWrite(m1pin1, LOW);
    digitalWrite(m2pin1, LOW);
  } 
  else if (command == 'D') {
    digitalWrite(m1pin1, HIGH);
    digitalWrite(m1pin2, LOW);
    digitalWrite(m2pin1, LOW);
    digitalWrite(m2pin2, HIGH);
    delay(500);
    digitalWrite(m1pin1, LOW);
    digitalWrite(m2pin2, LOW);
  } 
  else if (command == 'A') {
    digitalWrite(m1pin2, HIGH);
    digitalWrite(m1pin1, LOW);
    digitalWrite(m2pin2, LOW);
    digitalWrite(m2pin1, HIGH);
    delay(500);
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
        received.trim(); // Remove any trailing whitespace

        if (received.length() == 1) {
          char newCommand = received.charAt(0);
          commands += newCommand; // Append the new command to the string
          digitalWrite(ledPin, HIGH);
          delay(200);
          digitalWrite(ledPin, LOW);
        }
      }
    }
    client.stop();
  }
  if (commands.length() > 0){
    char commandToProcess = commands[0];
    string executable = translate[Curr_orientation][commandToProcess];
    for (char cmd : executable) { 
      move(cmd);
      if(cmd == 'A' or cmd == 'D'){
        Curr_orientation = commandToProcess;
      } 
    }
    commands.remove(0, 1);
  }
}
