#include <map>
#include <string>
using namespace std;
// IMPORTANT = ROBOT 1 code uses different pin configuration
typedef std::map<char, string> InnerMap; // Map char to string
typedef std::map<char, InnerMap> NestedMap; // Map char to InnerMap

NestedMap translate;
int m1pin2 = 25; 
int m1pin1 = 26; 
int m2pin1 = 13; 
int m2pin2 = 27; 
char commands[] = {'W', 'A', 'S', 'D'};
char Curr_orientation = 'W';

void setup() {
  pinMode(m1pin1, OUTPUT);
  pinMode(m1pin2, OUTPUT);
  pinMode(m2pin1, OUTPUT);
  pinMode(m2pin2, OUTPUT);

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

  Serial.begin(115200);
  Serial.println("Testing DC Motor...");
  for (int i = 0; i < sizeof(commands); i++) {
    char command = commands[i];
    string executable = translate[Curr_orientation][command];
    for (char cmd : executable) { 
      move(cmd);
      delay(1000);
      if(cmd == 'A' or cmd == 'D'){
        Curr_orientation = command;
      }
      
    }
  }
}

void move(char command) {
  if (command == 'W') {
    Serial.println("Moving Forward");
    digitalWrite(m1pin1, LOW);
    digitalWrite(m1pin2, HIGH);
    digitalWrite(m2pin1, LOW);
    digitalWrite(m2pin2, HIGH);
    delay(1300);
    digitalWrite(m1pin2, LOW);
    digitalWrite(m2pin2, LOW);
  } 
  else if (command == 'S') {
    Serial.println("Moving Backward");
    digitalWrite(m1pin2, LOW);
    digitalWrite(m1pin1, HIGH);
    digitalWrite(m2pin2, LOW);
    digitalWrite(m2pin1, HIGH);
    delay(1300);
    digitalWrite(m1pin1, LOW);
    digitalWrite(m2pin1, LOW);
  } 
  else if (command == 'D') {
    Serial.println("Turning Right");
    digitalWrite(m1pin1, HIGH);
    digitalWrite(m1pin2, LOW);
    digitalWrite(m2pin1, LOW);
    digitalWrite(m2pin2, HIGH);
    delay(500);
    digitalWrite(m1pin1, LOW);
    digitalWrite(m2pin2, LOW);
  } 
  else if (command == 'A') {
    Serial.println("Turning Left");
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
}
