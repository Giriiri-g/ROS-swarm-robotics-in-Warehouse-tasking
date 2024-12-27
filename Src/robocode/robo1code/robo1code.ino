#include <WiFi.h>

// IMPORTANT = ROBOT 1 code uses different pin configuration

const char* ssid = "Giriirig";
const char* password = "password";

WiFiServer wifiServer(80);

const int m1pin1 = 26, m1pin2 = 25, m2pin1 = 13, m2pin2 = 27, ledPin = 2;

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  wifiServer.begin();

  pinMode(m1pin1, OUTPUT);
  pinMode(m1pin2, OUTPUT);
  pinMode(m2pin1, OUTPUT);
  pinMode(m2pin2, OUTPUT);
  pinMode(ledPin, OUTPUT);
}

void move(char command, int speed) {
  switch (command) {
    case 'W':
      digitalWrite(m1pin1, LOW);
      digitalWrite(m1pin2, HIGH);
      digitalWrite(m2pin1, LOW);
      digitalWrite(m2pin2, HIGH);
      delay(speed);
      relax();
      break;
    case 'S':
      digitalWrite(m1pin2, LOW);
      digitalWrite(m1pin1, HIGH);
      digitalWrite(m2pin2, LOW);
      digitalWrite(m2pin1, HIGH);
      delay(speed);
      relax();
      break;
    case 'D':
      digitalWrite(m1pin1, HIGH);
      digitalWrite(m1pin2, LOW);
      digitalWrite(m2pin1, LOW);
      digitalWrite(m2pin2, HIGH);
      delay(speed);
      relax();
      move('W', 1300);
      break;
    case 'A':
      digitalWrite(m1pin2, HIGH);
      digitalWrite(m1pin1, LOW);
      digitalWrite(m2pin2, LOW);
      digitalWrite(m2pin1, HIGH);
      delay(speed);
      relax();
      move('W', 1300);
      break;
    default:
      return;
  }
}

void relax(){
  digitalWrite(m1pin1, LOW);
  digitalWrite(m1pin2, LOW);
  digitalWrite(m2pin1, LOW);
  digitalWrite(m2pin2, LOW);
}


void parseAndExecute(String data) {
  data.replace("(", "");
  data.replace(")", "");

  char command = data.charAt(0);
  int speed = data.substring(data.indexOf(',') + 2).toInt();

  digitalWrite(ledPin, HIGH);
  delay(200);
  digitalWrite(ledPin, LOW);

  move(command, speed);
}

void loop() {
  WiFiClient client = wifiServer.available();
  if (client) {
    Serial.println("Client connected");
    while (client.connected()) {
      if (client.available()) {
        String received = client.readStringUntil('\n');
        received.trim();

        if (!received.isEmpty()) {
          Serial.println("Received: " + received);
          parseAndExecute(received);
        }
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}