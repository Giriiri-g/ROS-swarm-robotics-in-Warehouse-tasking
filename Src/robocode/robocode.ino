#include <WiFi.h>

const char* ssid = "Giriirig";
const char* password = "password";

WiFiServer wifiServer(80);

int m1p1 = 26, m1p2 = 25, m2p1 = 13, m2p2 = 27, led = 2;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  
  wifiServer.begin();

  pinMode(m1p1, OUTPUT);
  pinMode(m1p2, OUTPUT);
  pinMode(m2p1, OUTPUT);
  pinMode(m2p2, OUTPUT);
  pinMode(led, OUTPUT);
}

void move(char command, int speed) {
  switch (command) {
    case 'W':
      digitalWrite(m1p2, LOW);
      digitalWrite(m1p1, HIGH);
      digitalWrite(m2p2, LOW);
      digitalWrite(m2p1, HIGH);
      delay(speed);
      relax();
      break;
    case 'S':
      digitalWrite(m1p1, LOW);
      digitalWrite(m1p2, HIGH);
      digitalWrite(m2p1, LOW);
      digitalWrite(m2p2, HIGH);
      delay(speed);
      relax();
      break;
    case 'D':
      digitalWrite(m1p1, HIGH);
      digitalWrite(m1p2, LOW);
      digitalWrite(m2p1, LOW);
      digitalWrite(m2p2, HIGH);
      delay(speed);
      relax();
      move('W', 1300);
      break;
    case 'A':
      digitalWrite(m1p2, HIGH);
      digitalWrite(m1p1, LOW);
      digitalWrite(m2p2, LOW);
      digitalWrite(m2p1, HIGH);
      delay(speed);
      relax();
      move('W', 1300);
      break;
    default:
      return;
  }
}

void relax(){
  digitalWrite(m1p1, LOW);
  digitalWrite(m1p2, LOW);
  digitalWrite(m2p1, LOW);
  digitalWrite(m2p2, LOW);
}

void parseAndExecute(String data) {
  data.replace("(", "");
  data.replace(")", "");
  
  int firstEnd = data.indexOf(',');
  int secondStart = data.indexOf(',', firstEnd + 1) + 2;
  char command = data.charAt(secondStart);

  int speedStart = data.indexOf(',', secondStart) + 2;
  int speedEnd = data.indexOf(',', speedStart);
  if (speedEnd == -1) speedEnd = data.length();

  int speed = data.substring(speedStart, speedEnd).toInt();
  digitalWrite(led, HIGH);
  delay(200);
  digitalWrite(led, LOW);
  move(command, speed);
}

void loop() {
  WiFiClient client = wifiServer.available();
  if (client) {
    while (client.connected()) {
      if (client.available()) {
        String received = client.readStringUntil('\n');
        received.trim();
        parseAndExecute(received);
      }
    }
    client.stop();
  }
}
