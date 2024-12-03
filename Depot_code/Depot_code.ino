#include <Servo.h>

const int USt1 = 3;
const int USe1 = 4;
const int USt2 = 5;
const int USe2 = 6;
const int USt3 = 7;
const int USe3 = 8;

Servo S1;
Servo S2;
Servo S3;

void setup() {
  pinMode(USt1, OUTPUT);
  pinMode(USe1, INPUT);
  pinMode(USt2, OUTPUT);
  pinMode(USe2, INPUT);
  pinMode(USt3, OUTPUT);
  pinMode(USe3, INPUT);

  S1.attach(9);
  S2.attach(10);
  S3.attach(11);

  Serial.begin(9600);
}

void loop() {
  float distance1 = measureDistance(USt1, USe1);
  float distance2 = measureDistance(USt2, USe2);
  float distance3 = measureDistance(USt3, USe3);

  Serial.print("Distance 1: ");
  Serial.print(distance1);
  Serial.println(" cm");
  Serial.print("Distance 2: ");
  Serial.print(distance2);
  Serial.println(" cm");
  Serial.print("Distance 3: ");
  Serial.print(distance3);
  Serial.println(" cm");

  controlServo(S1, distance1);
  controlServo(S2, distance2);
  controlServo(S3, distance3);

  delay(500);
}

float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = (duration * 0.034) / 2;
  return distance;
}

void controlServo(Servo& servo, float distance) {
  if (distance < 20) {
    servo.write(90);
  } else {
    servo.write(0);
  }
}
