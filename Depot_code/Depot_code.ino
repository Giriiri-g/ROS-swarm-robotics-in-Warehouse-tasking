#include <Servo.h>

const int USt1 = 2;
const int USe1 = 3;
const int USt2 = 4;
const int USe2 = 5;
const int USt3 = 6;
const int USe3 = 7;

Servo S1;
Servo S2;
Servo S3;

int counter1 = 0;
int counter2 = 0;
int counter3 = 0;

const int threshold = 10;    // Distance threshold in cm
const int triggerCount = 20; // Number of consecutive readings required

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

  handleServoControl(S1, distance1, counter1);
  handleServoControl(S2, distance2, counter2);
  handleServoControl(S3, distance3, counter3);

  delay(50); // Adjust delay as needed
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

void handleServoControl(Servo& servo, float distance, int& counter) {
  if (distance <= threshold) {
    counter++;
    if (counter >= triggerCount) {
      servo.write(90); // Move servo
      delay(1000);     // Optional delay for servo action
      servo.write(0);  // Reset servo to original position
      counter = 0;     // Reset counter
    }
  } else {
    counter = 0; // Reset counter if distance is above threshold
  }
}