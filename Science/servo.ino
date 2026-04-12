#include <ESP32Servo.h>

Servo myServo;

int servoPin = 26;

void setup() {
  myServo.attach(servoPin);

  pinMode(25, OUTPUT);   // set pin 25 as output
  digitalWrite(25, HIGH);
}

void loop() {

  myServo.write(0);
  delay(2000);
  myServo.write(180);
  delay(2000);

}