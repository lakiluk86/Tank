#include <Arduino.h>
#include <ESP32Servo.h>

int motorLeftPin = 0;
int motorRightPin = 0;

Servo motorLeft;
Servo motorRight;

void setup() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  motorLeft.setPeriodHertz(50);
  motorRight.setPeriodHertz(50);

  motorLeft.attach(motorLeftPin);
  motorRight.attach(motorRightPin);
}

void loop() {
  motorLeft.write(0);
  delay(2000);
  motorLeft.write(40);
  delay(2000);
  motorLeft.write(80);
  delay(2000);
  motorLeft.write(120);
  delay(2000);
  motorLeft.write(160);
}