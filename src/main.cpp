#include <Arduino.h>
#include <ESP32Servo.h>
#include <PS4Controller.h>

int motorLeftPin = 18;
int motorRightPin = 19;

Servo motorLeft;
Servo motorRight;

void setup() {
  Serial.begin(115200);
  PS4.begin();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  motorLeft.setPeriodHertz(50);
  motorRight.setPeriodHertz(50);

  motorLeft.attach(motorLeftPin);
  motorRight.attach(motorRightPin);

  Serial.println("Ready.");
}

void loop() {
  if (PS4.isConnected()) {
    Serial.println("Controller connected.");
  }

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