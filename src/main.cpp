#include <Arduino.h>
#include <ESP32Servo.h>
#include <PS4Controller.h>

uint8_t motorLeftPin = 18;
uint8_t motorRightPin = 19;
Servo motorLeft;
Servo motorRight;

const char* BT_MAC = "fc:01:7c:f4:1a:6c";
bool controller_connected = false;

uint8_t commandLeft = 90;
uint8_t commandRight = 90;

void setup() {
  Serial.begin(115200);
  PS4.begin(BT_MAC);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  motorLeft.setPeriodHertz(50);
  motorRight.setPeriodHertz(50);

  motorLeft.attach(motorLeftPin);
  motorRight.attach(motorRightPin);

  Serial.println("Reset drive");
  motorLeft.write(90);
  motorRight.write(90);

  Serial.println("Waiting for connection of controller...");
}

//calc drive command from axis value
uint8_t getDriveCommand(uint8_t axis) {
  return 90 + ((float) axis / 255.0) * 90.0;
}

void loop() {
  commandLeft = 90;
  commandRight = 90;

  //check first BT connection of controller
  if (PS4.isConnected() && controller_connected == false) {
    Serial.println("Controller connected");
    controller_connected = true;
  }

  //check axis on controller
  if (PS4.isConnected() && controller_connected == true){
    if (PS4.L2()) {
      commandLeft = getDriveCommand(PS4.L2Value());
    }
    if (PS4.R2()) {
      commandRight = getDriveCommand(PS4.R2Value());
    }

    Serial.printf("\rLeft axis: %3d | Left motor: %3d | Right axis: %3d | Right motor: %3d", PS4.L2Value(), commandLeft, PS4.R2Value(), commandRight);
  }

  motorLeft.write(commandLeft);
  motorRight.write(commandRight);
}