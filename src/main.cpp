#include <Arduino.h>
#include <ESP32Servo.h>
#include <PS4Controller.h>
#include "esp_gap_bt_api.h"

//drive servos and commands
uint8_t motorLeftPin = 18;
uint8_t motorRightPin = 19;
Servo motorLeft;
Servo motorRight;
uint8_t commandLeft = 90;
uint8_t commandRight = 90;

//BT settings and status
const char* BT_MAC = "7c:9e:bd:06:28:7a";
bool controller_connected = false;

//machine settings
const float MAX_SPEED = 25.0;

//unbinds all BT devices
void unbindAllBT() {
  uint8_t pairedDeviceBtAddr[20][6];  
  int count = esp_bt_gap_get_bond_device_num();
  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
  for(int i = 0; i < count; i++) 
  {
    esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
  }
  Serial.println("Unbound all BT devices");
}

//ESP32 setup function
void setup() {
  Serial.begin(115200);
  PS4.begin(BT_MAC);

  //unbind all earlier BT devices
  unbindAllBT();

  //start timers for PWM
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  //setup PWM outputs for drive
  motorLeft.setPeriodHertz(50);
  motorRight.setPeriodHertz(50);
  motorLeft.attach(motorLeftPin);
  motorRight.attach(motorRightPin);

  //reset drive to zero speed
  motorLeft.write(90);
  motorRight.write(90);

  Serial.println("Waiting for connection of PS4 controller...");
}

//calc drive command from axis value
uint8_t getDriveCommand(uint8_t axis, bool left) {
  uint8_t command = ((float) axis / 255.0) * MAX_SPEED;
  if (left)
  {
    return 90 - command;
  }
  return 90 + command;
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
      commandLeft = getDriveCommand(PS4.L2Value(), true);
    }
    if (PS4.R2()) {
      commandRight = getDriveCommand(PS4.R2Value(), false);
    }

    Serial.printf("\rLeft axis: %3d | Left motor: %3d | Right axis: %3d | Right motor: %3d", PS4.L2Value(), commandLeft, PS4.R2Value(), commandRight);
  }

  motorLeft.write(commandLeft);
  motorRight.write(commandRight);
}