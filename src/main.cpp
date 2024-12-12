#include <Arduino.h>
#include <PS4Controller.h>
#include "esp_gap_bt_api.h"

//GPIO definition for drive control
const uint8_t MOTOR_L_SPEED_GPIO = 18;
const uint8_t MOTOR_L_DIR1_GPIO = 22;
const uint8_t MOTOR_L_DIR2_GPIO = 23;
const uint8_t MOTOR_R_SPEED_GPIO = 19;
const uint8_t MOTOR_R_DIR1_GPIO = 4;
const uint8_t MOTOR_R_DIR2_GPIO = 5;

//machine settings
const uint8_t MAX_DRIVE_SPEED = 255;
const uint8_t DRIVE_SPEED_BACK = 200;

//PWM settings
const int MOTOR_L_PWM_CHANNEL = 0;
const int MOTOR_R_PWM_CHANNEL = 1;
const int PWM_FREQ = 500;
const int PWM_RESOLUTION = 8; //so max duty cycle is 255

//Drive commands
uint8_t commandLeft = 0;
uint8_t commandRight = 0;
int8_t directionLeft = 0;
int8_t directionRight = 0;

//BT settings and status
const char* BT_MAC = "7c:9e:bd:06:28:7a";
bool controller_connected = false;

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
  Serial.begin(115200); //setup serial monitor
  
  PS4.begin(BT_MAC);  //setup BT on MAC adress which is predefined on controller by SixAxisPairTool
  unbindAllBT();  //unbind all earlier BT devices

  //init digital drive GPIOs
  pinMode(MOTOR_L_DIR1_GPIO, OUTPUT);
  pinMode(MOTOR_L_DIR2_GPIO, OUTPUT);
  pinMode(MOTOR_R_DIR1_GPIO, OUTPUT);
  pinMode(MOTOR_R_DIR2_GPIO, OUTPUT);

  //init pwm outputs drive
  ledcSetup(MOTOR_L_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(MOTOR_R_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_L_SPEED_GPIO, MOTOR_L_PWM_CHANNEL);
  ledcAttachPin(MOTOR_R_SPEED_GPIO, MOTOR_R_PWM_CHANNEL);

  Serial.println("Waiting for connection of PS4 controller...");
}

//calc drive command from axis value
uint8_t getDriveFwdCommand(uint8_t axis) {
  return map(axis, 0, 255, 0 , MAX_DRIVE_SPEED);
}

//command motor driver
void commandMotor(uint8_t speed, uint8_t pwmChannel, int8_t dir, uint8_t dir1GPIO, uint8_t dir2GPIO) {
  //set direction
  switch (dir)
  {
    case 1:
      digitalWrite(dir1GPIO, 1);
      digitalWrite(dir2GPIO, 0);
      break;
      
    case -1:
      digitalWrite(dir1GPIO, 0);
      digitalWrite(dir2GPIO, 1);
      break;

    default:
      digitalWrite(dir1GPIO, 0);
      digitalWrite(dir2GPIO, 0);
      break;
  }

  //set speed
  ledcWrite(pwmChannel, speed);
}

//main program loop
void loop() {
  commandLeft = 0;
  commandRight = 0;
  directionLeft = 0;
  directionRight = 0;

  //check first BT connection of controller
  if (PS4.isConnected() && controller_connected == false) {
    Serial.println("Controller connected");
    controller_connected = true;
  }

  //check axis on controller
  if (PS4.isConnected() && controller_connected == true){
    if (PS4.L2()) {
      commandLeft = getDriveFwdCommand(PS4.L2Value());
      directionLeft = 1;
    }
    if (PS4.L1()){
      commandLeft = DRIVE_SPEED_BACK;
      directionLeft = -1;
    }
    if (PS4.R2()) {
      commandRight = getDriveFwdCommand(PS4.R2Value());
      directionRight = 1;
    }
    if (PS4.R1()){
      commandRight = DRIVE_SPEED_BACK;
      directionRight = -1;
    }

    Serial.printf("\rLeft axis/cmd/dir: %3d - %3d - %3d | Right axis/cmd/dir: %3d - %3d - %3d", PS4.L2Value(), commandLeft, directionLeft, PS4.R2Value(), commandRight, directionRight);
  }

  //command drive motors
  commandMotor(commandLeft, MOTOR_L_PWM_CHANNEL, directionLeft, MOTOR_L_DIR1_GPIO, MOTOR_L_DIR2_GPIO);
  commandMotor(commandRight, MOTOR_R_PWM_CHANNEL, directionRight, MOTOR_R_DIR1_GPIO, MOTOR_R_DIR2_GPIO);
}