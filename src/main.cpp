#include <Arduino.h>
#include <PS4Controller.h>
#include "esp_gap_bt_api.h"

//GPIO definition for drive control
const uint8_t MOTOR_L_1_GPIO = 23;
const uint8_t MOTOR_L_2_GPIO = 22;
const uint8_t MOTOR_R_1_GPIO = 19;
const uint8_t MOTOR_R_2_GPIO = 18;

//machine settings
const uint8_t MAX_DRIVE_SPEED_L = 255;  //maximum drive speed left track (duty cycle, absolut maximum is 255)
const uint8_t MAX_DRIVE_SPEED_R = 255;  //maximum drive speed right track (duty cycle, absolut maximum is 255)
const uint8_t DRIVE_SPEED_BACK_L = 200; //fixed speed for driving backwards left track (duty cycle, absolut maximum is 255)
const uint8_t DRIVE_SPEED_BACK_R = 200; //fixed speed for driving backwards right track (duty cycle, absolut maximum is 255)
const uint8_t DRIVE_L_FWD_START = 0;  //duty cycle for movement start left forward
const uint8_t DRIVE_R_FWD_START = 0;  //duty cycle for movement start right forward

//PWM settings
const int MOTOR_L_1_PWM_CHANNEL = 0;
const int MOTOR_L_2_PWM_CHANNEL = 1;
const int MOTOR_R_1_PWM_CHANNEL = 2;
const int MOTOR_R_2_PWM_CHANNEL = 3;
const int PWM_FREQ = 30000;
const int PWM_RESOLUTION = 8; //max duty cycle is 255 because of this resolution

//Drive commands
uint8_t driveCmdLeft = 0;
uint8_t driveCmdRight = 0;
int8_t driveDirLeft = 0;
int8_t driveDirRight = 0;

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

  //init pwm outputs drive
  ledcSetup(MOTOR_L_1_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(MOTOR_L_2_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(MOTOR_R_1_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(MOTOR_R_2_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_L_1_GPIO, MOTOR_L_1_PWM_CHANNEL);
  ledcAttachPin(MOTOR_L_2_GPIO, MOTOR_L_2_PWM_CHANNEL);
  ledcAttachPin(MOTOR_R_1_GPIO, MOTOR_R_1_PWM_CHANNEL);
  ledcAttachPin(MOTOR_R_2_GPIO, MOTOR_R_2_PWM_CHANNEL);

  Serial.println("Waiting for connection of PS4 controller...");
}

//calc drive command from axis value
uint8_t getDriveFwdCommand(uint8_t axis, uint8_t driveStart, uint8_t maxDriveSpeed) {
  return map(axis, 0, 255, driveStart, maxDriveSpeed);
}

//command motor driver
void commandMotor(uint8_t driveCmd, int8_t driveDir, uint8_t motorPWM1, uint8_t motorPWM2) {
  //set direction
  switch (driveDir)
  {
    case 1:
      //drive forward
      ledcWrite(motorPWM1, driveCmd);
      ledcWrite(motorPWM2, 0);
      break;
      
    case -1:
      //drive backwards
      ledcWrite(motorPWM1, 0);
      ledcWrite(motorPWM2, driveCmd);
      break;

    default:
      //stop drive
      ledcWrite(motorPWM1, 0);
      ledcWrite(motorPWM2, 0);
      break;
  }
}

//main program loop
void loop() {
  driveCmdLeft = 0;
  driveCmdRight = 0;
  driveDirLeft = 0;
  driveDirRight = 0;

  //check first BT connection of controller
  if (PS4.isConnected() && controller_connected == false) {
    Serial.println("Controller connected");
    controller_connected = true;
  }

  //check axis on controller
  if (PS4.isConnected() && controller_connected == true){
    if (PS4.L2()) {
      driveCmdLeft = getDriveFwdCommand(PS4.L2Value(), DRIVE_L_FWD_START, MAX_DRIVE_SPEED_L);
      driveDirLeft = 1;
    }
    if (PS4.L1()){
      driveCmdLeft = DRIVE_SPEED_BACK_L;
      driveDirLeft = -1;
    }
    if (PS4.R2()) {
      driveCmdRight = getDriveFwdCommand(PS4.R2Value(), DRIVE_R_FWD_START, MAX_DRIVE_SPEED_R);
      driveDirRight = 1;
    }
    if (PS4.R1()){
      driveCmdRight = DRIVE_SPEED_BACK_R;
      driveDirRight = -1;
    }

    Serial.printf("\rLeft axis/cmd/dir: %3d - %3d - %3d | Right axis/cmd/dir: %3d - %3d - %3d", PS4.L2Value(), driveCmdLeft, driveDirLeft, PS4.R2Value(), driveCmdRight, driveDirRight);
  }

  //command drive motors
  commandMotor(driveCmdLeft, driveDirLeft, MOTOR_L_1_PWM_CHANNEL, MOTOR_L_2_PWM_CHANNEL);
  commandMotor(driveCmdRight, driveDirRight, MOTOR_R_1_PWM_CHANNEL, MOTOR_R_2_PWM_CHANNEL);
}