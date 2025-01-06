#include <Arduino.h>
#include <PS4Controller.h>
#include "esp_gap_bt_api.h"

//GPIO definitions
const uint8_t MOTOR_L_1_GPIO = 23;
const uint8_t MOTOR_L_2_GPIO = 22;
const uint8_t MOTOR_R_1_GPIO = 19;
const uint8_t MOTOR_R_2_GPIO = 18;
const uint8_t TURRENT_ROTATE_GPIO = 5;
const uint8_t TURRENT_ELEVATE_GPIO = 4;

//machine settings
const uint8_t MAX_DRIVE_SPEED_L = 75;  //maximum drive speed left track (duty cycle, absolut maximum is 255)
const uint8_t MAX_DRIVE_SPEED_R = 75;  //maximum drive speed right track (duty cycle, absolut maximum is 255)
const uint8_t DRIVE_SPEED_BACK_L = 60; //fixed speed for driving backwards left track (duty cycle, absolut maximum is 255)
const uint8_t DRIVE_SPEED_BACK_R = 60; //fixed speed for driving backwards right track (duty cycle, absolut maximum is 255)
const uint8_t TURRENT_ROTATE_STOP = 19; //duty cycle for middle position of servo for rotating turrent
const uint8_t DRIVE_JOY_DEATHBAND = 40; //deathband of left joystick axis for driving
const uint8_t TURRENT_JOY_DEATHBAND = 40; //deathband of right joystick axis for rotating and elevating turrent
const uint8_t TURRENT_ELEVATE_MIN = 8; //minimum duty cycle for turrent elevation
const uint8_t TURRENT_ELEVATE_MAX = 30; //maximum duty cycle for turrent elevation
const uint8_t TURRENT_ELEVATE_DFLT = TURRENT_ELEVATE_MIN + (TURRENT_ELEVATE_MAX - TURRENT_ELEVATE_MIN) / 2;

//PWM settings for drive control and servos
const int MOTOR_L_1_PWM_CHANNEL = 0;
const int MOTOR_L_2_PWM_CHANNEL = 1;
const int MOTOR_R_1_PWM_CHANNEL = 2;
const int MOTOR_R_2_PWM_CHANNEL = 3;
const int TURRENT_ROTATE_PWM_CHANNEL = 4;
const int TURRENT_ELEVATE_PWM_CHANNEL = 5;
const int PWM_FREQ_DRIVE = 30000; //pwm frequency for drive control
const int PWM_FREQ_SERVO = 50; //pwm frequency for servos
const int PWM_RESOLUTION = 8; //max duty cycle is 255 because of this resolution

//Commands for driving and servos (pwm duty cycle)
uint8_t driveCmdLeft = 0;
uint8_t driveCmdRight = 0;
int8_t driveDirLeft = 0;
int8_t driveDirRight = 0;
uint8_t turrentRotateCmd = 0;
uint8_t turrentElevateCmd = TURRENT_ELEVATE_DFLT;
long elevationTimestamp = 0;

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

  //init pwm outputs drive and servos
  ledcSetup(MOTOR_L_1_PWM_CHANNEL, PWM_FREQ_DRIVE, PWM_RESOLUTION);
  ledcSetup(MOTOR_L_2_PWM_CHANNEL, PWM_FREQ_DRIVE, PWM_RESOLUTION);
  ledcSetup(MOTOR_R_1_PWM_CHANNEL, PWM_FREQ_DRIVE, PWM_RESOLUTION);
  ledcSetup(MOTOR_R_2_PWM_CHANNEL, PWM_FREQ_DRIVE, PWM_RESOLUTION);
  ledcSetup(TURRENT_ROTATE_PWM_CHANNEL, PWM_FREQ_SERVO, PWM_RESOLUTION);
  ledcSetup(TURRENT_ELEVATE_PWM_CHANNEL, PWM_FREQ_SERVO, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_L_1_GPIO, MOTOR_L_1_PWM_CHANNEL);
  ledcAttachPin(MOTOR_L_2_GPIO, MOTOR_L_2_PWM_CHANNEL);
  ledcAttachPin(MOTOR_R_1_GPIO, MOTOR_R_1_PWM_CHANNEL);
  ledcAttachPin(MOTOR_R_2_GPIO, MOTOR_R_2_PWM_CHANNEL);
  ledcAttachPin(TURRENT_ROTATE_GPIO, TURRENT_ROTATE_PWM_CHANNEL);
  ledcAttachPin(TURRENT_ELEVATE_GPIO, TURRENT_ELEVATE_PWM_CHANNEL);

  Serial.println("Waiting for connection of PS4 controller...");
}

//calc drive command from L2 or R2 value
uint8_t getDriveCommand(uint8_t axis, uint8_t maxDriveSpeed) {
  return map(axis, 0, 255, 0, maxDriveSpeed);
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
  turrentRotateCmd = TURRENT_ROTATE_STOP;

  //check first BT connection of controller
  if (PS4.isConnected() && controller_connected == false) {
    Serial.println("Controller connected");
    controller_connected = true;
  }

  //check axis on controller
  if (PS4.isConnected() && controller_connected == true){
    //check drive controls (driving is possible by L1/2 and R1/2 or by left joystick)
    if (PS4.L2()){
      driveCmdLeft = getDriveCommand(PS4.L2Value(), MAX_DRIVE_SPEED_L);
      driveDirLeft = -1;
    }
    if (PS4.L1()){
      driveCmdLeft = DRIVE_SPEED_BACK_L;
      driveDirLeft = 1;
    }
    if (PS4.R2()){
      driveCmdRight = getDriveCommand(PS4.R2Value(), MAX_DRIVE_SPEED_R);
      driveDirRight = 1;
    }
    if (PS4.R1()){
      driveCmdRight = DRIVE_SPEED_BACK_R;
      driveDirRight = -1;
    }

    //check turrent controls (control by right joystick)
    if (abs(PS4.RStickX()) >= TURRENT_JOY_DEATHBAND){
      //this servo has a very narrow working duty cycle because of modification to endless servo movement
      turrentRotateCmd = map(PS4.RStickX(), -127, 127, TURRENT_ROTATE_STOP - 5, TURRENT_ROTATE_STOP + 5);
    }
    if (abs(PS4.RStickY()) >= TURRENT_JOY_DEATHBAND && (millis() - elevationTimestamp > 50)){
      if (PS4.RStickY() > 0){
        turrentElevateCmd++;

      }
      else {
        turrentElevateCmd--;
      }
      turrentElevateCmd = min(turrentElevateCmd, TURRENT_ELEVATE_MAX);
      turrentElevateCmd = max(turrentElevateCmd, TURRENT_ELEVATE_MIN);
      elevationTimestamp = millis();
    }

    //Debugging
    Serial.printf("\rRight stick x/y/cmdx/cmdy: %3d - %3d - %3d - %3d | Left axis/cmd/dir: %3d - %3d - %3d | Right axis/cmd/dir: %3d - %3d - %3d", PS4.RStickX(), PS4.RStickY(), turrentRotateCmd, turrentElevateCmd, PS4.L2Value(), driveCmdLeft, driveDirLeft, PS4.R2Value(), driveCmdRight, driveDirRight);
  }

  //command drive motors
  commandMotor(driveCmdLeft, driveDirLeft, MOTOR_L_1_PWM_CHANNEL, MOTOR_L_2_PWM_CHANNEL);
  commandMotor(driveCmdRight, driveDirRight, MOTOR_R_1_PWM_CHANNEL, MOTOR_R_2_PWM_CHANNEL);

  //command servos
  ledcWrite(TURRENT_ROTATE_PWM_CHANNEL, turrentRotateCmd);
  ledcWrite(TURRENT_ELEVATE_PWM_CHANNEL, turrentElevateCmd);
}
