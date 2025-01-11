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
const uint8_t TURRENT_SHOOT_GPIO = 2;

//machine settings
const uint8_t MAX_DRIVE_SPEED_L = 85;  //maximum drive speed left track (duty cycle, absolut maximum is 255)
const uint8_t MAX_DRIVE_SPEED_R = 85;  //maximum drive speed right track (duty cycle, absolut maximum is 255)
const uint8_t DRIVE_SPEED_BACK_L = 70; //fixed speed for driving backwards left track (duty cycle, absolut maximum is 255)
const uint8_t DRIVE_SPEED_BACK_R = 70; //fixed speed for driving backwards right track (duty cycle, absolut maximum is 255)
const uint16_t TURRENT_ROTATE_STOP = 305; //duty cycle for middle position of servo for rotating turrent
const uint8_t DRIVE_JOY_DEATHBAND = 40; //deathband of left joystick axis for driving
const uint8_t TURRENT_JOY_DEATHBAND = 50; //deathband of right joystick axis for rotating and elevating turrent
const uint16_t TURRENT_ELEVATE_MIN = 106; //minimum duty cycle for turrent elevation
const uint16_t TURRENT_ELEVATE_MAX = 435; //maximum duty cycle for turrent elevation
const uint16_t TURRENT_ELEVATE_DFLT = TURRENT_ELEVATE_MIN + (TURRENT_ELEVATE_MAX - TURRENT_ELEVATE_MIN) / 2;

//PWM settings for drive control and servos
const uint8_t MOTOR_L_1_PWM_CHANNEL = 0;
const uint8_t MOTOR_L_2_PWM_CHANNEL = 1;
const uint8_t MOTOR_R_1_PWM_CHANNEL = 2;
const uint8_t MOTOR_R_2_PWM_CHANNEL = 3;
const uint8_t TURRENT_ROTATE_PWM_CHANNEL = 4;
const uint8_t TURRENT_ELEVATE_PWM_CHANNEL = 5;
const uint32_t PWM_FREQ_DRIVE = 30000; //pwm frequency for drive control
const uint8_t PWM_FREQ_SERVO = 50; //pwm frequency for servos
const uint8_t PWM_RESOLUTION_DRIVE = 8; //max duty cycle is 255 because of this resolution
const uint8_t PWM_RESOLUTION_SERVO = 12; //max duty cycle is 4095 because of this resolution

//Commands for driving and servos (pwm duty cycle)
uint8_t driveCmdLeft = 0;
uint8_t driveCmdRight = 0;
int8_t driveDirLeft = 0;
int8_t driveDirRight = 0;
uint16_t turrentRotateCmd = 0;
uint16_t turrentElevateCmd = TURRENT_ELEVATE_DFLT;
long rotateLeftTimestamp = 0;
long rotateRightTimestamp = 0;
bool turrentRotateLeft = false;
bool turrentRotateRight = false;
long elevationTimestamp = 0;
bool turrentElevateUp = false;
bool turrentElevateDown = false;
uint8_t shootCmd = LOW;

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
  ledcSetup(MOTOR_L_1_PWM_CHANNEL, PWM_FREQ_DRIVE, PWM_RESOLUTION_DRIVE);
  ledcSetup(MOTOR_L_2_PWM_CHANNEL, PWM_FREQ_DRIVE, PWM_RESOLUTION_DRIVE);
  ledcSetup(MOTOR_R_1_PWM_CHANNEL, PWM_FREQ_DRIVE, PWM_RESOLUTION_DRIVE);
  ledcSetup(MOTOR_R_2_PWM_CHANNEL, PWM_FREQ_DRIVE, PWM_RESOLUTION_DRIVE);
  ledcSetup(TURRENT_ROTATE_PWM_CHANNEL, PWM_FREQ_SERVO, PWM_RESOLUTION_SERVO);
  ledcSetup(TURRENT_ELEVATE_PWM_CHANNEL, PWM_FREQ_SERVO, PWM_RESOLUTION_SERVO);
  ledcAttachPin(MOTOR_L_1_GPIO, MOTOR_L_1_PWM_CHANNEL);
  ledcAttachPin(MOTOR_L_2_GPIO, MOTOR_L_2_PWM_CHANNEL);
  ledcAttachPin(MOTOR_R_1_GPIO, MOTOR_R_1_PWM_CHANNEL);
  ledcAttachPin(MOTOR_R_2_GPIO, MOTOR_R_2_PWM_CHANNEL);
  ledcAttachPin(TURRENT_ROTATE_GPIO, TURRENT_ROTATE_PWM_CHANNEL);
  ledcAttachPin(TURRENT_ELEVATE_GPIO, TURRENT_ELEVATE_PWM_CHANNEL);

  //setup shooting output pin
  pinMode(TURRENT_SHOOT_GPIO, OUTPUT);

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
  shootCmd = LOW;
  turrentRotateCmd = TURRENT_ROTATE_STOP;

  //check first BT connection of controller
  if (PS4.isConnected() && controller_connected == false) {
    Serial.println("Controller connected");
    controller_connected = true;
  }

  //check axis on controller
  if (PS4.isConnected() && controller_connected == true){

    //check drive controls (driving is done by L1/2 and R1/2)
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

    //check turrent control rotation by right joystick
    if (abs(PS4.RStickX()) >= TURRENT_JOY_DEATHBAND){
      //this servo has a very narrow working duty cycle because of modification to endless servo movement
      turrentRotateCmd = map(PS4.RStickX(), -127, 127, TURRENT_ROTATE_STOP + 10, TURRENT_ROTATE_STOP - 10);
    }

    //check turrent control rotation by left/right buttons for fine controlling
    if (PS4.Left() || (millis() - rotateLeftTimestamp < 250)){
      if(turrentRotateLeft == false){
        turrentRotateCmd = TURRENT_ROTATE_STOP + 4;
        rotateLeftTimestamp = millis();
        turrentRotateLeft = true;
      }
      if(millis() - rotateLeftTimestamp < 250){
        turrentRotateCmd = TURRENT_ROTATE_STOP + 4;
      }
    }
    else {
      turrentRotateLeft = false;
    }
    if (PS4.Right() || (millis() - rotateRightTimestamp < 250)){
      if(turrentRotateRight == false){
        turrentRotateCmd = TURRENT_ROTATE_STOP - 4;
        rotateRightTimestamp = millis();
        turrentRotateRight = true;
      }
      if(millis() - rotateRightTimestamp < 250){
        turrentRotateCmd = TURRENT_ROTATE_STOP - 4;
      }
    }
    else {
      turrentRotateRight = false;
    }

    //check turrent control elevation by right joystick
    if (abs(PS4.RStickY()) >= TURRENT_JOY_DEATHBAND && (millis() - elevationTimestamp > 15)){
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

    //check turrent control elevation by up/down buttons for fine controlling
    if (PS4.Up()){
      if(turrentElevateUp == false){
        turrentElevateCmd += 4;
        turrentElevateUp = true;
      }
    }
    else {
      turrentElevateUp = false;
    }
    if (PS4.Down()){
      if(turrentElevateDown == false){
        turrentElevateCmd -= 4;
        turrentElevateDown = true;
      }
    }
    else {
      turrentElevateDown = false;
    }

    //check shooting control
    if (PS4.Cross()){
      shootCmd = HIGH;
    }
  }

  //command drive motors
  commandMotor(driveCmdLeft, driveDirLeft, MOTOR_L_1_PWM_CHANNEL, MOTOR_L_2_PWM_CHANNEL);
  commandMotor(driveCmdRight, driveDirRight, MOTOR_R_1_PWM_CHANNEL, MOTOR_R_2_PWM_CHANNEL);

  //command servos
  ledcWrite(TURRENT_ROTATE_PWM_CHANNEL, turrentRotateCmd);
  ledcWrite(TURRENT_ELEVATE_PWM_CHANNEL, turrentElevateCmd);

  //command shooting
  digitalWrite(TURRENT_SHOOT_GPIO, shootCmd);
}
