#include <Arduino.h>
#include <Wire.h>
#include "Kalman.h"
#include "main.h"
#include "PID_v1.h"
#include <Servo.h>


//const int MPU_addr=0x68;  // I2C address of the MPU-6050

Kalman kalman;
cfg_t cfg;
int i =0;

int output2;

int state =1;
void setup(){
  setValues();
  Serial.begin(9600);
  MPU_setup();
  //setup_encoder();
  motor_setup();
  stopAndReset();
  testTimer = millis();
  nTimer = millis();
  n3Timer = millis();
  output2 = 1500;
}

void loop(){
  MPU_update();
  input = (double)pitch;
  uint32_t timer = micros();
  updatePID();
  if(millis()-testTimer>=1000){
    Serial.println("Kalman Value" + (String)pitch);
    Serial.println("PID Value:" + (String)output);
    testTimer = millis();
  }

  //updateEncoder();
}

void setValues(){
  cfg.accYzero = 0.0f;
  cfg.accZzero = 0.0f;
  cfg.backToSpot = 0;
  cfg.controlAngleLimit = 45;
  cfg.P = 10.5f;//13.24;//40.0f;//28.80f;//21.3f;//40.0f;
  cfg.I = 170.0f;//200.0f;//139.25;//435.0f;//650.0f;//650.0f;//385.0f;//300
  cfg.D = 0.35f;//0.414f;//1.5f;//0.98f;//0.7;//1.5f;
  cfg.targetAngle = 269.60f;
  cfg.turningLimit = 2;
//P:20I:0.005D:8.0
}
