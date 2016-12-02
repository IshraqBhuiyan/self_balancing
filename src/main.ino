#include <Arduino.h>
#include <Wire.h>
#include "Kalman.h"
#include "main.h"
#include "PID_v1.h"
#include <Servo.h>


//const int MPU_addr=0x68;  // I2C address of the MPU-6050
//int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
Kalman kalman;
cfg_t cfg;
int i =0;
double setpoint, input, output;
PID anglePID(&input, &output, &setpoint, (double)cfg.P, (double)cfg.I, (double)cfg.D, DIRECT);

void setup(){
  setValues();
  Serial.begin(9600);
  anglePID.SetTunings((double)cfg.P, (double)cfg.I, (double)cfg.D);
  anglePID.SetSampleTime(10);
  setpoint = cfg.targetAngle;
  anglePID.SetOutputLimits(-400, 400);
  anglePID.SetMode(AUTOMATIC);
  MPU_setup();
  //setup_encoder();
  motor_setup();
  stopAndReset();
  testTimer = millis();
  nTimer = millis();
  n3Timer = millis();
  testTimer2 = millis();
}

void loop(){
  MPU_update();
  input = (double)pitch;
  uint32_t timer = micros();
  anglePID.Compute();
  if(millis()-testTimer>=1000){
    Serial.println("Kalman Value" + (String)pitch);
    Serial.println("PID Value:" + (String)output);
    testTimer = millis();
  }
  if(false&&(pitch - cfg.targetAngle) <0){
    drive_motor(1500-output, 1500-output);
  }else if(false){
    drive_motor(1500+output, 1500+output);
  }
  drive_motor(1500+output, 1500+output);
  //updatePID(cfg.targetAngle, 0, 0,(float)(timer-PIDTimer)/1000000.0f);
  //drive_motor(1900, 1500);
  //updateEncoder();
  //uint32_t currTime = millis();
  //if(currTime-testTimer2 >= 1000){
  //  Serial.println("Left Velocity: " + (String)leftCounter);
  //  Serial.println("Right Velocity: " + (String)rightCounter);
  //  testTimer2 = currTime;
  //}
}

void setValues(){
  cfg.accYzero = 0.0f;
  cfg.accZzero = 0.0f;
  cfg.backToSpot = 0;
  //cfg.bindSpektrum = false;
  cfg.controlAngleLimit = 45;
  cfg.P = 55.00f;
  cfg.I = 0.0f;
  cfg.D = 30.0f;
  cfg.leftMotorScaler = 1.0f;
  cfg.rightMotorScaler = 0.97f;
  cfg.targetAngle = 274.50f;
  cfg.turningLimit = 2;
//P:20I:0.005D:8.0
}
