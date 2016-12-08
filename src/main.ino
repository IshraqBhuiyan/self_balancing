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
double setpoint, input, output;
PID anglePID(&input, &output, &setpoint, (double)cfg.P, (double)cfg.I, (double)cfg.D, DIRECT);
int output2;
int state =1;
uint32_t testTimer3, tt4;
void setup(){
  setValues();
  Serial.begin(9600);
  anglePID.SetTunings((double)cfg.P, (double)cfg.I, (double)cfg.D);
  anglePID.SetSampleTime(3);
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
  output2 = 1500;
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
  if(millis()-testTimer2 <=2000){
    setpoint = cfg.targetAngle;
  }else if(millis()-testTimer2 <=5000){
    setpoint = cfg.targetAngle + 0.8f;
  }else{
    testTimer2 = millis();
  }
  if(false&&millis()-testTimer2>1000){
    drive_motor(output2,output2);
    output2-=10;
    Serial.println("Output2: " + (String)output2);
    testTimer2=millis();
  }
  if(abs(pitch-cfg.targetAngle)>=30){
    drive_motor(1500,1500);
  }else{
    drive_motor(1500+output, 1500+output);
  }
  //updatePID(cfg.targetAngle, 0, 0,(float)(timer-PIDTimer)/1000000.0f);
  //updateEncoder();
}

void setValues(){
  cfg.accYzero = 0.0f;
  cfg.accZzero = 0.0f;
  cfg.backToSpot = 0;
  cfg.controlAngleLimit = 45;
  cfg.P = 10.5f;//13.24;//40.0f;//28.80f;//21.3f;//40.0f;
  cfg.I = 170.0f;//200.0f;//139.25;//435.0f;//650.0f;//650.0f;//385.0f;//300
  cfg.D = 0.45f;//0.414f;//1.5f;//0.98f;//0.7;//1.5f;
  cfg.targetAngle = 269.60f;
  cfg.turningLimit = 2;
//P:20I:0.005D:8.0
}
