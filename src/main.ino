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
//PID anglePID(&input, &output, &setpoint, (double)cfg.P, (double)cfg.I, (double)cfg.D, DIRECT);

void setup(){
  setValues();
  Serial.begin(9600);
  //anglePID.SetTunings((double)cfg.P, (double)cfg.I, (double)cfg.D);
  //anglePID.SetSampleTime(10);
  setpoint = cfg.targetAngle;
  //anglePID.SetOutputLimits(-400, 400);
  //anglePID.SetMode(AUTOMATIC);
  MPU_setup();
  //setup_encoder();
  motor_setup();
  stopAndReset();
  testTimer = millis();
  nTimer = millis();
  n3Timer = millis();
  testTimer2 = millis();
  //Sets the PID Timer so the computation is executed in the first iteration
  PIDTimer = micros() - 2000;
}

void loop(){
  MPU_update();
  input = (double)pitch;
  uint32_t timer = micros();
  //anglePID.Compute();
  if(millis()-testTimer>=1000){
    Serial.println("Kalman Value" + (String)pitch);
    Serial.println("PID Value:" + (String)output);
    testTimer = millis();
  }
  updatePID(0.0,0.0);
  //drive_motor(1700,1700);
  /*
  int i = 0;
  for(;i<150;i+=10){
    Serial.println("val = " + (String)(1500-i));
    drive_motor(1400,1500);
    delay(1500);
  }
  */
  //left.writeMicroseconds(1600);
  //drive_motor(1500+output, 1500+output);
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
  cfg.SAMPLETIME = 8000; //Sample time set to 2 ms
  cfg.accYzero = 0.0f;
  cfg.accZzero = 0.0f;
  cfg.backToSpot = 0;
  cfg.controlAngleLimit = 45;
  cfg.P = 9.25f;//21.3f;//28.0f;//25.20f;  //28.0f;//21.3f;//40.0f;
  cfg.I = 77.25f;//440.0f;//122.0f;//135.0f;//20.0f; //440.0f;//650.0f;//385.0f;//300
  cfg.D = 0.370f;//0.92f;//0.22f;//0.55f;//0.35f; //0.92f;//0.7;//1.5f;
  cfg.targetAngle = 269.12f;
  cfg.rightMotorForwardOffset = 78;
  cfg.rightMotorReverseOffset = 85;
  cfg.leftMotorForwardOffset = 70;
  cfg.leftMotorReverseOffset = 75;
  cfg.PIDMIN = -400;
  cfg.PIDMAX = 400;
//P:20I:0.005D:8.0
}
