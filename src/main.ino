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
float Pspeed, Ispeed, Dspeed, speedDifference, SMoffset;
double speedTarget = 0;
PID anglePID(&input, &output, &setpoint, (double)cfg.P, (double)cfg.I, (double)cfg.D, DIRECT);
PID speedPID(&speedDifference, &SMoffset, &speedTarget, (double)Pspeed, (double)Ispeed, (double)Dspeed, DIRECT);

int state =1;
uint32_t testTimer3, tt4;
float leftMultiplier = 1.0;
float rightMultiplier = 1.0;


void setup(){
  setValues();
  Serial.begin(9600);
  anglePID.SetTunings((double)cfg.P, (double)cfg.I, (double)cfg.D);
  anglePID.SetSampleTime(10);
  setpoint = cfg.targetAngle;
  anglePID.SetOutputLimits(-400, 400);
  anglePID.SetMode(AUTOMATIC);
  MPU_setup();
  speedDifference = 0.0;
  SMoffset = 0.0;
  speedPID.SetTunings(Pspeed,Ispeed,Dspeed);
  speedPID.SetSampleTime(10);
  speedPID.SetOutputLimits(-1,1);
  speedPID.SetMode(AUTOMATIC);

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
  speedDifference = leftVelocity - rightVelocity;
  speedPID.Compute();
  /*if(false&&(pitch - cfg.targetAngle) <0){
    drive_motor(1500-output, 1500-output);
  }else if(false){
    drive_motor(1500+output, 1500+output);
  }*/
  leftMultiplier = constrain(leftMultiplier + SMoffset, 0.0,2.0);
  rightMultiplier = constrain(rightMultiplier - SMoffset, 0.0,2.0);
  drive_motor(1500+leftMultiplier*output, 1500+leftMultiplier*output);
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
  cfg.P = 28.80f;//21.3f;//40.0f;
  cfg.I = 650.0f;//650.0f;//385.0f;//300
  cfg.D = 0.98f;//0.7;//1.5f;
  cfg.leftMotorScaler = 1.0f;
  cfg.rightMotorScaler = 0.97f;
  cfg.targetAngle = 270.85f;
  cfg.turningLimit = 2;
//P:20I:0.005D:8.0
  Pspeed = 5.0;
  Ispeed = 0.001;
  Dspeed = 2.0;

}
