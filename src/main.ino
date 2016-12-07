#include <Arduino.h>
#include <Wire.h>
#include "Kalman.h"
#include "main.h"
#include "PID_v1.h"
#include <Servo.h>

Kalman kalman;
cfg_t cfg;
int i =0;
double setpoint, input, output;
PID anglePID(&input, &output, &setpoint, (double)cfg.P, (double)cfg.I, (double)cfg.D, DIRECT);

int debugTime = 0;
float bonusSpeed;
int loops = 0;

int trigPin = 11;
int echoPin = 12;
int trigPin2 = 7;
int echoPin2 = 8;

int state =1;
uint32_t testTimer3, tt4;
void setup(){
  setValues();
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  anglePID.SetTunings((double)cfg.P, (double)cfg.I, (double)cfg.D);
  anglePID.SetSampleTime(6);
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

if (loops % 100 == 0){
if(abs(pitch-270) < 15){
  if (measurePing() < 10){
    bonusSpeed = -10;
  } else if (11 < 10){
    bonusSpeed = 10;
  } else {
    bonusSpeed = 0;
  }
} else {
    bonusSpeed = 0;
}
bonusSpeed = 0;
}
  /*debugTime += 1;
  if (debugTime < 100000){
    setpoint = cfg.targetAngle + 1.5;
  } /*else if (debugTime < 20000){
    setpoint = cfg.targetAngle - 0.4;
  }*/


  anglePID.Compute();
  if(millis()-testTimer>=1000){
    Serial.println("Kalman Value" + (String)pitch);
    Serial.println("PID Value:" + (String)output);
    testTimer = millis();
  }

if(abs(pitch-270)>40){
  drive_motor(1500,1500);
} else {
  drive_motor(1500+output, 1500+output);
}

  //updateEncoder();
}

long measurePing(){
  long duration = 0;
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(9);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  return (long)((float)duration/5.8);
}


long measurePing2(){
  long duration = 0;
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(9);
  digitalWrite(trigPin2, LOW);
  duration = pulseIn(echoPin2, HIGH);
  return (long)((float)duration/5.8);
}

void setValues(){
  cfg.accYzero = 0.0f;
  cfg.accZzero = 0.0f;
  cfg.backToSpot = 0;
  //cfg.bindSpektrum = false;
  cfg.controlAngleLimit = 45;
  cfg.P = 13.24f;//28.80f;//21.3f;//40.0f;
  cfg.I = 139.25f;//650.0f;//650.0f;//385.0f;//300
  cfg.D = 0.414f;//0.98f;//0.7;//1.5f;
  cfg.leftMotorScaler = 1.0f;
  cfg.rightMotorScaler = 0.97f;
  cfg.targetAngle = 269.442f;
  cfg.turningLimit = 2;
//P:20I:0.005D:8.0
}
