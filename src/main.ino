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
  setpoint = cfg.targetAngle;
  anglePID.SetOutputLimits(0, 400);
  anglePID.SetMode(REVERSE);
  //cfg.targetAngle = 180.0f;
  //Serial.println("Hi2");
  MPU_setup();
  //setup_encoder();
  motor_setup();
  stopAndReset();
  testTimer = millis();
  nTimer = millis();
  n3Timer = millis();
  //cfg.backToSpot = 0;
  //Serial.println("Hi1");

}

void loop(){
  MPU_update();
  input = (double)pitch;
  //Serial.println("Kalman Pitch: " + (String)pitch);
  uint32_t timer = micros();
  //drive_motor(2200, 2200);
  anglePID.Compute();
  if(millis()-testTimer>=1000){
    Serial.println("Kalman Value" + (String)pitch);
    Serial.println("PID Value:" + (String)output);
    testTimer = millis();
  }
  if((pitch - cfg.targetAngle) <0){
    drive_motor(1500-output, 1500-output);
  }else{
    drive_motor(1500+output, 1500+output);
  }
  //drive_motor(output, output);
  //updatePID(cfg.targetAngle, 0, 0,(float)(timer-PIDTimer)/1000000.0f);
  //updateEncoder();
  //Serial.println("Motor Speeds" + (String)(1500+i));
  /*
  uint32_t currTime = millis();
  if(currTime-testTimer >= 1000){
    Serial.println("Left Velocity: " + (String)leftVelocity);
    Serial.println("Right Velocity: " + (String)rightVelocity);
    testTimer = currTime;
  }
  //Serial.println("Left Counter " + (String)leftCounter);
  i+=10;
  */
  /*
  Serial.println("Hi");
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
  delay(1000);
  */
}

void setValues(){
  cfg.accYzero = 0.0f;
  cfg.accZzero = 0.0f;
  cfg.backToSpot = 0;
  //cfg.bindSpektrum = false;
  cfg.controlAngleLimit = 45;
  cfg.P = 1.00f;
  cfg.I = 0.00f;
  cfg.D = 00.0f;
  cfg.leftMotorScaler = 1.0f;
  cfg.rightMotorScaler = 0.97f;
  cfg.targetAngle = 275.00f;
  cfg.turningLimit = 2;
//P:20I:0.005D:8.0
}
