#include <Servo.h>
#include "main.h"
#include "PID_v1.h"
Servo left;
Servo right;

PID anglePID(&input, &output, &setpoint, (double)cfg.P, (double)cfg.I, (double)cfg.D, DIRECT);

extern cfg_t cfg;

void motor_setup(){
  left.attach(leftMotorPin);
  right.attach(rightMotorPin);
  Serial.println("Attached");
  left.writeMicroseconds((min_pulsewidth + max_pulsewidth)/2);
  right.writeMicroseconds((min_pulsewidth + max_pulsewidth)/2);
  delay(1500);
  Serial.println("Armed");
  anglePID.SetTunings((double)cfg.P, (double)cfg.I, (double)cfg.D);
  anglePID.SetSampleTime(5);
  setpoint = cfg.targetAngle;
  anglePID.SetOutputLimits(-400, 400);
  anglePID.SetMode(AUTOMATIC);
  testTimer2 = millis();
}

void drive_motor(uint16_t leftMotor, uint16_t rightMotor){
  if(leftMotor<1500){
    left.writeMicroseconds(leftMotor-90);
  }else if(leftMotor>1500){
    left.writeMicroseconds(leftMotor+80);
  }else{
    left.writeMicroseconds(leftMotor);
  }
  if(rightMotor<1500){
    right.writeMicroseconds(rightMotor-90);
  }else if (rightMotor>1500){
    right.writeMicroseconds(rightMotor+70);
  }else{
    right.writeMicroseconds(rightMotor);
  }

}

void updatePID(){
  anglePID.Compute();
  if(millis()-testTimer2 <=2000){
    setpoint = cfg.targetAngle;
  }else if(millis()-testTimer2 <=5000){
    setpoint = cfg.targetAngle-1.5; //+-1.5 //
  }else{
    testTimer2 = millis();
  }
  if(abs(pitch-cfg.targetAngle)>=30){
    drive_motor(1500,1500);
  }else{
    drive_motor(1500+output, 1500+output);
  }
}

void stopAndReset(){
  drive_motor((min_pulsewidth + max_pulsewidth)/2, (min_pulsewidth + max_pulsewidth)/2);
  lastError = 0;
  integratedError = 0;
  targetPosition = getWheelsPosition();
  lastRestAngle = cfg.targetAngle;
}
