#include <Servo.h>
#include "main.h"
Servo left;
Servo right;

extern cfg_t cfg;

void motor_setup(){
  left.attach(leftMotorPin);
  right.attach(rightMotorPin);
  Serial.println("Attached");
  left.writeMicroseconds((min_pulsewidth + max_pulsewidth)/2);
  right.writeMicroseconds((min_pulsewidth + max_pulsewidth)/2);
  delay(1500);
  Serial.println("Armed");
}

void drive_motor(uint16_t leftMotor, uint16_t rightMotor){

  if(leftMotor<1500){
    left.writeMicroseconds(leftMotor-75);
  }else if(leftMotor>1500){
    left.writeMicroseconds(leftMotor+70);
  }else{
    left.writeMicroseconds(leftMotor);
  }
  if(rightMotor<1500){
    right.writeMicroseconds(rightMotor-85);
  }else if (rightMotor>1500){
    right.writeMicroseconds(rightMotor+70);
  }else{
    right.writeMicroseconds(rightMotor);
  }

  //left.writeMicroseconds(leftMotor);
  //right.writeMicroseconds(rightMotor);
  //delay(500);
  //Serial.println("Wrote left" + (String)leftMotor + " Wrote right " + (String)rightMotor);
}
void updatePID(float restAngle, float offset, float turning, float dt){
  float error = restAngle - pitch;
  float pTerm = cfg.P * error;
  integratedError += error*dt;
  integratedError = constrain(integratedError, -100.0f, 100.0f);
  float iTerm = cfg.I * integratedError;
  float dTerm = cfg.D * (error-lastError) / dt;
  lastError = error;
  float PIDValue = pTerm + iTerm + dTerm;

  float PIDLeft = PIDValue + turning;
  float PIDRight = PIDValue - turning;
  uint32_t n2Timer = millis();
  if(n2Timer - nTimer >= 100){
    Serial.println("Left PID: "+(String)PIDLeft);
    Serial.println("RightPID: "+(String)PIDRight);
    nTimer = n2Timer;
  }

  PIDLeft = constrain(map(PIDLeft, -300, 300, min_pulsewidth, max_pulsewidth), min_pulsewidth, max_pulsewidth);
  PIDRight = constrain(map(PIDRight, -300, 300, min_pulsewidth, max_pulsewidth), min_pulsewidth, max_pulsewidth);

  drive_motor(PIDLeft, PIDRight);
}
//old Update PID, kept here for reference
void updatePID2(float restAngle, float offset, float turning, float dt){
  if(false/*steerStop*/){
    int32_t wheelPosition = getWheelsPosition();
    int32_t positionError = wheelPosition - targetPosition;
    if(abs(positionError)<2000){
      restAngle -= (float)positionError/1000.0f;
    }else{
      targetPosition = wheelPosition;
    }
    restAngle -=(float)wheelVelocity / 120.0f;
    restAngle = constrain(restAngle, cfg.targetAngle - 10, cfg.targetAngle +10);
  }
  float error = restAngle - pitch;
  float pTerm = cfg.P * error;
  i2Term += cfg.I * 100.0f * error * dt;
  i2Term = constrain(i2Term, -100.0f, 100.0f);
  float dTerm = (cfg.D/100.0f) * (error-lastError) / dt;
  lastError = error;
  float PIDValue = pTerm + i2Term + dTerm;

  float PIDLeft = PIDValue + turning;
  float PIDRight = PIDValue - turning;

  PIDLeft = constrain(map(PIDLeft, -300, 300, min_pulsewidth, max_pulsewidth), min_pulsewidth, max_pulsewidth);
  PIDRight = constrain(map(PIDRight, -300, 300, min_pulsewidth, max_pulsewidth), min_pulsewidth, max_pulsewidth);

  drive_motor(PIDLeft, PIDRight);
}

void stopAndReset(){
  drive_motor((min_pulsewidth + max_pulsewidth)/2, (min_pulsewidth + max_pulsewidth)/2);
  lastError = 0;
  integratedError = 0;
  targetPosition = getWheelsPosition();
  lastRestAngle = cfg.targetAngle;
}
