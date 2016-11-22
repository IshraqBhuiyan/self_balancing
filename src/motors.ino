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
  left.writeMicroseconds(leftMotor);
  right.writeMicroseconds(rightMotor);
  delay(500);
  //Serial.println("Wrote left" + (String)leftMotor + " Wrote right " + (String)rightMotor);
}

void updatePID(float restAngle, float offset, float turning, float dt){
  if(steerStop){
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
  iTerm += cfg.I * 100.0f * error * dt;
  iTerm = constrain(iTerm, -100.0f, 100.0f);
  float dTerm = (cfg.D/100.0f) * (error-lastError) / dt;
  lastError = error;
  float PIDValue = pTerm + iTerm + dTerm;

  float PIDLeft = PIDValue + turning;
  float PIDRight = PIDValue - turning;

  PIDLeft *= cfg.leftMotorScaler;
  PIDRight *= cfg.rightMotorScaler;

  drive_motor(constrain(PIDLeft, min_pulsewidth, max_pulsewidth), constrain(PIDRight, min_pulsewidth, max_pulsewidth));
}

void stopAndReset(){
  drive_motor(1500, 1500);
  lastError = 0;
  iTerm = 0;
  targetPosition = getWheelsPosition();
  lastRestAngle = cfg.targetAngle;
}
