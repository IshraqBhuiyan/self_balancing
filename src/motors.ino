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
  uint32_t n2Timer = millis();

  if(false && n2Timer - n3Timer >= 1000){
    Serial.println("Left motor: "+(String)leftMotor);
    Serial.println("right motor: "+(String)rightMotor);
    n3Timer = n2Timer;
  }
  if(leftMotor<NEUTRAL){
    left.writeMicroseconds(leftMotor-cfg.leftMotorReverseOffset);
  }else if(leftMotor>NEUTRAL){
    left.writeMicroseconds(leftMotor+cfg.leftMotorForwardOffset);
  }else{
    left.writeMicroseconds(leftMotor);
  }
  if(rightMotor<NEUTRAL){
    right.writeMicroseconds(rightMotor-cfg.rightMotorReverseOffset);
  }else if (rightMotor>NEUTRAL){
    right.writeMicroseconds(rightMotor+cfg.rightMotorForwardOffset);
  }else{
    right.writeMicroseconds(rightMotor);
  }

}
/*
  Update PID function
  Does the actual computation for each PID cycle and then executes motor movement
  Parameters:
    offset - (TODO:)A value which indicates what sort of forward or backward motion
            is desired, 1 for forward, 0 for stationary, -1 for backwards
    turning - (TODO) Either -1, 0 or 1. Indicates how to turn. -1 for left,
              0 for in place, 1 for right.
*/
void updatePID(float offset, float turning){
  //If the angle is greater than 45 degrees, the robot is not in a position
  //to be balancing and therefore the motors will be shut off until otherwise
  float restAngle = cfg.targetAngle;
  if(!(abs(pitch - restAngle)>45)){
    uint32_t currTime = micros();
    if(currTime-PIDTimer>=cfg.SAMPLETIME){
      float dt = float((currTime - PIDTimer))/1000000.0f;
      float error = restAngle - pitch;
      float pTerm = cfg.P * error;
      integratedError += cfg.I * error * dt;
      integratedError = constrain(integratedError, -400.0f, 400.0f);
      float iTerm = integratedError;
      float dTerm = (cfg.D * (error - lastError))/dt;
      lastError = error;
      float PIDValue = pTerm + iTerm - dTerm;

      float PIDLeft = PIDValue;
      float PIDRight = PIDValue;


      PIDLeft = constrain(PIDLeft, cfg.PIDMIN, cfg.PIDMAX);
      PIDRight = constrain(PIDRight, cfg.PIDMIN, cfg.PIDMAX);

      uint32_t n2Timer = millis();

      if(n2Timer - nTimer >= 1000){
        Serial.println("Left PID: "+(String)PIDLeft);
        Serial.println("RightPID: "+(String)PIDRight);
        nTimer = n2Timer;
      }

      drive_motor(NEUTRAL + (uint16_t)PIDLeft, NEUTRAL + (uint16_t)PIDRight);
      PIDTimer = currTime;
    }
  //There are two else clauses here because for testing purposes the values
  //were set to max after a certain angle in hopes of restabilizing the robot
  }else if((pitch-restAngle)<0){ //Will stop the motor at angles greater than 45
    drive_motor(NEUTRAL,NEUTRAL);
  }else{
    drive_motor(NEUTRAL,NEUTRAL);
  }
}

void stopAndReset(){
  drive_motor((min_pulsewidth + max_pulsewidth)/2, (min_pulsewidth + max_pulsewidth)/2);
  lastError = 0;
  integratedError = 0;
  targetPosition = getWheelsPosition();
  lastRestAngle = cfg.targetAngle;
}
