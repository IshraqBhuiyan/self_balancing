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

}


void updatePID(float restAngle, float offset, float turning, float dt){
  if(!(abs(pitch - restAngle)>30)){
    float error = restAngle - pitch;
    float pTerm = cfg.P * error;
    integratedError += error*dt;
    integratedError = constrain(integratedError, -10.0f, 10.0f);
    float iTerm = cfg.I * integratedError;
    float dTerm = cfg.D * ((error-lastError) / dt);
    lastError = error;
    float PIDValue = pTerm + iTerm + dTerm;

    float PIDLeft = PIDValue; //+ turning;
    float PIDRight = PIDValue;// - turning;
    uint32_t n2Timer = millis();

    if(n2Timer - nTimer >= 1000){
      Serial.println("Left PID: "+(String)PIDLeft);
      Serial.println("RightPID: "+(String)PIDRight);
      nTimer = n2Timer;
    }

    PIDLeft = constrain(map(PIDLeft, -300, 300, min_pulsewidth, max_pulsewidth), min_pulsewidth, max_pulsewidth);
    PIDRight = constrain(map(PIDRight, -300, 300, min_pulsewidth, max_pulsewidth), min_pulsewidth, max_pulsewidth);

    drive_motor(PIDLeft, PIDRight);
  }else if((pitch-restAngle)<0){
    drive_motor(max_pulsewidth,max_pulsewidth);
  }else{
    drive_motor(min_pulsewidth,min_pulsewidth);
  }
}

void stopAndReset(){
  drive_motor((min_pulsewidth + max_pulsewidth)/2, (min_pulsewidth + max_pulsewidth)/2);
  lastError = 0;
  integratedError = 0;
  targetPosition = getWheelsPosition();
  lastRestAngle = cfg.targetAngle;
}
