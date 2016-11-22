#include "Arduino.h"
#include "PIDexternal.h"
#include <PID_v1.h>



PIDexternal::PIDexternal(float Pv, float Iv, float Dv, float TargetValue, float StartingValue)
{
 float P = Pv;
 float I = Iv;
 float D = Dv;
 float TVal = TargetValue;
 float CVal = StartingValue;
 float LastVal = StartingValue;
 float Dif = 0;
 float Offset = 0;
 float Isum = 0;
 float Output = 0;
 boolean includeI = True;
}

void PIDexternal::setOffset(float Offsetv)}
  Offset = Offsetv;
}

void PIDexternal::setP(float Pv){
  P = Pv;
}

void PIDexternal::setI(float Iv){
  I = Iv;
}

void PIDexternal::setD(float Dv){
  D = Dv;
}

void PIDexternal::setTarget(float TargetValue){
  TVal = TargetValue;
}

void PIDexternal::step(float CurrentPosition, float Time){
  LastVal = CVal;
  CVal = CurrentPosition;
  Isum += CVal - Offset
  float Sum = P * (TVal + Offset - CVal);
  if(includeI){
    Sum += I * Isum;
  }
  Sum += D * (CVal - LastVal) / Time;
  Output = Sum;
}

float PIDexternal::getCorrection(){
  return Output;
}

float PIDexternal::StepGetC(float CurrentPosition, float Time){
  step(CurrentPosition, Time);
  return getCorrection();
}



