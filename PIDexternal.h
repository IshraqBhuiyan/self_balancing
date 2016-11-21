#ifndef PIDexternal_h
#define PIDexternal_h

#include "Arduino.h"

class PIDexternal
{
  public:
    PIDexternal(float Pv, float Iv, float Dv, float TargetValue, float StartingValue);
    void setOffset(float Offset);
    void setP(float P);
    void setI(float I);
    void setD(float D);
    void setTarget(float TargetValue);
    void step(float CurrentPosition, float Time);
    float getCorrection();
    float StepGetC(float CurrentPosition);
  private:
    float P;
    float I;
    float D;
    float TVal;
    float CVal;
    float LastVal;
    float Dif;
    float Offset;
    boolean includeI;
};

#endif
