#ifndef PIDexternal_h
#define PIDexternal_h

#include "Arduino.h"

class PIDexternal
{
  public:
    PIDexternal(float Pv, float Iv, float Dv, float TargetValue, float StartingValue);
    void setOffset(float Offsetv);
    void setP(float Pv);
    void setI(float Iv);
    void setD(float Dv);
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
    float Isum;
    boolean includeI;
};

#endif
