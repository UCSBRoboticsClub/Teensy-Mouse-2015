#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "LowPass.h"


class PIDController
{
public:
    PIDController(float dt);
    float update(float error, float feedForward = 0.f);
    void setTuning(float kp, float ki, float kd);
    void setDerivLowpassFreq(float freq);
    void setOutputLimits(float min, float max);
    void zeroIntegral();
    
    float kp;
    float ki;
    float kd;
    
private:
    const float dt;
    float ierror;
    LowPass derror;
    LowPass ierrorlp;
    float outputMin;
    float outputMax;
    float lastError;
};

#endif // PIDCONTROLLER_H
