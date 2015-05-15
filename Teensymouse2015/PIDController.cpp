#include "PIDController.h"


PIDController::PIDController(float dt) : 
    kp(0.f), 
    ki(0.f), 
    kd(0.f), 
    dt(dt), 
    ierror(0.f), 
    outputMin(-1.f), 
    outputMax(1.f), 
    lastError(0.f)
{
}


void PIDController::setTuning(float kp, float ki, float kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}


void PIDController::setDerivLowpassFreq(float freq)
{
    derror.setCutoffFreq(freq, dt);
}


void PIDController::setOutputLimits(float min, float max)
{
    outputMin = min;
    outputMax = max;
}


void PIDController::zeroIntegral()
{
    ierror = 0;
}


float PIDController::update(float error, float feedForward)
{
    float ierrorTemp = ierror + error * dt;
    
    derror.push((error - lastError) / dt);
    lastError = error;
    
    float control = feedForward + kp * error + ki * ierrorTemp + kd * derror;
    
    if (ki > 0.f)
    {
        // Don't integrate if the output is pinned at the limits
        if ( !(control > outputMax && error > 0.f) &&
             !(control < outputMin && error < 0.f) )
        {
            ierror = ierrorTemp;
        }
        
        // Limit the integral to the amount needed to saturate the output
        if (ierror * ki > outputMax)
            ierror = outputMax / ki;
        if (ierror * ki < outputMin)
            ierror = outputMin / ki;
    }
    
    return control > outputMax ? outputMax : (control < outputMin ? outputMin : control);
}
