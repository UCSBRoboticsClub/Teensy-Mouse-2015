#ifndef LOWPASS_H
#define LOWPASS_H


class LowPass
{
public:
    LowPass();
    LowPass(float fc);
    void setCutoffFreq(float freq, float dt);
    void setTimeConst(float tc, float dt);
    void setFilterConst(float fc);
    void push(float value);
    operator float();
    void flush(float value = 0.f);
    
private:
    float fc;
    float currentValue;
};

#endif // LOWPASS_H