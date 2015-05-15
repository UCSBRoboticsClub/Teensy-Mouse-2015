#include "LowPass.h"
#include <cmath>


LowPass::LowPass() : fc(0.f), currentValue(0.f)
{
}


LowPass::LowPass(float fc) : fc(fc), currentValue(0.f)
{
}


void LowPass::setCutoffFreq(float dt, float freq)
{
    fc = std::exp(-6.283185f * dt * freq);
}


void LowPass::setTimeConst(float dt, float tc)
{
    fc = std::exp(-dt / tc);
}


void LowPass::setFilterConst(float fc)
{
    this->fc = fc;
}


void LowPass::push(float value)
{
    currentValue = fc * currentValue + (1.f - fc) * value;
}


LowPass::operator float()
{
    return currentValue;
}


void LowPass::flush(float value)
{
    currentValue = value;
}
