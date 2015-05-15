#include "drv8833.h"
#include <Arduino.h>
#include <cmath>


Motor::Motor(int pin1, int pin2) : pin1(pin1), pin2(pin2)
{
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    
    // Set up PWM
    analogWriteFrequency(pin1, 20000);
    analogWriteFrequency(pin2, 20000);
    analogWriteResolution(16);
    analogWrite(pin1, 0);
    analogWrite(pin2, 0);
}


void Motor::write(float value)
{
    int intval = int(65535.f * std::fabs(value));
    intval = intval > 65535 ? 65535 : (intval < 0 ? 0 : intval);
    
    if (value > 0.f)
    {
        analogWrite(pin1, intval);
        analogWrite(pin2, 0);
    }
    else
    {
        analogWrite(pin1, 0);
        analogWrite(pin2, intval);
    }
}


Motor& Motor::operator=(float value)
{
    write(value);
    return *this;
}


void Motor::brake()
{
    analogWrite(pin1, 65535);
    analogWrite(pin2, 65535);
}
