#include "Button.h"
#include <Arduino.h>


Button::Button() :
    pin(-1),
    edge(false)
{
}


void Button::init(int pin, bool activeState, bool pullup)
{
    this->pin = pin;
    this->activeState = activeState;
    edge = false;
    pinMode(pin, pullup ? INPUT_PULLUP : INPUT);
    state = digitalRead(pin);
}


void Button::setPullup(bool on)
{
    pinMode(pin, on ? INPUT_PULLUP : INPUT);
}


void Button::update()
{
    if (pin < 0)
    {
        state = false;
        edge = false;
        return;
    }
    
    bool newState = digitalRead(pin);
    edge = (newState != state);
    state = newState;
}


bool Button::pressed()
{
    return (state == activeState);
}


bool Button::pressEdge()
{
    return (edge && state == activeState);
}


bool Button::releaseEdge()
{
    return (edge && state != activeState);
}
