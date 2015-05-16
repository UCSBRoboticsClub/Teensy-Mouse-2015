#include "adam.h"
#include "Globals.h"
#include <Arduino.h>


enum class Movement
{
    forward,
    backward,
    left,
    right
};


Movement pos2move(Gps position, Node target)
{
    int i = target.i - position.current.i;
    int j = target.j - position.current.j;
    
    if (Direction::jneg == position.heading)
    {
        i = -i;
        j = -j;
    }
    else if (Direction::ipos == position.heading)
    {
        const int temp = i;
        i = j;
        j = -temp;
    }
    else if (Direction::ineg == position.heading)
    {
        const int temp = i;
        i = -j;
        j = temp;
    }
    
    switch (j)
    {
    case 1:
        return Movement::forward;
        break;
    case -1:
        return Movement::backward;
        break;
    case 0:
        switch(i)
        {
        case 1:
            return Movement::right;
            break;
        case -1:
            return Movement::left;
            break;
        }
    }
    
    // Should not reach here
    led(1);
    return Movement::forward;
}


Gps drive2cell(Gps position, Node target)
{
    Movement move = pos2move(position, target);

    switch (move)
    {
    case Movement::forward:
        rightMotor = 0.882813f;
        leftMotor = 0.664063f;
        delay(2391);
        rightMotor = 0.f;
        leftMotor = 0.f;
        position.current = target;
        break;
    case Movement::backward:
        rightMotor = 0.882813f;
        leftMotor = -0.664063f;
        delay(1024);
        ++(++position.heading);
        rightMotor = 0.f;
        leftMotor = 0.f;
        break;
    case Movement::left:
        rightMotor = 0.882813f;
        leftMotor = -0.664063f;
        delay(512);
        ++position.heading;
        rightMotor = 0.f;
        leftMotor = 0.f;
        break;
    case Movement::right:
        rightMotor = -0.882813f;
        leftMotor = 0.664063f;
        delay(512);
        --position.heading;
        rightMotor = 0.f;
        leftMotor = 0.f;
        break;
    }

    return position;
}


const Direction Direction::ipos(0);
const Direction Direction::jpos(1);
const Direction Direction::ineg(2);
const Direction Direction::jneg(3);

Direction& Direction::operator++()
{
    dir = (dir + 1) % 4;
    return *this;
}

Direction& Direction::operator--()
{
    dir = (dir - 1 + 4) % 4;
    return *this;
}

Direction Direction::operator++(int)
{
    Direction copy(*this);
    ++(*this);
    return copy;
}

Direction Direction::operator--(int)
{
    Direction copy(*this);
    --(*this);
    return copy;
}

bool Direction::operator==(const Direction& other) const
{
    return dir == other.dir;
}

bool Direction::operator!=(const Direction& other) const
{
    return !(*this == other);
}
