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


void stop()
{
    rightMotor = 0.f;
    leftMotor = 0.f;
    delay(250);
}

  
void forward()
{   //tweakable values
    const float threshold = 0.1f;
    const float sideOffset = (cellw - sensw)/2.f;
    const float frontOffset = cellw/2.f - fsensoff;
    const int drivetime = 2500;
    const int lastTime = millis();

    const float kpDist = 0.5f;
    const float kpTheta = 10.f;
    const float maxControl = 0.1f;

    const float rdist = rightSensor.getDistance();
    const float ldist = leftSensor.getDistance();
    const float fdist = frontSensor.getDistance();
    
    while (millis() - lastTime < drivetime)
    {
        float control = 0.f;
        
        if (rdist < threshold)
        {
            control += kpDist * -(rdist - sideOffset);
            control += kpTheta * -drdt;
        }

        if (ldist < threshold)
        {
            control += kpDist * (ldist - sideOffset);
            control += kpTheta * dldt;
        }

        if (control > maxControl)
            control = maxControl;
        
        if (control < -maxControl)
            control = -maxControl;

        rightMotor = 0.8f + control;
        leftMotor = 0.75f - control;
    }
}


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

void updateMaze(Gps position)
{
    bool l = leftSensor.getDistance() < 0.15f;
    bool r = rightSensor.getDistance() < 0.15f;
    bool f = frontSensor.getDistance() < 0.15f;
    bool b = false;
    bool temp;

    Direction dir = position.heading;
    while (dir != Direction::jpos)
    {
        ++dir;
        temp = f;
        f = l;
        l = b;
        b = r;
        r = temp;
    }

    maze.setCellWalls(position.current.i, position.current.j, {r, f, l, b});
}


Gps drive2cell(Gps position, Node target)
{
    Movement move = pos2move(position, target);

    switch (move)
    {
    case Movement::forward:
        forward();
        stop();
        position.current = target;
        break;
    case Movement::backward:
        rightMotor = 1.f;
        leftMotor = -0.95f;
        delay(750);
        ++(++position.heading);
        stop();
        drive2cell(position, target);
        break;
    case Movement::left:
        rightMotor = 1.f;
        leftMotor = -0.95f;
        delay(300);
        --position.heading;
        stop();
        drive2cell(position, target);
        break;
    case Movement::right:
        rightMotor = -1.f;
        leftMotor = 0.95f;
        delay(300);
        ++position.heading;
        stop();
        drive2cell(position, target);
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


