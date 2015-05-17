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
    const float threshold = 0.14f;
    const float sideOffset = (cellw - sensw)/2.f;
    const float frontOffset = cellw/2.f - fsensoff;
    const int drivetime = 2500;
    const int lastTime = millis();

    const float kp = 2.f;
    const float kd = 3.f;
    const float maxControl = 0.1f;

    while (millis() - lastTime < drivetime)
    {
        const float rdist = rightSensor.getDistance();
        const float ldist = leftSensor.getDistance();

        float control = 0.f;

        if (rdist < threshold)
            control -= kp * (rdist - sideOffset) + kd * drdt;

        if (ldist < threshold)
            control += kp * (ldist - sideOffset) + kd * dldt;

        if (control > maxControl)
            control = maxControl;
        
        if (control < -maxControl)
            control = -maxControl;

        if (millis() - lastTime > 700 && millis() - lastTime < 1600)
            control = 0.f;

        rightMotor = 0.9f + control;
        leftMotor = 0.865f - control;
    }

    while (frontSensor.getDistance() < 0.1f &&
           (frontSensor.getDistance() < frontOffset - 0.01f ||
            frontSensor.getDistance() > frontOffset + 0.01f))
    {
        if (frontSensor.getDistance() < frontOffset)
        {
            rightMotor = -0.9f;
            leftMotor = -0.865f;
        }
        else
        {
            rightMotor = 0.9f;
            leftMotor = 0.865f;
        }
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
        i = -j;
        j = temp;
    }
    else if (Direction::ineg == position.heading)
    {
        const int temp = i;
        i = j;
        j = -temp;
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
    bool l = leftSensor.getDistance() < 0.13f;
    bool r = rightSensor.getDistance() < 0.13f;
    bool f = frontSensor.getDistance() < 0.13f;
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
        updateMaze(position);
        break;
    case Movement::backward:
        rightMotor = 1.f;
        leftMotor = -0.95f;
        delay(580);
        ++(++position.heading);
        stop();
        drive2cell(position, target);
        break;
    case Movement::left:
        rightMotor = 1.f;
        leftMotor = -0.95f;
        delay(300);
        ++position.heading;
        stop();
        drive2cell(position, target);
        break;
    case Movement::right:
        rightMotor = -1.f;
        leftMotor = 0.95f;
        delay(300);
        --position.heading;
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


