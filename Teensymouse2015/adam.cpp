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


void Drive(float angle, float speedd)
  {
    //positive angle turns clockwise
    rightMotor = (speedd + angle);
    leftMotor = (speedd - angle);
  }
  
void forward()
{   //tweakable values
    const float threshhold = 0.15f;
    const float offsetR = 0.08f;
    const float offsetL = 0.08f;
    const float offsetF = 0.05f;
    const float drivespeed = 0.8f;
    const float motordifference = 0.05f;
    const int drivetime = 2500;
    const float pterm = 10.f;
    //forward!
    int lastTime = millis();
  while(millis()-lastTime<drivetime) //1100
  {//wall on each side
      //stop if about to run into a wall
    if (frontSensor.getDistance()  < offsetF)
    {
      rightMotor = 0;
      leftMotor = 0;
    }  
    else if(leftSensor.getDistance() < threshhold && rightSensor.getDistance() < threshhold)
    {
        rightMotor = pterm*(leftSensor.getDistance()-rightSensor.getDistance())+drivespeed;
        leftMotor = pterm*(rightSensor.getDistance()-leftSensor.getDistance())+drivespeed-motordifference;
    }
    else if(leftSensor.getDistance() < threshhold)        
    {
        rightMotor = pterm*(leftSensor.getDistance()-offsetL)+drivespeed;
        leftMotor = pterm*(offsetL-leftSensor.getDistance())+drivespeed-motordifference;
    }
    else if(rightSensor.getDistance() < threshhold)
    {
        rightMotor = pterm*(offsetR-rightSensor.getDistance())+drivespeed;
        leftMotor = pterm*(rightSensor.getDistance()-offsetR)+drivespeed-motordifference;
    }
    else 
    {
        rightMotor = drivespeed;
        leftMotor = drivespeed-motordifference;
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
    bool l = false;
    bool r = false;
    bool f = false;
    bool b = false;
    bool temp;
    if(leftSensor.getDistance() < 0.15f)
    {
        l = true;
    }
    if(rightSensor.getDistance() < 0.15f)
    {
        r = true;
    }
    if(frontSensor.getDistance() < 0.15f)
    {
        f = true;
    }
    Direction dir = position.heading;
    while(dir != Direction::jpos)
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
        rightMotor = 0.f;
        leftMotor = 0.f;
        delay(250);
        position.current = target;
        break;

    case Movement::backward:
        rightMotor = 1.f;
        leftMotor = -0.95f;
        delay(750);
        ++(++position.heading);
        rightMotor = 0.f;
        leftMotor = 0.f;
        delay(250);
        drive2cell(position, target);
        break;
    case Movement::left:
        rightMotor = 1.f;
        leftMotor = -0.95f;
        delay(300);
        --position.heading;
        rightMotor = 0.f;
        leftMotor = 0.f;
        delay(250);
        drive2cell(position, target);
        break;
    case Movement::right:
        rightMotor = -1.f;
        leftMotor = 0.95f;
        delay(300);
        ++position.heading;
        rightMotor = 0.f;
        leftMotor = 0.f;
        delay(250);
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


