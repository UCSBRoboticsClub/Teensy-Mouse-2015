#include <Encoder.h>
#include <IntervalTimer.h>
#include <SPI.h>
#include "Globals.h"
#include "Songs.h"
#include "RadioTerminal.h"
#include "Commands.h"
#include "BFS.h"
#include "BitArray2D.h"
#include <cmath>


#define YIELD                                   \
    do {                                        \
        if (abortFlag) {                        \
            abortCleanup();                     \
            return;                             \
        } else {                                \
            delay(1);                           \
        }                                       \
    } while (false)
bool abortFlag = false;
void abortCleanup();

BitArray2D<16, 16> goals;
NodeStack bfsPath;

IntervalTimer controlTimer;
IntervalTimer sensorTimer;

void controlLoop();
void sensorLoop();
float circleDist(float a, float b);
float stateDist(State s, State t);
float limit(float x, float lim);


void setup()
{
    asm(".global _printf_float");
    
    pinMode(buzzerPin, OUTPUT);
    pinMode(led1Pin, OUTPUT);
    pinMode(led2Pin, OUTPUT);
    pinMode(nfaultPin, INPUT_PULLUP);

    controlTimer.begin(controlLoop, controlPeriodUs);
    controlTimer.priority(144);

    drdt.setTimeConst(dtsensor, 0.3f);
    dfdt.setTimeConst(dtsensor, 0.3f);
    dldt.setTimeConst(dtsensor, 0.3f);
    dthdt.setTimeConst(dt, 0.1f);
    dsdt.setTimeConst(dt, 0.1f);

    VL6180X::setup();
    rightSensor.init(0x31);
    frontSensor.init(0x32);
    leftSensor.init(0x33);
    
    sensorTimer.begin(sensorLoop, sensorPeriodUs);
    sensorTimer.priority(160);

    maze.setCellWalls(0, 0, {true, false, true, true});

    delay(5000);

    //playSong(startup);
    maxSpeed = 0.15f;
}


void loop()
{
    goals.setAll(false);
    goals.set(7, 7, true);
    goals.set(8, 7, true);
    goals.set(8, 8, true);
    goals.set(7, 8, true);

    maxSpeed += 0.05f;
    
    while (!goals.get(currentCell.i, currentCell.j))
    {
        bfs(maze, currentCell, goals, bfsPath);
        if (bfsPath.size() > 0)
        {
            bfsPath.pop();
            targetCell = bfsPath.pop();
            while (currentCell != targetCell) YIELD;
        }
        YIELD;
    }

    float tempMaxSpeed = maxSpeed;
    if (maxSpeed > 0.4f)
        maxSpeed = 0.4f;
    
    goals.setAll(false);
    goals.set(0, 0, true);

    while (!goals.get(currentCell.i, currentCell.j))
    {
        bfs(maze, currentCell, goals, bfsPath);
        if (bfsPath.size() > 0)
        {
            bfsPath.pop();
            targetCell = bfsPath.pop();
            while (currentCell != targetCell) YIELD;
        }
        YIELD;
    }

    //manualSlow = true;
    target.theta = pi*0.5f;
    delay(1000);
    YIELD;
    //manualSlow = false;

    maxSpeed = tempMaxSpeed;
}


void controlLoop()
{
    rled(1);

    // Update state based on encoders
    const int rCountNew = rightWheel.getCounts();
    const int lCountNew = leftWheel.getCounts();
    const int rDiff = rCount - rCountNew;
    const int lDiff = lCount - lCountNew;
    rCount = rCountNew;
    lCount = lCountNew;

    const float dTheta = -(rDiff - lDiff) * count2dist / wheelBase;
    const float dDist = -(rDiff + lDiff) * 0.5f * count2dist;

    state.x += dDist * std::cos(state.theta + dTheta * 0.5f);
    state.y += dDist * std::sin(state.theta + dTheta * 0.5f);
    state.theta += dTheta;
    state.theta = std::fmod(state.theta + pi*2.f, pi*2.f);

    dthdt.push(dTheta / dt);
    dsdt.push(dDist / dt);

    // Determine which coordinates correspond to the sides of the robot
    direction = Direction::undefined;
    float dummy = 0.f;
    float* side = &dummy;
    float* front = &dummy;
    float sideDir = 0.f;
    float frontDir = 0.f;
    float thoffset = 0.f;
    const float thwidth2 = pi/8.f;
    if (circleDist(state.theta, 0.f) < thwidth2)
    {
        direction = Direction::ipos;
        side = &state.y; sideDir = -1.f;
        front = &state.x; frontDir = 1.f;
        thoffset = state.theta < pi ? 0 : 2.f*pi;
    }
    else if (circleDist(state.theta, pi*0.5f) < thwidth2)
    {
        direction = Direction::jpos;
        side = &state.x; sideDir = 1.f;
        front = &state.y; frontDir = 1.f;
        thoffset = pi*0.5f;
    }
    else if (circleDist(state.theta, pi) < thwidth2)
    {
        direction = Direction::ineg;
        side = &state.y; sideDir = 1.f;
        front = &state.x; frontDir = -1.f;
        thoffset = pi;
    }
    else if (circleDist(state.theta, pi*1.5f) < thwidth2)
    {
        direction = Direction::jneg;
        side = &state.x; sideDir = -1.f;
        front = &state.y; frontDir = -1.f;
        thoffset = pi*1.5f;
    }

    // Add distance sensor measurements to state
    const float sideOffset = (cellw - wallw - sensw)/2.f;
    const float frontOffset = (cellw - wallw)/2.f - fsensoff;
    const float rdist = rightSensor.getDistance();
    const float fdist = frontSensor.getDistance();
    const float ldist = leftSensor.getDistance();
    const bool inMidCell = std::fabs(frontDir*(*front) + 0.035f - cellw*0.5f) > 0.04f;

    lled(inMidCell);

    if (rdist < 0.15f && circleDist(state.theta, thoffset) < 0.5f &&
        std::fabs(dsdt) > 0.01f && std::fabs(dthdt) < 0.5f && 
        std::fabs(dthdt)/std::fabs(dsdt) < 10.f && inMidCell)
    {
        const float thMeas = thoffset + limit(drdt/(dsdt - dthdt*(sensw*0.5f + rdist)), 0.25f*pi);
        state.theta = (1.f - ctheta)*state.theta + ctheta*thMeas;
    }
    if (ldist < 0.15f && circleDist(state.theta, thoffset) < 0.5f &&
        std::fabs(dsdt) > 0.01f && std::fabs(dthdt) < 0.5f && 
        std::fabs(dthdt)/std::fabs(dsdt) < 10.f && inMidCell)
    {
        const float thMeas = thoffset - limit(dldt/(dsdt - dthdt*(sensw*0.5f + ldist)), 0.25f*pi);
        state.theta = (1.f - ctheta)*state.theta + ctheta*thMeas;
    }

    if (rdist < 0.1f && std::fabs(dthdt) < 0.5f && inMidCell)
        *side = (1.f - cside)*(*side) + sideDir*cside*(sideOffset - rdist*std::cos(state.theta - thoffset));
    if (ldist < 0.1f && std::fabs(dthdt) < 0.5f && inMidCell)
        *side = (1.f - cside)*(*side) - sideDir*cside*(sideOffset - ldist*std::cos(state.theta - thoffset));
    if (fdist < 0.15f && std::fabs(dthdt) < 0.5f && circleDist(state.theta, thoffset) < 0.3f)
    {
        const float fdistExp = frontOffset - frontDir*(*front);
        float fdistAdj = fdist*std::cos(state.theta - thoffset);
        if (fdistAdj > fdistExp + cellw*0.5f)
            fdistAdj -= cellw;
        *front = (1.f - cfront)*(*front) + frontDir*cfront*(frontOffset - fdistAdj);
    }

    // Change current cell if robot has moved far enough
    const float hyst = 0.03f;
    bool newCell = false;
    if (state.x > cellw*0.5f + hyst && currentCell.i < mazem - 1)
    {
        state.x -= cellw;
        target.x -= cellw;
        ++currentCell.i;
        newCell = true;
    }
    if (state.y > cellw*0.5f + hyst && currentCell.j < mazen - 1)
    {
        state.y -= cellw;
        target.y -= cellw;
        ++currentCell.j;
        newCell = true;
    }
    if (state.x < -(cellw*0.5f + hyst) && currentCell.i > 0)
    {
        state.x += cellw;
        target.x += cellw;
        --currentCell.i;
        newCell = true;
    }
    if (state.y < -(cellw*0.5f + hyst) && currentCell.j > 0)
    {
        state.y += cellw;
        target.y += cellw;
        --currentCell.j;
        newCell = true;
    }

    // Fill in maze walls
    if (newCell && !switch1.pressed() && circleDist(state.theta, target.theta) < 0.5f)
    {
        const bool rwall = rightSensor.getDistance() < 0.1f;
        const bool fwall = frontSensor.getDistance() < 0.15f;
        const bool lwall = leftSensor.getDistance() < 0.1f;

        auto cw = maze.getCellWalls(currentCell.i, currentCell.j);
        switch (direction)
        {
        case Direction::ipos:
            cw = {fwall, lwall, cw[2], rwall}; break;
        case Direction::jpos:
            cw = {rwall, fwall, lwall, cw[3]}; break;
        case Direction::ineg:
            cw = {cw[0], rwall, fwall, lwall}; break;
        case Direction::jneg:
            cw = {lwall, cw[1], rwall, fwall}; break;
        case Direction::undefined: break;
        }
        maze.setCellWalls(currentCell.i, currentCell.j, cw);

        tone(buzzerPin, 880, 150);
    }

    // Choose target state
    const float xdiff = cellw*(targetCell.i - currentCell.i) - state.x;
    const float ydiff = cellw*(targetCell.j - currentCell.j) - state.y;
    
    const float radius = cellw*0.5f + 0.03f;
    if (xdiff*xdiff + ydiff*ydiff < radius*radius)
    {
        const State prevTarget = target;
        target.x = cellw*(targetCell.i - currentCell.i);
        target.y = cellw*(targetCell.j - currentCell.j);
        if (prevTarget.x != target.x && prevTarget.y != target.y)
        {
            if (std::fabs(target.x - prevTarget.x) > std::fabs(target.x - prevTarget.y))
                target.theta = (target.x > prevTarget.x) ? 0.f : pi;
            else
                target.theta = (target.y > prevTarget.y) ? 0.5f*pi : 1.5f*pi;
        }
    }
    else
    {
        if (std::fabs(xdiff) > std::fabs(ydiff))
        {
            if (xdiff > 0.f)
            {
                target.x = cellw*(targetCell.i - currentCell.i) - cellw*0.5f;
                target.y = cellw*(targetCell.j - currentCell.j);
                target.theta = 0.f;
            }
            else
            {
                target.x = cellw*(targetCell.i - currentCell.i) + cellw*0.5f;
                target.y = cellw*(targetCell.j - currentCell.j);
                target.theta = pi;
            }
        }
        else
        {
            if (ydiff > 0.f)
            {
                target.x = cellw*(targetCell.i - currentCell.i);
                target.y = cellw*(targetCell.j - currentCell.j) - cellw*0.5f;
                target.theta = pi*0.5f;
            }
            else
            {
                target.x = cellw*(targetCell.i - currentCell.i);
                target.y = cellw*(targetCell.j - currentCell.j) + cellw*0.5f;
                target.theta = pi*1.5f;
            }
        }
    }

    // Move towards target
    const float deadband = inDeadband ? 0.011f : 0.01f;
    const float thDeadband = deadband; // remove thDeadband if this works
    const float slowDownDist = deadband + 0.015f;
    targetDist = stateDist(state, target);
    inDeadband = targetDist < thDeadband;

    if (targetDist < thDeadband)
        thgoal = target.theta;
    else
        thgoal = 2.f*std::atan2(target.y - state.y, target.x - state.x) - target.theta;
        
    therr = std::fmod(thgoal - state.theta + 7.f*pi, 2.f*pi) - pi;
    thctrl = thetakp*therr - thetakd*dthdt;
    if (thctrl > thctrlmax)
        thctrl = thctrlmax;
    else if (thctrl < -thctrlmax)
        thctrl = -thctrlmax;

    if (targetDist < deadband)
    {
        speed = 0.f;
    }
    else
    {
        float slowDownFactor = 1.f;
        if (targetDist < slowDownDist)
            slowDownFactor = (targetDist - deadband) / (slowDownDist - deadband);
        if (slowDownFactor < 0.f) // shouldn't happen
            slowDownFactor = 0.f;
        
        thctrl *= slowDownFactor*0.5f + 0.5f;

        const float therrAdj = std::fabs(therr*3.f) < pi*0.5f ? therr*3.f : pi*0.5f;
        speed = maxSpeed*std::cos(therrAdj)*slowDownFactor;
        speed = speed > 0.f ? speed : 0.f;
        if ((target.x - state.x)*std::cos(target.theta) + (target.y - state.y)*std::sin(target.theta) < 0.f)
            speed = -speed;
    }

    // Send commands to wheels
    if (manualSlow)
    {
        speed *= 0.2f;
        thctrl *= 0.2f;
    }

    rightWheel.setVelocity(speed + thctrl);
    leftWheel.setVelocity(speed - thctrl);
    leftWheel.update();
    rightWheel.update();

    logData();

    rled(0);
}


int rholdoff = 0;
int fholdoff = 0;
int lholdoff = 0;

void sensorLoop()
{
    const float drlast = rightSensor.getDistance();
    const float dflast = frontSensor.getDistance();
    const float dllast = leftSensor.getDistance();
    
    rightSensor.poll();
    frontSensor.poll();
    leftSensor.poll();

    // Calculate raw sensor derivatives
    const float drdtnew = (rightSensor.getDistance() - drlast) / dtsensor;
    const float dfdtnew = (frontSensor.getDistance() - dflast) / dtsensor;
    const float dldtnew = (leftSensor.getDistance() - dllast) / dtsensor;

    // If the raw derivative is high, don't add it to the filter for a while
    const float threshold = 0.5f;
    const int holdoffSteps = 8;
    if (std::fabs(drdtnew) > threshold)
        rholdoff = holdoffSteps;
    if (std::fabs(dfdtnew) > threshold)
        lholdoff = holdoffSteps;
    if (std::fabs(dldtnew) > threshold)
        lholdoff = holdoffSteps;

    if (rholdoff > 0)
        --rholdoff;
    else
        drdt.push(drdtnew);

    if (fholdoff > 0)
        --fholdoff;
    else
        dfdt.push(dfdtnew);

    if (lholdoff > 0)
        --lholdoff;
    else
        dldt.push(dldtnew);

    switch1.update();
    switch2.update();
    button1.update();
    button2.update();

    abortFlag = button1.pressed();
}


float circleDist(float a, float b)
{
    return std::fabs(std::fmod(a - b + 3.f*pi, 2.f*pi) - pi);
}


float stateDist(State s, State t)
{
    const float normFactor = 0.3f;
    
    const float vx = std::cos(t.theta);
    const float vy = std::sin(t.theta);
    const float xdiff = t.x - s.x;
    const float ydiff = t.y - s.y;
    const float tanDist = vx*xdiff + vy*ydiff;
    const float normDist = (-vy*xdiff + vx*ydiff) * normFactor;
    return std::sqrt(tanDist*tanDist + normDist*normDist);
}


float limit(float x, float lim)
{
    return x > lim ? lim : (x < -lim ? -lim : x);
}


void abortCleanup()
{
    currentCell = {0, 0};
    targetCell = currentCell;
    state = {0.f, 0.f, pi*0.5f};
    target = state;
    maxSpeed = 0.15f;

    const unsigned int startTime = millis();
    while (millis() < startTime + 3000)
    {
        if (button2.pressEdge())
        {
            maxSpeed += 0.05f;
            tone(buzzerPin, maxSpeed*1000, 100);
            delay(100);
        }
    }

    abortFlag = false;
}
