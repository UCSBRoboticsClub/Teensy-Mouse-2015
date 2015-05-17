#include <Encoder.h>
#include <IntervalTimer.h>
#include <SPI.h>
#include <i2c_t3.h>
#include "Globals.h"
#include "BFS.h"
#include "BitArray2D.h"
#include <cmath>
#include "adam.h"
#include <stdio.h>


BitArray2D<16, 16> goals;
NodeStack bfsPath;

IntervalTimer sensorTimer;

void sensorLoop();

Gps position = {{0, 0}, Direction::jpos};


void setup()
{
    asm(".global _printf_float");

    Serial.begin(115200);
    
    pinMode(ledPin, OUTPUT);

    drdt.setTimeConst(dtsensor, 0.3f);
    dfdt.setTimeConst(dtsensor, 0.3f);
    dldt.setTimeConst(dtsensor, 0.3f);

    VL6180X::setup();
    rightSensor.init(0x31);
    frontSensor.init(0x32);
    leftSensor.init(0x33);
    
    sensorTimer.begin(sensorLoop, sensorPeriodUs);
    sensorTimer.priority(160);

    maze.setCellWalls(0, 0, {true, false, true, true});

    for (int sumdumbcount = 0; sumdumbcount < 7; ++sumdumbcount)
    {
        led(1);
        delay(500);
        led(0);
        delay(500);
    }
    
    rightMotor = 0.f;
    leftMotor = 0.f;
}


void loop()
{
    goals.setAll(false);
    goals.set(7, 7, true);
    goals.set(8, 7, true);
    goals.set(8, 8, true);
    goals.set(7, 8, true);
    
    while (!goals.get(position.current.i, position.current.j))
    {
        bfs(maze, position.current, goals, bfsPath);
        if (bfsPath.size() > 0)
        {
            bfsPath.pop();
            position = drive2cell(position, bfsPath.pop());

            char buf[32];
            snprintf(buf, 32, "%d, %d, %d\r\n", position.current.i, position.current.j, int(position.heading));
            Serial.write(buf);
        }
    }
    
    goals.setAll(false);
    goals.set(0, 0, true);

    while (!goals.get(position.current.i, position.current.j))
    {
        bfs(maze, position.current, goals, bfsPath);
        if (bfsPath.size() > 0)
        {
            bfsPath.pop();
            position = drive2cell(position, bfsPath.pop());
        }
    }

    // turn 180 deg
}


void sensorLoop()
{
    const float drlast = rightSensor.getDistance();
    const float dflast = frontSensor.getDistance();
    const float dllast = leftSensor.getDistance();
    
    rightSensor.poll();
    frontSensor.poll();
    leftSensor.poll();

    // Calculate raw sensor derivatives
    float drdtnew = (rightSensor.getDistance() - drlast) / dtsensor;
    float dfdtnew = (frontSensor.getDistance() - dflast) / dtsensor;
    float dldtnew = (leftSensor.getDistance() - dllast) / dtsensor;

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
    {
        --rholdoff;
        drdtnew = 0.f;
    }

    if (fholdoff > 0)
    {
        --fholdoff;
        dfdtnew = 0.f;
    }

    if (lholdoff > 0)
    {
        --lholdoff;
        dldtnew = 0.f;
    }

    
    drdt.push(drdtnew);
    dfdt.push(dfdtnew);
    dldt.push(dldtnew);
}
