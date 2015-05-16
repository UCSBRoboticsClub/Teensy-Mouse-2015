#include <Encoder.h>
#include <IntervalTimer.h>
#include <SPI.h>
#include <i2c_t3.h>
#include "Globals.h"
#include "BFS.h"
#include "BitArray2D.h"
#include <cmath>


BitArray2D<16, 16> goals;
NodeStack bfsPath;

IntervalTimer sensorTimer;

void sensorLoop();

Gps position = {currentCell, Direction::jpos};


void setup()
{
    asm(".global _printf_float");
    
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

    delay(5000);
}


void loop()
{
    goals.setAll(false);
    goals.set(7, 7, true);
    goals.set(8, 7, true);
    goals.set(8, 8, true);
    goals.set(7, 8, true);
    
    while (!goals.get(currentCell.i, currentCell.j))
    {
        bfs(maze, currentCell, goals, bfsPath);
        if (bfsPath.size() > 0)
        {
            bfsPath.pop();
            targetCell = bfsPath.pop();
            while (currentCell != targetCell)
            {
                position = drive2(position, targetCell);
                currentCell = position.current;
            }
        }
    }
    
    goals.setAll(false);
    goals.set(0, 0, true);

    while (!goals.get(currentCell.i, currentCell.j))
    {
        bfs(maze, currentCell, goals, bfsPath);
        if (bfsPath.size() > 0)
        {
            bfsPath.pop();
            targetCell = bfsPath.pop();
            // go to target cell
        }
    }

    // turn 180 deg
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
