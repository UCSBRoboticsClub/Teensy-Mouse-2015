#include "Globals.h"
#include <Arduino.h>


Motor leftMotor(motorLF, motorLR);
Motor rightMotor(motorRF, motorRR);
VL6180X rightSensor(1);
VL6180X frontSensor(2);
VL6180X leftSensor(17);
LowPass drdt;
LowPass dfdt;
LowPass dldt;
Maze<16, 16> maze;

int rholdoff = 0;
int fholdoff = 0;
int lholdoff = 0;

void led(bool on)
{
    digitalWriteFast(ledPin, on);
}
