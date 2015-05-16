#include "Globals.h"
#include <Arduino.h>


Node currentCell = {0, 0};
Direction direction = Direction::undefined;
float speed = 0.5f;
Motor leftMotor;
Motor rightMotor;
VL6180X rightSensor(0);
VL6180X frontSensor(1);
VL6180X leftSensor(2);
LowPass drdt;
LowPass dfdt;
LowPass dldt;
Maze<16, 16> maze;

void led(bool on)
{
    digitalWriteFast(ledPin, on);
}
