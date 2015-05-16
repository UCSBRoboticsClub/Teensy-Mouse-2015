#ifndef GLOBALS_H
#define GLOBALS_H

#include "vl6180x.h"
#include "drv8833.cpp"
#include "LowPass.h"
#include "BFS.h"
#include "Maze.h"

//Teensy pinout
const int ledPin = 13;
const int motorRR = 5; // right reverse
const int motorRF = 6; // right forward motor pin
const int motorLR = 3; // left forward
const int motorLF = 4; // left reverse

const float wheelCirc = 0.1f;
const float wheelBase = 0.082f;
const float pi = 3.14159265f;
const float cellw = 0.18f;
const float wallw = 0.012f;
const float sensw = 0.067f;
const float fsensoff = 0.054f;

const int mazem = 16;
const int mazen = 16;

const unsigned int sensorFreq = 50; // Hz
const unsigned int sensorPeriodUs = 1000000 / sensorFreq;
const float dtsensor = 1.f / sensorFreq;

enum class Direction
{
    undefined,
    ipos,
    jpos,
    ineg,
    jneg
};

extern Node currentCell;
extern Direction direction;
extern float speed;
extern Motor leftMotor;
extern Motor rightMotor;
extern VL6180X rightSensor;
extern VL6180X frontSensor;
extern VL6180X leftSensor;
extern LowPass drdt;
extern LowPass dfdt;
extern LowPass dldt;
extern Maze<16, 16> maze;

void led(bool on);


#endif // GLOBALS_H
