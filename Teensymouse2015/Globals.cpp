#include "Globals.h"
#include <Arduino.h>


State state = {0.f, 0.f, pi*0.5f};
State target = {0.f, 0.f, pi*0.5f};
Node currentCell = {0, 0};
Node targetCell = {0, 0};
Node prevTargetCell = {0, 0};
int lCount = 0;
int rCount = 0;
LowPass drdt;
LowPass dfdt;
LowPass dldt;
LowPass dthdt;
LowPass dsdt;
float thgoal = 0.f;
float therr = 0.f;
float thctrl = 0.f;
float speed = 0.f;
float maxSpeed = 0.2f;
float ctheta = 0.03f;
float cside = 0.1f;
float cfront = 0.1f;
float targetDist = 0.f;
Direction direction = Direction::undefined;
bool inDeadband = false;
Maze<16, 16> maze;
bool manualSlow = false;
float thetakp = 0.4f;
float thetakd = 0.01f;
float thctrlmax = 0.2f;

Button switch1;
Button switch2;
Button button1;
Button button2;

Wheel leftWheel(motorLR, motorLF, encoderL1, encoderL2, dt, wheelCirc, ppr);
Wheel rightWheel(motorRR, motorRF, encoderR1, encoderR2, dt, wheelCirc, ppr);

VL6180X rightSensor(0);
VL6180X frontSensor(1);
VL6180X leftSensor(2);


void rled(bool on)
{
    digitalWriteFast(led1Pin, on); 
}


void lled(bool on)
{
    digitalWriteFast(led2Pin, on);
}
