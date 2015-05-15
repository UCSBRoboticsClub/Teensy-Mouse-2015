#include "Commands.h"
#include "Globals.h"
#include "RadioTerminal.h"
#include <IntervalTimer.h>
#undef min
#undef max
#include <functional>
#include <cstring>
#include <stdlib.h>
#include <stdio.h>
#include "RingBuffer.h"
#include <array>
#include <limits>

#define GET(name) [&]{ return float(name); }
#define SETFLOAT(name) [&](float f){ name = f; }


struct Setter
{
    const char* name;
    std::function<void(float)> func;
};

struct Getter
{
    const char* name;
    std::function<float(void)> func;
};


std::function<void(float)> setNull = [](float f){};
std::function<float(void)> getNull = []{ return std::numeric_limits<float>::quiet_NaN(); };


const Setter setList[] =
{
    {"s.x", SETFLOAT(state.x)},
    {"s.y", SETFLOAT(state.y)},
    {"s.th", SETFLOAT(state.theta)},
    {"t.x", SETFLOAT(target.x)},
    {"t.y", SETFLOAT(target.y)},
    {"t.th", SETFLOAT(target.theta)},
    {"cc.i", SETFLOAT(currentCell.i)},
    {"cc.j", SETFLOAT(currentCell.j)},
    {"tc.i", SETFLOAT(targetCell.i)},
    {"tc.j", SETFLOAT(targetCell.j)},
    {"mspeed", SETFLOAT(maxSpeed)},
    {"cside", SETFLOAT(cside)},
    {"cfront", SETFLOAT(cfront)},
    {"ctheta", SETFLOAT(ctheta)},
    {"tdist", SETFLOAT(targetDist)},
    {"th.kp", SETFLOAT(thetakp)},
    {"th.kd", SETFLOAT(thetakd)},
    {"th.lim", SETFLOAT(thctrlmax)},
    {"rw.vkp", SETFLOAT(rightWheel.velocityLoop.kp)},
    {"rw.vki", SETFLOAT(rightWheel.velocityLoop.ki)},
    {"rw.vkd", SETFLOAT(rightWheel.velocityLoop.kd)},
    {"lw.vkp", SETFLOAT(leftWheel.velocityLoop.kp)},
    {"lw.vki", SETFLOAT(leftWheel.velocityLoop.ki)},
    {"lw.vkd", SETFLOAT(leftWheel.velocityLoop.kd)},
    {"rw.vs", [&](float f){ rightWheel.setVelocity(f); }},
    {"lw.vs", [&](float f){ leftWheel.setVelocity(f); }},
    {"rw.pkp", SETFLOAT(rightWheel.positionLoop.kp)},
    {"rw.pki", SETFLOAT(rightWheel.positionLoop.ki)},
    {"rw.pkd", SETFLOAT(rightWheel.positionLoop.kd)},
    {"lw.pkp", SETFLOAT(leftWheel.positionLoop.kp)},
    {"lw.pki", SETFLOAT(leftWheel.positionLoop.ki)},
    {"lw.pkd", SETFLOAT(leftWheel.positionLoop.kd)},
    {"rw.ps", [&](float f){ rightWheel.setPosition(f); }},
    {"lw.ps", [&](float f){ leftWheel.setPosition(f); }},
};


const Getter getList[] =
{
    {"s.x", GET(state.x)},
    {"s.y", GET(state.y)},
    {"s.th", GET(state.theta)},
    {"t.x", GET(target.x)},
    {"t.y", GET(target.y)},
    {"t.th", GET(target.theta)},
    {"cc.i", GET(currentCell.i)},
    {"cc.j", GET(currentCell.j)},
    {"tc.i", GET(targetCell.i)},
    {"tc.j", GET(targetCell.j)},
    {"drdt", GET(drdt)},
    {"dfdt", GET(dfdt)},
    {"dldt", GET(dldt)},
    {"dthdt", GET(dthdt)},
    {"dsdt", GET(dsdt)},
    {"thgoal", GET(thgoal)},
    {"therr", GET(therr)},
    {"thctrl", GET(thctrl)},
    {"speed", GET(speed)},
    {"mspeed", GET(maxSpeed)},
    {"cside", GET(cside)},
    {"cfront", GET(cfront)},
    {"ctheta", GET(ctheta)},
    {"tdist", GET(targetDist)},
    {"th.kp", GET(thetakp)},
    {"th.kd", GET(thetakd)},
    {"th.lim", GET(thctrlmax)},
    {"rw.vkp", GET(rightWheel.velocityLoop.kp)},
    {"rw.vki", GET(rightWheel.velocityLoop.ki)},
    {"rw.vkd", GET(rightWheel.velocityLoop.kd)},
    {"lw.vkp", GET(leftWheel.velocityLoop.kp)},
    {"lw.vki", GET(leftWheel.velocityLoop.ki)},
    {"lw.vkd", GET(leftWheel.velocityLoop.kd)},
    {"rw.vs", GET(rightWheel.velocitySetpoint)},
    {"lw.vs", GET(leftWheel.velocitySetpoint)},
    {"rw.v", GET(rightWheel.getVelocity())},
    {"lw.v", GET(leftWheel.getVelocity())},
    {"rw.vc", GET(rightWheel.velocityControl)},
    {"lw.vc", GET(leftWheel.velocityControl)},
    {"rw.pkp", GET(rightWheel.positionLoop.kp)},
    {"rw.pki", GET(rightWheel.positionLoop.ki)},
    {"rw.pkd", GET(rightWheel.positionLoop.kd)},
    {"lw.pkp", GET(leftWheel.positionLoop.kp)},
    {"lw.pki", GET(leftWheel.positionLoop.ki)},
    {"lw.pkd", GET(leftWheel.positionLoop.kd)},
    {"rw.ps", GET(rightWheel.positionSetpoint)},
    {"lw.ps", GET(leftWheel.positionSetpoint)},
    {"rw.p", GET(rightWheel.getPosition())},
    {"lw.p", GET(leftWheel.getPosition())},
    {"rw.enc", GET(rightWheel.getCounts())},
    {"lw.enc", GET(leftWheel.getCounts())},
    {"rs.d", GET(rightSensor.getDistance())},
    {"ls.d", GET(leftSensor.getDistance())},
    {"fs.d", GET(frontSensor.getDistance())},
    {"sw1", GET(switch1.pressed())},
    {"sw2", GET(switch2.pressed())},
    {"bt1", GET(button1.pressed())},
    {"bt2", GET(button2.pressed())},
};


IntervalTimer commandTimer;
void ctimerRefresh();


class WatchHandler : public CmdHandler
{
public:
    WatchHandler(std::function<float(void)> wf);
    ~WatchHandler() { watching = false; }
    virtual void sendChar(char c) { RadioTerminal::terminateCmd(); }
    static std::function<float(void)> watchFun;
    static bool watching;
    static void refresh();
};

std::function<float(void)> WatchHandler::watchFun;
bool WatchHandler::watching = false;


CmdHandler* watch(const char* input)
{
    char buf[32];
    const int getListSize = (sizeof getList) / (sizeof getList[0]);

    const char* s = std::strchr(input, ' ');
    if (s != nullptr)
    {
        ++s;
        for (int i = 0; i < getListSize; ++i)
        {
            if (std::strncmp(s, getList[i].name, 32) == 0)
                return new WatchHandler(getList[i].func);
        }
    }

    RadioTerminal::write("Usage: w <var>\nValid vars:");
    for (int i = 0; i < getListSize; ++i)
    {
        snprintf(buf, 32, "\n  %s", getList[i].name);
        RadioTerminal::write(buf);
    }
    return nullptr;
}


WatchHandler::WatchHandler(std::function<float(void)> wf)
{
    watchFun = wf;
    watching = true;
}


void WatchHandler::refresh()
{
    char output[256];

    sprintf(output, "\r         \r\r\r%4.4f", watchFun());
    RadioTerminal::write(output);
}


CmdHandler* print(const char* input)
{
    char buf[64];
    const int getListSize = (sizeof getList) / (sizeof getList[0]);
    
    const char* s = std::strchr(input, ' ');
    if (s != nullptr)
    {
        ++s;
        for (int i = 0; i < getListSize; ++i)
        {
            if (std::strncmp(s, getList[i].name, 32) == 0)
            {
                snprintf(buf, 32, "%s = %4.4f",
                         getList[i].name,
                         getList[i].func());
                RadioTerminal::write(buf);
                return nullptr;
            }
        }

        if (std::strncmp(s, "maze", 4) == 0)
        {
            const char rowstr[] = "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n";
            const char colstr[] = "| | | | | | | | | | | | | | | | |\n";

            RadioTerminal::write(rowstr);

            for (int i = 0; i < 15; ++i)
            {
                std::strcpy(buf, colstr);
                for (int j = 0; j < 15; ++j)
                    buf[2+j*2] = maze.nWalls.get(i, j) ? '|' : ' ';
                RadioTerminal::write(buf);

                std::strcpy(buf, rowstr);
                for (int j = 0; j < 16; ++j)
                    buf[1+j*2] = maze.mWalls.get(i, j) ? '-' : ' ';
                RadioTerminal::write(buf);
            }

            std::strcpy(buf, colstr);
            for (int j = 0; j < 15; ++j)
                buf[2+j*2] = maze.nWalls.get(15, j) ? '|' : ' ';
            RadioTerminal::write(buf);

            RadioTerminal::write(rowstr);

            return nullptr;
        }
    }

    RadioTerminal::write("Usage: p <var>\nValid variables:");
    for (int i = 0; i < getListSize; ++i)
    {
        snprintf(buf, 32, "\n  %s", getList[i].name);
        RadioTerminal::write(buf);
    }
    return nullptr;
}


CmdHandler* set(const char* input)
{
    char buf[32];
    const int setListSize = (sizeof setList) / (sizeof setList[0]);
    
    const char* s = std::strchr(input, ' ');
    if (s != nullptr)
    {
        const char* s2 = std::strchr(++s, ' ');
        if (s2 != nullptr)
        {
            int pslen = s2 - s;
            float value = strtof(++s2, nullptr);
            
            for (int i = 0; i < setListSize; ++i)
            {
                if (std::strncmp(s, setList[i].name, pslen) == 0)
                {
                    setList[i].func(value);
                    snprintf(buf, 32, "%s = %4.4f",
                             setList[i].name,
                             value);
                    RadioTerminal::write(buf);
                    return nullptr;
                }
            }
        }
    }
    
    RadioTerminal::write("Usage: s <var> <value>\nValid variables:");
    for (int i = 0; i < setListSize; ++i)
    {
        snprintf(buf, 32, "\n  %s", setList[i].name);
        RadioTerminal::write(buf);
    }
    return nullptr;
}


class LogHandler : public CmdHandler
{
public:
    LogHandler() { logging = true; }
    virtual void sendChar(char c) { logging = false; RadioTerminal::terminateCmd(); }
    ~LogHandler() { logging = false; }
    static std::function<float(void)> logFun1;
    static std::function<float(void)> logFun2;
    static std::function<float(void)> logFun3;
    static bool logging;
    static int numLogging;
    static RingBuffer<std::array<float, 3>, 1024> dataLog;
};

std::function<float(void)> LogHandler::logFun1 = getNull;
std::function<float(void)> LogHandler::logFun2 = getNull;
std::function<float(void)> LogHandler::logFun3 = getNull;
bool LogHandler::logging;
int LogHandler::numLogging;
RingBuffer<std::array<float, 3>, 1024> LogHandler::dataLog;


CmdHandler* log(const char* input)
{
    char buf[32];
    const int getListSize = (sizeof getList) / (sizeof getList[0]);

    const char* s1 = std::strchr(input, ' ');
    const char* s2 = (s1 != nullptr) ? std::strchr(s1+1, ' ') : nullptr;
    const char* s3 = (s2 != nullptr) ? std::strchr(s2+1, ' ') : nullptr;

    LogHandler::logFun1 = getNull;
    LogHandler::logFun2 = getNull;
    LogHandler::logFun3 = getNull;
    LogHandler::numLogging = 0;

    if (s1 != nullptr)
    {
        int len = s2 != nullptr ? s2 - s1 - 1: 32;
        for (int i = 0; i < getListSize; ++i)
        {
            if (std::strncmp(s1+1, getList[i].name, len) == 0)
            {
                LogHandler::logFun1 = getList[i].func;
                LogHandler::numLogging = 1;
                break;
            }
        }
    }

    if (s2 != nullptr)
    {
        int len = s3 != nullptr ? s3 - s2 - 1: 32;
        for (int i = 0; i < getListSize; ++i)
        {
            if (std::strncmp(s2+1, getList[i].name, len) == 0)
            {
                LogHandler::logFun2 = getList[i].func;
                LogHandler::numLogging = 2;
                break;
            }
        }
    }

    if (s3 != nullptr)
    {
        for (int i = 0; i < getListSize; ++i)
        {
            if (std::strncmp(s3+1, getList[i].name, 32) == 0)
            {
                LogHandler::logFun3 = getList[i].func;
                LogHandler::numLogging = 3;
                break;
            }
        }
    }

    if (LogHandler::numLogging > 0)
    {
        snprintf(buf, 32, "Logging %d variable(s)...", LogHandler::numLogging);
        RadioTerminal::write(buf);
        LogHandler::dataLog.clear();
        return new LogHandler();
    }

    RadioTerminal::write("Usage: l <var1> <var2?> <var3?>\nValid vars:");
    for (int i = 0; i < getListSize; ++i)
    {
        snprintf(buf, 32, "\n  %s", getList[i].name);
        RadioTerminal::write(buf);
    }
    return nullptr;
}


void logData()
{
    if (!LogHandler::logging)
        return;

    if (LogHandler::dataLog.size() < LogHandler::dataLog.capacity())
        LogHandler::dataLog.push({LogHandler::logFun1(),
                                  LogHandler::logFun2(),
                                  LogHandler::logFun3()});
}


void writeLog()
{
    if (LogHandler::logging && LogHandler::dataLog.size() >= LogHandler::dataLog.capacity())
    {
        char buf[32];

        switch (LogHandler::numLogging)
        {
        case 1:
            for (int i = LogHandler::dataLog.size()-1; i >= 0; --i)
            {
                snprintf(buf, 32, "\n%.6f", LogHandler::dataLog[i][0]);
                RadioTerminal::write(buf);
            }
            break;
        case 2:
            for (int i = LogHandler::dataLog.size()-1; i >= 0; --i)
            {
                snprintf(buf, 32, "\n%.6f,%.6f", LogHandler::dataLog[i][0],
                                                 LogHandler::dataLog[i][1]);
                RadioTerminal::write(buf);
            }
            break;
        case 3:
            for (int i = LogHandler::dataLog.size()-1; i >= 0; --i)
            {
                snprintf(buf, 32, "\n%.6f,%.6f,%.6f", LogHandler::dataLog[i][0],
                                                      LogHandler::dataLog[i][1],
                                                      LogHandler::dataLog[i][2]);
                RadioTerminal::write(buf);
            }
            break;
        }

        LogHandler::logging = false;
        LogHandler::dataLog.clear();
        RadioTerminal::terminateCmd();
    }
}


void ctimerRefresh()
{
    if (WatchHandler::watching)
        WatchHandler::refresh();

    writeLog();
}


void setupCommands()
{
    RadioTerminal::addCommand("w", &watch);
    RadioTerminal::addCommand("p", &print);
    RadioTerminal::addCommand("s", &set);
    RadioTerminal::addCommand("l", &log);

    commandTimer.begin(ctimerRefresh, 200000);
}


// Workaround for teensyduino/libstdc++ bug
void std::__throw_bad_function_call() { while(true) {} };
