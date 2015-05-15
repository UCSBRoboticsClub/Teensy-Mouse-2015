#ifndef VL6180X_H
#define VL6180X_H

#include <cstdint>


class VL6180X
{
public:
    VL6180X(int enablePin);
    bool init(uint8_t address);
    void poll();
    float getDistance();

    static void setup();

private:
    const int enablePin;
    uint8_t address;
    uint8_t lastReading;

    uint8_t getRegister(uint16_t regAddr);
    void setRegister(uint16_t regAddr, uint8_t data);
};


#endif // VL6810X_H
