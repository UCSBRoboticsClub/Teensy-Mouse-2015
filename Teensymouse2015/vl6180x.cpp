#include "vl6180x.h"
#include <Arduino.h>
#include "i2c_t3.h"


#define IDENTIFICATION__MODEL_ID              0x0000
#define IDENTIFICATION__MODEL_REV_MAJOR       0x0001
#define IDENTIFICATION__MODEL_REV_MINOR       0x0002
#define IDENTIFICATION__MODULE_REV_MAJOR      0x0003
#define IDENTIFICATION__MODULE_REV_MINOR      0x0004
#define IDENTIFICATION__DATE_HI               0x0006
#define IDENTIFICATION__DATE_LOW              0x0007
#define IDENTIFICATION__TIME                  0x0008
#define SYSTEM__MODE_GPIO0                    0x0010
#define SYSTEM__MODE_GPIO1                    0x0011
#define SYSTEM__HISTORY_CTRL                  0x0012
#define SYSTEM__INTERRUPT_CONFIG_GPIO         0x0014
#define SYSTEM__INTERRUPT_CLEAR               0x0015
#define SYSTEM__FRESH_OUT_OF_RESET            0x0016
#define SYSTEM__GROUPED_PARAMETER_HOLD        0x0017
#define SYSRANGE__START                       0x0018
#define SYSRANGE__THRESH_HIGH                 0x0019
#define SYSRANGE__THRESH_LOW                  0x001a
#define SYSRANGE__INTERMEASUREMENT_PERIOD     0x001b
#define SYSRANGE__MAX_CONVERGENCE_TIME        0x001c
#define SYSRANGE__CROSSTALK_COMPENSATION_RATE 0x001e
#define SYSRANGE__CROSSTALK_VALID_HEIGHT      0x0021
#define SYSRANGE__EARLY_CONVERGENCE_ESTIMATE  0x0022
#define SYSRANGE__PART_TO_PART_RANGE_OFFSET   0x0024
#define SYSRANGE__RANGE_IGNORE_VALID_HEIGHT   0x0025
#define SYSRANGE__RANGE_IGNORE_THRESHOLD      0x0026
#define SYSRANGE__MAX_AMBIENT_LEVEL_MULT      0x002c
#define SYSRANGE__RANGE_CHECK_ENABLES         0x002d
#define SYSRANGE__VHV_RECALIBRATE             0x002e
#define SYSRANGE__VHV_REPEAT_RATE             0x0031
#define SYSALS__START                         0x0038
#define SYSALS__THRESH_HIGH                   0x003a
#define SYSALS__THRESH_LOW                    0x003c
#define SYSALS__INTERMEASUREMENT_PERIOD       0x003e
#define SYSALS__ANALOGUE_GAIN                 0x003f
#define SYSALS__INTEGRATION_PERIOD            0x0040
#define RESULT__RANGE_STATUS                  0x004d
#define RESULT__ALS_STATUS                    0x004e
#define RESULT__INTERRUPT_STATUS_GPIO         0x004f
#define RESULT__ALS_VAL                       0x0050
#define RESULT__HISTORY_BUFFER_0              0x0052
#define RESULT__HISTORY_BUFFER_1              0x0054
#define RESULT__HISTORY_BUFFER_2              0x0056
#define RESULT__HISTORY_BUFFER_3              0x0058
#define RESULT__HISTORY_BUFFER_4              0x005a
#define RESULT__HISTORY_BUFFER_5              0x005c
#define RESULT__HISTORY_BUFFER_6              0x005e
#define RESULT__HISTORY_BUFFER_7              0x0060
#define RESULT__RANGE_VAL                     0x0062
#define RESULT__RANGE_RAW                     0x0064
#define RESULT__RANGE_RETURN_RATE             0x0066
#define RESULT__RANGE_REFERENCE_RATE          0x0068
#define RESULT__RANGE_RETURN_SIGNAL_COUNT     0x006c
#define RESULT__RANGE_REFERENCE_SIGNAL_COUNT  0x0070
#define RESULT__RANGE_RETURN_AMB_COUNT        0x0074
#define RESULT__RANGE_REFERENCE_AMB_COUNT     0x0078
#define RESULT__RANGE_RETURN_CONV_TIME        0x007c
#define RESULT__RANGE_REFERENCE_CONV_TIME     0x0080
#define READOUT__AVERAGING_SAMPLE_PERIOD      0x010a
#define FIRMWARE__BOOTUP                      0x0119
#define FIRMWARE__RESULT_SCALER               0x0120
#define I2C_SLAVE__DEVICE_ADDRESS             0x0212
#define INTERLEAVED_MODE__ENABLE              0x02a3


VL6180X::VL6180X(int enablePin) : 
    enablePin(enablePin),
    address(0x29)
{
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, LOW);
}


void VL6180X::setup()
{
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
}


bool VL6180X::init(uint8_t newAddress)
{
    // Reset device
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, LOW);
    delayMicroseconds(100);
    
    pinMode(enablePin, INPUT);
    delay(1);

    // Secret handshake
    setRegister(0x0207, 0x01);
    setRegister(0x0208, 0x01);
    setRegister(0x0096, 0x00);
    setRegister(0x0097, 0xfd);
    setRegister(0x00e3, 0x00);
    setRegister(0x00e4, 0x04);
    setRegister(0x00e5, 0x02);
    setRegister(0x00e6, 0x01);
    setRegister(0x00e7, 0x03);
    setRegister(0x00f5, 0x02);
    setRegister(0x00d9, 0x05);
    setRegister(0x00db, 0xce);
    setRegister(0x00dc, 0x03);
    setRegister(0x00dd, 0xf8);
    setRegister(0x009f, 0x00);
    setRegister(0x00a3, 0x3c);
    setRegister(0x00b7, 0x00);
    setRegister(0x00bb, 0x3c);
    setRegister(0x00b2, 0x09);
    setRegister(0x00ca, 0x09);
    setRegister(0x0198, 0x01);
    setRegister(0x01b0, 0x17);
    setRegister(0x01ad, 0x00);
    setRegister(0x00ff, 0x05);
    setRegister(0x0100, 0x05);
    setRegister(0x0199, 0x05);
    setRegister(0x01a6, 0x1b);
    setRegister(0x01ac, 0x3e);
    setRegister(0x01a7, 0x1f);
    setRegister(0x0030, 0x00);

    // Change address
    setRegister(I2C_SLAVE__DEVICE_ADDRESS, newAddress);
    address = newAddress;

    // Settings
    setRegister(SYSTEM__MODE_GPIO1, 0x00);
    setRegister(SYSRANGE__INTERMEASUREMENT_PERIOD, 0x00);
    setRegister(SYSRANGE__MAX_CONVERGENCE_TIME, 0x4f);
    setRegister(SYSTEM__INTERRUPT_CONFIG_GPIO, 0x04);
    setRegister(READOUT__AVERAGING_SAMPLE_PERIOD, 0x30);
    setRegister(SYSRANGE__VHV_REPEAT_RATE, 0xff);
    setRegister(SYSRANGE__VHV_RECALIBRATE, 0x01);
    setRegister(SYSRANGE__START, 0x01);
    setRegister(SYSTEM__FRESH_OUT_OF_RESET, 0x00);
    
    return true;
}


void VL6180X::poll()
{
    lastReading = getRegister(RESULT__RANGE_VAL);
    setRegister(SYSTEM__INTERRUPT_CLEAR, 0x07);
    setRegister(SYSRANGE__START, 0x01);
}


float VL6180X::getDistance()
{
    return lastReading * 1e-3f;
}


uint8_t VL6180X::getRegister(uint16_t regAddr)
{
    Wire.beginTransmission(address);
    Wire.write((regAddr >> 8) & 0xFF);
    Wire.write(regAddr & 0xFF);
    Wire.endTransmission(I2C_NOSTOP);
    
    Wire.requestFrom(address, 1, I2C_STOP);
    uint8_t data = Wire.read();
    
    return data;
}


void VL6180X::setRegister(uint16_t regAddr, uint8_t data)
{
    Wire.beginTransmission(address);
    Wire.write((regAddr >> 8) & 0xFF);
    Wire.write(regAddr & 0xFF);
    Wire.write(data);
    Wire.endTransmission();
}
