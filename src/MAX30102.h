#ifndef MAX30102_h
#define MAX30102_h

#include "Arduino.h"
#include <Wire.h>
#include "MAX30102_CONFIG.h"

class MAX30102
{
public:
    MAX30102(void);
    bool begin(uint8_t I2C_ADDRESS = MAX30102_I2C_ADDRESS, uint8_t SENSOR_PARTID = MAX30102_PARTID, uint32_t I2C_SPEED = I2C_SPEED_STANDARD);
};

#endif