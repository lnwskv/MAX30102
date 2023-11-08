#ifndef MAX30102_h
#define MAX30102_h

#include "Arduino.h"
#include <Wire.h>
#include "MAX30102_CONFIG.h"

class MAX30102
{
public:
    MAX30102(void);
    bool begin(uint8_t MAX30102_ADDRESS = MAX30102_I2C_ADDRESS, uint8_t MAX30102_REGISTER = MAX30102_PARTID_REGISTER, uint32_t I2C_SPEED = I2C_SPEED_STANDARD);

    // Low-level I2C communication
    uint8_t readRegister8(uint8_t address, uint8_t reg);
};

#endif