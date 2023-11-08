#include "MAX30102.h"

MAX30102::MAX30102()
{

}

bool MAX30102::begin(uint8_t I2C_ADDRESS, uint8_t SENSOR_PARTID, uint32_t I2C_SPEED)
{
    Wire.begin();
    if (readRegister8(I2C_ADDRESS, SENSOR_PARTID) != 0x15) // ติดต่อการสื่อสารกับ MAX30102 ได้หรือไม่
        return false;

    Wire.setClock(I2C_SPEED);
    return true;
}