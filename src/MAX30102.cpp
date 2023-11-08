#include "MAX30102.h"

MAX30102::MAX30102()
{
}

bool MAX30102::begin(uint32_t I2C_SPEED)
{
    Wire.begin();
    if (readRegister8(MAX30102_I2C_ADDRESS, MAX30102_PARTID_REGISTER) != MAX30102_EXPECTEDPARTID) // ติดต่อการสื่อสารกับ MAX30102 ได้หรือไม่
        return false;

    Wire.setClock(I2C_SPEED);
    return true;
}

uint8_t MAX30102::readRegister8(uint8_t address, uint8_t reg)
{
    Wire.beginTransmission(address); // เริ่มต้นการสื่อสารกับ I2C device
    Wire.write(reg);                 // เต้องการอ่านข้อมูลจาก registerน นั้น
    Wire.endTransmission(false);     // เปิดให้สื่อสารกับอุปกรณ์ได้เพราะยังไม่ปล่อย I2C bus

    Wire.requestFrom(address, (uint8_t)1); // ขอ 1 byte
    if (Wire.available())                  // มี 1 byte ใน buffer
    {
        uint8_t value = Wire.read();
        return value;
    }
    else
        return 0;
}