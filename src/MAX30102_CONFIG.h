#ifndef MAX30102_CONFIG_H
#define MAX30102_CONFIG_H

#define MAX30102_I2C_ADDRESS 0x57     // 7-bit I2C Address, Note that MAX30102 has the same I2C address and Part ID
#define MAX30102_PARTID_REGISTER 0xFF // PARTID register address of MAX30102
#define MAX30102_EXPECTEDPARTID 0x15

#define I2C_SPEED_STANDARD 100000
#define I2C_SPEED_FAST 400000

#endif