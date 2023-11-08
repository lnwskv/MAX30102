#ifndef MAX30102_CONFIG_H
#define MAX30102_CONFIG_H

#define MAX30102_I2C_ADDRESS 0x57 // 7-bit I2C Address, Note that MAX30102 has the same I2C address and Part ID
#define MAX30102_EXPECTEDPARTID 0x15

// Config I2C
#define I2C_SPEED_STANDARD 100000
#define I2C_SPEED_FAST 400000

// Register of MAX30102
#define MAX30102_PARTID_REGISTER 0xFF // PARTID register address

// Config register
#define MAX30102_FIFOCONFIG 0x08
#define MAX30102_FIFOWRITEPTR 0x04
#define MAX30102_FIFOOVERFLOW 0x05
#define MAX30102_FIFOREADPTR 0x06
#define MAX30102_FIFODATA 0x07

#define MAX30105_MODECONFIG 0x09
#define MAX30102_PARTICLECONFIG 0x0A
#define MAX30102_MULTILEDCONFIG1 0x11
#define MAX30102_MULTILEDCONFIG2 0x12

// FIFO MASK
#define MAX30102_SAMPLEAVG_MASK (byte) ~0b11100000
#define MAX30102_ROLLOVER_MASK 0xEF
#define MAX30102_ROLLOVER_ENABLE 0x10

// Config MASK
#define MAX30102_MODE_MASK 0xF8
#define MAX30102_ADCRANGE_MASK 0x9F
#define MAX30102_SAMPLERATE_MASK 0xE3
#define MAX30102_PULSEWIDTH_MASK 0xFC
#define MAX30102_SLOT1_MASK 0xF8
#define MAX30102_SLOT2_MASK 0x8F
#define MAX30102_SLOT3_MASK 0xF8
#define MAX30102_SLOT4_MASK 0x8F

#endif