#ifndef MAX30102_CONFIG_H
#define MAX30102_CONFIG_H

#define MAX30102_I2C_ADDRESS 0x57 // 7-bit I2C Address, Note that MAX30102 has the same I2C address and Part ID
#define MAX30102_EXPECTEDPARTID 0x15

// Register
#define MAX30102_PARTID_REGISTER 0xFF // PARTID register address

#define MAX30102_FIFOCONFIG 0x08
#define MAX30102_FIFOWRITEPTR 0x04
#define MAX30102_FIFOOVERFLOW 0x05
#define MAX30102_FIFOREADPTR 0x06
#define MAX30102_FIFODATA 0x07

#define MAX30105_MODECONFIG 0x09
#define MAX30102_PARTICLECONFIG 0x0A
#define MAX30102_MULTILEDCONFIG1 0x11
#define MAX30102_MULTILEDCONFIG2 0x12

// MASK, For select specific bit you want to modify
#define MAX30102_SAMPLEAVG_MASK (byte) ~0b11100000
#define MAX30102_ROLLOVER_MASK 0xEF
#define MAX30102_ROLLOVER_ENABLE 0x10

#define MAX30102_MODE_MASK 0xF8
#define MAX30102_ADCRANGE_MASK 0x9F
#define MAX30102_SAMPLERATE_MASK 0xE3
#define MAX30102_PULSEWIDTH_MASK 0xFC
#define MAX30102_SLOT1_MASK 0xF8
#define MAX30102_SLOT2_MASK 0x8F
#define MAX30102_SLOT3_MASK 0xF8
#define MAX30102_SLOT4_MASK 0x8F

// Setting value
#define I2C_SPEED_STANDARD 100000
#define I2C_SPEED_FAST 400000

#define MAX30102_SAMPLEAVG_4 0x40
#define MAX30102_SAMPLEAVG_8 0x60
#define MAX30102_SAMPLEAVG_16 0x80
#define MAX30102_SAMPLEAVG_32 0xA0

#define MAX30102_ADCRANGE_2048 0x00
#define MAX30102_ADCRANGE_4096 0x20
#define MAX30102_ADCRANGE_8192 0x40

#define MAX30102_SAMPLERATE_50 0x00
#define MAX30102_SAMPLERATE_400 0x0C
#define MAX30102_SAMPLERATE_800 0x10
#define MAX30102_SAMPLERATE_1600 0x18
#define MAX30102_SAMPLERATE_3200 0x1C

#define MAX30102_PULSEWIDTH_69 0x00
#define MAX30102_PULSEWIDTH_118 0x01
#define MAX30102_PULSEWIDTH_215 0x02
#define MAX30102_PULSEWIDTH_411 0x03

#define MAX30102_LED1_PULSEAMP 0x0C
#define MAX30102_LED2_PULSEAMP 0x0D
#define MAX30102_LED3_PULSEAMP 0x0E
#define MAX30102_LED_PROX_AMP 0x10

#define SLOT_RED_LED  0x01
#define SLOT_IR_LED  0x02
#define SLOT_GREEN_LED 0x03

#endif