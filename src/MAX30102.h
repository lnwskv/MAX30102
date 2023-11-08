#pragma once

#include "Arduino.h"
#include <Wire.h>
#include "MAX30102_CONFIG.h"

// Define the size of the I2C buffer based on the platform the user has
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)

// I2C_BUFFER_LENGTH is defined in Wire.H
#define I2C_BUFFER_LENGTH BUFFER_LENGTH

#elif defined(__SAMD21G18A__)

// SAMD21 uses RingBuffer.h
#define I2C_BUFFER_LENGTH SERIAL_BUFFER_SIZE

#else

// The catch-all default is 32
#define I2C_BUFFER_LENGTH 32

#endif

class MAX30102
{
public:
    MAX30102(void);
    bool begin(uint32_t I2C_SPEED = I2C_SPEED_STANDARD);
    void setup(byte powerLevel = 0x1F, byte sampleAverage = 4, byte ledMode = 3, int sampleRate = 400, int pulseWidth = 411, int adcRange = 4096);

    // Data
    bool checkData(uint8_t maxTimeToCheck);
    uint32_t getIR(void); // Returns immediate IR value

    // Low-level I2C communication
    uint8_t readRegister8(uint8_t address, uint8_t reg);
    void writeRegister8(uint8_t address, uint8_t reg, uint8_t value);
    void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);

    // FIFO
    void setFIFOAverage(uint8_t samples);
    void enableFIFORollover();
    void clearFIFO(void);

    // FIFO reading
    uint16_t CheckAndFillFIFO(void);

    // Configurations
    byte activeLEDs;
    void setLEDMode(uint8_t mode);
    void setADCRange(uint8_t adcRange);
    void setSampleRate(uint8_t sampleRate);
    void setPulseWidth(uint8_t pulseWidth);

    void enableSlot(uint8_t slotNumber, uint8_t device);
    void setPulseAmplitudeRed(uint8_t value);
    void setPulseAmplitudeIR(uint8_t value);
    void setPulseAmplitudeGreen(uint8_t value);
    void setPulseAmplitudeProximity(uint8_t value);

private:
#define STORAGE_SIZE 4
    typedef struct Record
    {
        uint32_t red[STORAGE_SIZE];
        uint32_t IR[STORAGE_SIZE];
        uint32_t green[STORAGE_SIZE];
        byte head;
        byte tail;
    } sense_struct; // This is our circular buffer of readings from the sensor

    sense_struct sense;
};