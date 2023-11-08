#include "MAX30102.h"

static const uint8_t MAX30102_SAMPLEAVG_4 = 0x40;

static const uint8_t MAX30102_ADCRANGE_2048 = 0x00;
static const uint8_t MAX30102_ADCRANGE_4096 = 0x20;

static const uint8_t MAX30102_SAMPLERATE_50 = 0x00;
static const uint8_t MAX30102_SAMPLERATE_400 = 0x0C;

static const uint8_t MAX30102_PULSEWIDTH_69 = 0x00;
static const uint8_t MAX30102_PULSEWIDTH_411 = 0x03;

static const uint8_t MAX30102_LED1_PULSEAMP = 0x0C;
static const uint8_t MAX30102_LED2_PULSEAMP = 0x0D;
static const uint8_t MAX30102_LED3_PULSEAMP = 0x0E;
static const uint8_t MAX30102_LED_PROX_AMP = 0x10;

static const uint8_t SLOT_RED_LED = 0x01;
static const uint8_t SLOT_IR_LED = 0x02;
static const uint8_t SLOT_GREEN_LED = 0x03;

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

void MAX30102::setup(byte powerLevel, byte sampleAverage, byte ledMode, int sampleRate, int pulseWidth, int adcRange)
{
    setFIFOAverage(MAX30102_SAMPLEAVG_4);
    enableFIFORollover();

    if (ledMode == 3)
        setLEDMode(0x07); // Watch all three LED channels
    else if (ledMode == 2)
        setLEDMode(0x03); // Red and IR
    else
        setLEDMode(0x02); // Red only
    activeLEDs = ledMode;

    setPulseAmplitudeRed(powerLevel);
    setPulseAmplitudeIR(powerLevel);
    setPulseAmplitudeGreen(powerLevel);
    setPulseAmplitudeProximity(powerLevel);

    if (adcRange == 4096)
    {
        setADCRange(MAX30102_ADCRANGE_4096);
    }
    else
    {
        setADCRange(MAX30102_ADCRANGE_2048);
    }

    if (sampleRate == 400)
    {
        setSampleRate(MAX30102_SAMPLERATE_400);
    }
    else
    {
        setSampleRate(MAX30102_SAMPLERATE_50);
    }

    if (pulseWidth == 411)
    {
        setPulseWidth(MAX30102_PULSEWIDTH_411);
    }
    else
    {
        setPulseWidth(MAX30102_PULSEWIDTH_69);
    }

    // powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
    setPulseAmplitudeRed(powerLevel);
    setPulseAmplitudeIR(powerLevel);
    setPulseAmplitudeGreen(powerLevel);
    setPulseAmplitudeProximity(powerLevel);

    // Multi-LED Mode Configuration, Enable the reading of the three LEDs
    enableSlot(1, SLOT_RED_LED);
    if (ledMode > 1)
        enableSlot(2, SLOT_IR_LED);
    if (ledMode > 2)
        enableSlot(3, SLOT_GREEN_LED);

    clearFIFO();
}
void MAX30102::setLEDMode(uint8_t mode)
{ // set MAX30102 reference from datasheet
    bitMask(MAX30105_MODECONFIG, MAX30102_MODE_MASK, mode);
}
void MAX30102::setADCRange(uint8_t adcRange)
{
    bitMask(MAX30102_PARTICLECONFIG, MAX30102_ADCRANGE_MASK, adcRange);
}
void MAX30102::setSampleRate(uint8_t sampleRate)
{
    bitMask(MAX30102_PARTICLECONFIG, MAX30102_SAMPLERATE_MASK, sampleRate);
}
void MAX30102::setPulseWidth(uint8_t pulseWidth)
{
    bitMask(MAX30102_PARTICLECONFIG, MAX30102_PULSEWIDTH_MASK, pulseWidth);
}
void MAX30102::setPulseAmplitudeRed(uint8_t amplitude)
{
    writeRegister8(MAX30102_I2C_ADDRESS, MAX30102_LED1_PULSEAMP, amplitude);
}
void MAX30102::setPulseAmplitudeIR(uint8_t amplitude)
{
    writeRegister8(MAX30102_I2C_ADDRESS, MAX30102_LED2_PULSEAMP, amplitude);
}
void MAX30102::setPulseAmplitudeGreen(uint8_t amplitude)
{
    writeRegister8(MAX30102_I2C_ADDRESS, MAX30102_LED3_PULSEAMP, amplitude);
}
void MAX30102::setPulseAmplitudeProximity(uint8_t amplitude)
{
    writeRegister8(MAX30102_I2C_ADDRESS, MAX30102_LED_PROX_AMP, amplitude);
}
void MAX30102::enableSlot(uint8_t slotNumber, uint8_t device)
{
    switch (slotNumber)
    {
    case (1):
        bitMask(MAX30102_MULTILEDCONFIG1, MAX30102_SLOT1_MASK, device);
        break;
    case (2):
        bitMask(MAX30102_MULTILEDCONFIG1, MAX30102_SLOT2_MASK, device << 4);
        break;
    case (3):
        bitMask(MAX30102_MULTILEDCONFIG2, MAX30102_SLOT3_MASK, device);
        break;
    case (4):
        bitMask(MAX30102_MULTILEDCONFIG2, MAX30102_SLOT4_MASK, device << 4);
        break;
    default:
        break;
    }
}

void MAX30102::setFIFOAverage(uint8_t numberOfSamples)
{
    bitMask(MAX30102_FIFOCONFIG, MAX30102_SAMPLEAVG_MASK, numberOfSamples); // set sample average
}
void MAX30102::enableFIFORollover(void)
{ // enable rollover mode that mean when the FIFO buffer is full and new data is added, it will overwrite the oldest data
    bitMask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_ENABLE);
}
void MAX30102::clearFIFO(void)
{
    writeRegister8(MAX30102_I2C_ADDRESS, MAX30102_FIFOWRITEPTR, 0);
    writeRegister8(MAX30102_I2C_ADDRESS, MAX30102_FIFOOVERFLOW, 0);
    writeRegister8(MAX30102_I2C_ADDRESS, MAX30102_FIFOREADPTR, 0);
}

void MAX30102::bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
    uint8_t byte = readRegister8(Wire, reg);

    byte = byte & mask;

    writeRegister8(MAX30102_I2C_ADDRESS, reg, byte | thing);
}
uint8_t MAX30102::readRegister8(uint8_t address, uint8_t reg)
{
    Wire.beginTransmission(address); // เริ่มต้นการสื่อสารกับ I2C device
    Wire.write(reg);                 // ต้องการอ่านเขียนข้อมูลลง register นั้น
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
void MAX30102::writeRegister8(uint8_t address, uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}