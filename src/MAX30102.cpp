#include "MAX30102.h"

MAX30102::MAX30102(){
}

bool MAX30102::begin(uint32_t I2C_SPEED){
    Wire.begin();
    if (readRegister8(MAX30102_I2C_ADDRESS, MAX30102_PARTID_REGISTER) != MAX30102_EXPECTEDPARTID) // ติดต่อการสื่อสารกับ MAX30102 ได้หรือไม่
        return false;

    Wire.setClock(I2C_SPEED);
    return true;
}

void MAX30102::setup(byte powerLevel, byte sampleAverage, byte ledMode, int sampleRate, int pulseWidth, int adcRange){
    if (sampleAverage == 4)  setFIFOAverage(MAX30102_SAMPLEAVG_4);
    if (sampleAverage == 8)  setFIFOAverage(MAX30102_SAMPLEAVG_8);
    if (sampleAverage == 16) setFIFOAverage(MAX30102_SAMPLEAVG_16);
    if (sampleAverage == 32) setFIFOAverage(MAX30102_SAMPLEAVG_32);
    enableFIFORollover(); // overwriting oldest data when FIFO is full

    if (ledMode == 3)      setLEDMode(0x07); // Watch all three LED channels
    else if (ledMode == 2) setLEDMode(0x03); // Red and IR
    else                   setLEDMode(0x02); // Red only
    activeLEDs = ledMode;

    if      (adcRange == 4096) setADCRange(MAX30102_ADCRANGE_4096); //15.63pA per LSB
    else if (adcRange == 8192) setADCRange(MAX30102_ADCRANGE_8192); //31.25pA per LSB
    else                       setADCRange(MAX30102_ADCRANGE_2048); //7.81pA per LSB

    if (sampleRate == 50)       setSampleRate(MAX30102_SAMPLERATE_50);
    else if(sampleRate == 400)  setSampleRate(MAX30102_SAMPLERATE_400);
    else if(sampleRate == 800)  setSampleRate(MAX30102_SAMPLERATE_800);
    else if(sampleRate == 1600) setSampleRate(MAX30102_SAMPLERATE_1600);
    else                        setSampleRate(MAX30102_SAMPLERATE_3200);
        
    if (pulseWidth < 118)     setPulseWidth(MAX30102_PULSEWIDTH_69);
    else if(pulseWidth < 215) setPulseWidth(MAX30102_PULSEWIDTH_118);
    else if(pulseWidth < 411) setPulseWidth(MAX30102_PULSEWIDTH_215);
    else                      setPulseWidth(MAX30102_PULSEWIDTH_411);

    // powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
    setPulseAmplitudeRed(powerLevel);
    setPulseAmplitudeIR(powerLevel);
    setPulseAmplitudeGreen(powerLevel);
    setPulseAmplitudeProximity(powerLevel);

    // Multi-LED Mode Configuration, Enable the reading of the three LEDs
    enableSlot(1, SLOT_RED_LED);
    if (ledMode > 1) enableSlot(2, SLOT_IR_LED);
    if (ledMode > 2) enableSlot(3, SLOT_GREEN_LED);

    clearFIFO();
}
void MAX30102::setLEDMode(uint8_t mode){ // set MAX30102 reference from datasheet
    bitMask(MAX30105_MODECONFIG, MAX30102_MODE_MASK, mode);
}
void MAX30102::enableSlot(uint8_t slotNumber, uint8_t device){
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
void MAX30102::setADCRange(uint8_t adcRange){
    bitMask(MAX30102_PARTICLECONFIG, MAX30102_ADCRANGE_MASK, adcRange);
}
void MAX30102::setSampleRate(uint8_t sampleRate){
    bitMask(MAX30102_PARTICLECONFIG, MAX30102_SAMPLERATE_MASK, sampleRate);
}
void MAX30102::setPulseWidth(uint8_t pulseWidth){
    bitMask(MAX30102_PARTICLECONFIG, MAX30102_PULSEWIDTH_MASK, pulseWidth);
}
void MAX30102::setPulseAmplitudeRed(uint8_t amplitude){
    writeRegister8(MAX30102_I2C_ADDRESS, MAX30102_LED1_PULSEAMP, amplitude);
}
void MAX30102::setPulseAmplitudeIR(uint8_t amplitude){
    writeRegister8(MAX30102_I2C_ADDRESS, MAX30102_LED2_PULSEAMP, amplitude);
}
void MAX30102::setPulseAmplitudeGreen(uint8_t amplitude){
    writeRegister8(MAX30102_I2C_ADDRESS, MAX30102_LED3_PULSEAMP, amplitude);
}
void MAX30102::setPulseAmplitudeProximity(uint8_t amplitude){
    writeRegister8(MAX30102_I2C_ADDRESS, MAX30102_LED_PROX_AMP, amplitude);
}

void MAX30102::setFIFOAverage(uint8_t numberOfSamples){
    bitMask(MAX30102_FIFOCONFIG, MAX30102_SAMPLEAVG_MASK, numberOfSamples); // set sample average
}
void MAX30102::enableFIFORollover(void){ // enable rollover mode that mean when the FIFO buffer is full and new data is added, it will overwrite the oldest data
    bitMask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_ENABLE);
}
void MAX30102::clearFIFO(void){
    writeRegister8(MAX30102_I2C_ADDRESS, MAX30102_FIFOWRITEPTR, 0);
    writeRegister8(MAX30102_I2C_ADDRESS, MAX30102_FIFOOVERFLOW, 0);
    writeRegister8(MAX30102_I2C_ADDRESS, MAX30102_FIFOREADPTR, 0);
}

uint16_t MAX30102::checkAndFillFIFO(void){
    byte readPointer = readRegister8(MAX30102_I2C_ADDRESS, MAX30102_FIFOREADPTR);
    byte writePointer = readRegister8(MAX30102_I2C_ADDRESS, MAX30102_FIFOWRITEPTR);

    int numberOfSamples = 0;

    // มีข้อมูลใหม่เข้ามาหรือไม่
    if (readPointer != writePointer)
    {
        numberOfSamples = writePointer - readPointer;
        if (numberOfSamples < 0)
            numberOfSamples += 32; // Wrap condition
        int bytesLeftToRead = numberOfSamples * activeLEDs * 3;

        Wire.beginTransmission(MAX30102_I2C_ADDRESS);
        Wire.write(MAX30102_FIFODATA);
        Wire.endTransmission();

        while (bytesLeftToRead > 0)
        {
            int bytesToGet = bytesLeftToRead;

            if (bytesToGet > I2C_BUFFER_LENGTH)
            {
                bytesToGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeLEDs * 3)); // Trim toGet to be a multiple of the samples we need to read
            }
            bytesLeftToRead -= bytesToGet;

            Wire.requestFrom(MAX30102_I2C_ADDRESS, bytesToGet);

            while (bytesToGet > 0)
            {
                sense.head++;               // Advance the head of the storage struct
                sense.head %= STORAGE_SIZE; // Wrap condition

                byte temp[sizeof(uint32_t)]; // Array of 4 bytes that we will convert into long
                uint32_t tempLong;

                // Burst read three bytes - RED
                temp[3] = 0;
                temp[2] = Wire.read();
                temp[1] = Wire.read();
                temp[0] = Wire.read();
                memcpy(&tempLong, temp, sizeof(tempLong));
                tempLong &= 0x3FFFF; // Zero out all but 18 bits

                sense.red[sense.head] = tempLong; // Store this reading into the sense array
                tempLong &= 0x3FFFF;              // Zero out all but 18 bits
                sense.IR[sense.head] = tempLong;

                if (activeLEDs > 1)
                {
                    // Burst read three more bytes - IR
                    temp[3] = 0;
                    temp[2] = Wire.read();
                    temp[1] = Wire.read();
                    temp[0] = Wire.read();

                    // Convert array to long
                    memcpy(&tempLong, temp, sizeof(tempLong));

                    tempLong &= 0x3FFFF; // Zero out all but 18 bits

                    sense.IR[sense.head] = tempLong;
                }
                if (activeLEDs > 2)
                {
                    // Burst read three more bytes - Green
                    temp[3] = 0;
                    temp[2] = Wire.read();
                    temp[1] = Wire.read();
                    temp[0] = Wire.read();

                    // Convert array to long
                    memcpy(&tempLong, temp, sizeof(tempLong));

                    tempLong &= 0x3FFFF; // Zero out all but 18 bits

                    sense.green[sense.head] = tempLong;
                }
                bytesToGet -= activeLEDs * 3;
            }
        }
    }
    return (numberOfSamples);
}
bool MAX30102::checkData(uint8_t maxTimeToCheck){
    uint32_t markTime = millis();

    while (1)
    {
        if (millis() - markTime > maxTimeToCheck)
            return (false);

        if (checkAndFillFIFO() == true) // We found new data!
            return (true);

        delay(1);
    }
}
uint32_t MAX30102::getRed(void){
  if(checkData(250))
    return (sense.red[sense.head]);
  else
    return(0);
}
uint32_t MAX30102::getIR(void){
    if (checkData(250))
        return (sense.IR[sense.head]);
    else
        return (0);
}
uint32_t MAX30102::getGreen(void)
{
  if(checkData(250))
    return (sense.green[sense.head]);
  else
    return(0);
}

void MAX30102::bitMask(uint8_t reg, uint8_t mask, uint8_t thing){ 
    uint8_t byte = readRegister8(MAX30102_I2C_ADDRESS, reg); //อ่านข้อมูลปัจจุบันของ register

    byte = byte & mask; // ใส่ mask เพื่อ clear specific bits ภายใน byte

    writeRegister8(MAX30102_I2C_ADDRESS, reg, byte | thing); // ตั้งค่าบิตเฉพาะส่วนนั้นให้เป็นค่าที่เราต้องการ
}
uint8_t MAX30102::readRegister8(uint8_t address, uint8_t reg){
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
void MAX30102::writeRegister8(uint8_t address, uint8_t reg, uint8_t value){
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}
