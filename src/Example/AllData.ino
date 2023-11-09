#include <Wire.h>
#include <MAX30102.h>
#include "heartRate.h"
#include "oxygen.h"
MAX30102 max30102;

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100];   //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100];   //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength;   //data length
int32_t spo2;           //SPO2 value
int8_t validSPO2;       //indicator to show if the SPO2 calculation is valid
int32_t heartRate;      //heart rate value
int8_t validHeartRate;  //indicator to show if the heart rate calculation is valid

long lastBeat = 0;
float beatsPerMinute = 0;

void setup() {
  Serial.begin(115200);
  if (!max30102.begin(I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power. ");
    while (1) {
    };
  }
  Serial.println("Found MAX30102");
  max30102.setup();
  max30102.setPulseAmplitudeRed(0x0A);  //Turn Red LED to low to indicate sensor is running
  max30102.setPulseAmplitudeGreen(0);   //Turn off Green LED
}

void loop() {
  long irValue = max30102.getIR();

  if (checkForBeat(irValue) == true) {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);  // formula for calculate bpm
  }

  if(irValue < 50000){
    Serial.println("no finger");
  }else{
    OxygenSaturation();
  }

  //Serial.print("Your bpm is: ");
  //Serial.println(beatsPerMinute);
}

void OxygenSaturation() {
  bufferLength = 100;

  for (byte i = 0; i < bufferLength; i++) {
    while (max30102.available() == false)  //do we have new data?
      max30102.checkAndFillFIFO();         //Check the sensor for new data

    redBuffer[i] = max30102.getRed();
    irBuffer[i] = max30102.getIR();
    max30102.nextSample();  //We're finished with this sample so move to next sample
  }
  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);


  Serial.print(F(", HR="));
  Serial.print(heartRate, DEC);

  Serial.print(F(", HRvalid="));
  Serial.print(validHeartRate, DEC);

  Serial.print(F(", SPO2="));
  Serial.print(spo2, DEC);

  Serial.print(F(", SPO2Valid="));
  Serial.println(validSPO2, DEC);
}
