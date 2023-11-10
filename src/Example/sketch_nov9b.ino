#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100];   //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100];   //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

#include <MAX30102.h>
#include <heartRate.h>
#include <oxygen.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <esp_task_wdt.h>

#define ONE_WIRE_BUS 5
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
#define temp

MAX30102 max30102;

//watchdog
#define WATCHDOG_INTERVAL 10

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

  sensors.begin();

  esp_task_wdt_init(WATCHDOG_INTERVAL, true);
  esp_task_wdt_add(NULL); 
}

int command = 0;
int isSet = false;
void loop() {
  if (Serial.available()) {
    command = Serial.parseInt();
    Serial.read();
  }

  if (command == 0) {
    if (!isSet) {
      max30102.shutDown();
      isSet = !isSet;
    }
  }
  if (command == 1) {
    if (isSet) {
      max30102.wakeUp();
      isSet = !isSet;
    }
  }
  if (command == 3) {
    if (CheckFinger()) {
      CalculateBPM();
    }
  }
  if (command == 4) {
    OxygenSaturation();
  }
}


bool CheckFinger() {
  long irValue = max30102.getIR();
  if (irValue < 50000)
    return false;
  else
    return true;
}

void CalculateBPM() {
  long irValue = max30102.getIR();
  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);  // formula for calculate bpm
    DisplayTempature();
    DisplayBPM();
    Serial.println("-,");
  }
}
void DisplayBPM() {
  Serial.print(beatsPerMinute);
  Serial.print(",");
}
void DisplayTempature() {
  sensors.requestTemperatures();
  Serial.print(sensors.getTempCByIndex(0));
  Serial.print(",");
}

void OxygenSaturation() {
  bufferLength = 100;

  for (byte i = 0; i < bufferLength; i++) {
    while (max30102.available() == false)  //do we have new data?
      max30102.checkAndFillFIFO();         //Check the sensor for new data

    redBuffer[i] = max30102.getTailRed();
    irBuffer[i] = max30102.getTailIR();
    max30102.nextSample();  //We're finished with this sample so move to next sample
  }
  //calculate heart rate and SpO2 after first 100 samples
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  String data = "-,-," + String(spo2) + ",";

  Serial.println(data);
}