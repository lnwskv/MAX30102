#include <Wire.h>
#include "MAX30102.h"

MAX30102 max30102;

long samplesTaken = 0;
unsigned long startTime;

void setup()
{
    Serial.begin(9600);

    if (!max30102.begin(I2C_SPEED_FAST))
    {
        Serial.println("MAX30102 not found!");
        while (1)
            ;
    }
    delay(1000);
    Serial.println("MAX30102 found!");
    max30102.setup();
}

void loop()
{
    samplesTaken++;

    Serial.print("IR[");
    Serial.print(particleSensor.getIR());

    Serial.println();
}