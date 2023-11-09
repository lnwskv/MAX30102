#include <Wire.h>
#include "MAX30102.h"

#include "heartRate.h"

MAX30102 max30102;

void setup()
{
    Serial.begin(9600);
    if (!max30102.begin())
    {
        while (1)
        {
            Serial.println("MAX30102 was not found. Please check wiring/power. ");
        };
    }
    delay(1000);
    Serial.println("Found MAX30102");
    max30102.setup();
}

