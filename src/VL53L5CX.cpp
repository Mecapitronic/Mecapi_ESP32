#include "VL53L5CX.h"
#ifdef VL53

void VL53L5CX::Initialisation()
{
    println("Init VL53L5CX");
    Config();

    Serial.end();

    Serial.setRxBufferSize(1024);
    Serial.setTxBufferSize(1024);
    Serial.begin(115200);
    delay(1000);
    Serial.println("SparkFun VL53L5CX Imager Example");

    Wire.begin();            // This resets I2C bus to 100kHz
    Wire.setClock(1000000);  // Sensor has max I2C freq of 1MHz

    // myImager.setWireMaxPacketSize(128);  // Increase default from 32 bytes to 128 - not supported on all platforms

    Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
    // Time how long it takes to transfer firmware to sensor
    long startTime = millis();
    bool startup = myImager.begin();
    long stopTime = millis();

    if (startup == false)
    {
        Serial.println(F("Sensor not found - check your wiring. Freezing"));
        while (1);
    }

    Serial.print("Firmware transfer time: ");
    float timeTaken = (stopTime - startTime) / 1000.0;
    Serial.print(timeTaken, 3);
    Serial.println("s");

    myImager.setResolution(8 * 8);  // Enable all 64 pads

    imageResolution = myImager.getResolution();  // Query sensor for current resolution - either 4x4 or 8x8
    imageWidth = sqrt(imageResolution);          // Calculate printing width

    // Using 4x4, min frequency is 1Hz and max is 60Hz
    // Using 8x8, min frequency is 1Hz and max is 15Hz
    myImager.setRangingFrequency(15);

    myImager.startRanging();

    measurementStartTime = millis();
}

void VL53L5CX::Config() {}

void VL53L5CX::Update()
{
#define _CSV_FORMAT_
    //  Poll sensor for new data
    if (myImager.isDataReady() == true)
    {
        if (myImager.getRangingData(&measurementData))  // Read distance data into array
        {
            // The ST library returns the data transposed from zone mapping shown in datasheet
            // Pretty-print data with increasing y, decreasing x to reflect reality

#ifdef _CSV_FORMAT_
            for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth)
            {
                for (int x = imageWidth - 1; x >= 0; x--)
                {
                    Serial.print(measurementData.distance_mm[x + y]);
                    Serial.print(",");
                }
            }
#else
            for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth)
            {
                // for (int x = imageWidth - 1; x >= 0; x--)
                for (int x = 0; x <= (imageWidth - 1); x++)
                {
                    Serial.print("\t");
                    Serial.print(measurementData.distance_mm[x + y]);
                }
                Serial.println();
            }
#endif
            Serial.println();

            // Uncomment to display actual measurement rate
            // measurements++;
            // float measurementTime = (millis() - measurementStartTime) / 1000.0;
            // Serial.print("rate: ");
            // Serial.print(measurements / measurementTime, 3);
            // Serial.println("Hz");
        }
    }

    // delay(5);  // Small delay between polling
}
#endif
