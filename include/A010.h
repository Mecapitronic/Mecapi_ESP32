/**
 * Camera TOF manipulation and data computation
 * fetch data from A010 scans and compute filters
 * Computations are done on ESP32 for performances purposes
 */
#ifndef A010_H
#define A010_H

// Serial 2 : U2TX = GPIO17 ; U2RX = GPIO16
#define SERIAL_A010 Serial2

#define A010_SERIAL_PACKET_SIZE 64
#define A010_FIRST_PACKET_BYTE 0x54

#define SERIAL_A010_COPY Serial1

// we change the UART 1 RX pin from 9 to 2
// we change the UART 1 TX pin from 10 to 4

#define RX1 2
#define TX1 4

#include <Arduino.h>
#include "Debugger.h"

struct ConfigA010
{
    int minDistance;
    int maxDistance;
};

class A010
{
   public:
    /**
     * A010 Constructor
     */
    A010(void);

    /**
     * @brief Configure A010Config local variable with the given values in parameters
     *
     * @param min (int) do not detect points closer than min distance (mm)
     * @param max (int) do not detect points further than max distance (mm)
     */
    void Config(int min, int max);

    /**
     * Read data from serial and put in a buffer if it comes from the A010
     */
    boolean ReadSerial();

   private:
    uint32_t serialBuffer[A010_SERIAL_PACKET_SIZE] = {0};
    uint8_t cursorTmp = 0;

    ConfigA010 a010Config = {0, 0};
};
#endif