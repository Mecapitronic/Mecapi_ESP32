/**
 * Camera TOF manipulation and data computation
 * fetch data from A010 scans and compute filters
 * Computations are done on ESP32 for performances purposes
 */
#ifndef A010_H
#define A010_H

// Serial 2 : U2TX = GPIO17 ; U2RX = GPIO16
#define SERIAL_A010 Serial2

// 47 = 2(Start) + 2(Datalen) + 16(Other) + ?(Image frame) + 1(CRC) + 1(End)
// #define A010_SERIAL_PACKET_SIZE 1024

#define A010_FIRST_PACKET_BYTE 0x00
#define A010_SECOND_PACKET_BYTE 0xFF
#define A010_END_PACKET_BYTE 0xDD

#define SERIAL_A010_COPY Serial1

// we change the UART 1 RX pin from 9 to 2
// we change the UART 1 TX pin from 10 to 4

#define RX1 2
#define TX1 4

#include <Arduino.h>
#include <vector>
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
    std::vector<uint8_t> serialBuffer;
    uint8_t cursorTmp = 0;
    uint16_t packetSize = 0;

    ConfigA010 a010Config = {0, 0};
};
#endif