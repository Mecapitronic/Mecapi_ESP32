/**
 * Camera TOF manipulation and data computation
 * fetch data from A010 scans and compute filters
 * Computations are done on ESP32 for performances purposes
 */
#ifndef A010_H
#define A010_H

// Serial 2 : U2TX = GPIO17 ; U2RX = GPIO16
#define SERIAL_A010 Serial2

// 20+?+2 = 2(Start) + 2(Datalen) + 16(Other) + ?(Image frame) + 1(CRC) + 1(End)
// #define A010_SERIAL_PACKET_SIZE 1024

// Protocol : 2Byte header + 2Byte length + 1Byte command + 1Byte output_mode + 1Byte Sensor Temp + 1Byte Driver Temp
// 4Bytes exposure time + 1Byte error code + 1Byte reserved1 + 1Byte res rows + 1Byte res cols
// 2Byte Frame ID + 1 Byte ISP version + 1 Byte reserved3
// frame data : 100 x 100 bytes ?
// 1Byte checksum + 1Byte tail
// length count from Byte4 to the Byte before Checksum

#define A010_FIRST_PACKET_BYTE 0x00
#define A010_SECOND_PACKET_BYTE 0xFF
#define A010_END_PACKET_BYTE 0xDD

#define SERIAL_A010_COPY Serial1
// we change the UART 1 RX pin from 9 to 2
// we change the UART 1 TX pin from 10 to 4
#define RX1 2
#define TX1 4

#define BINNING_SIZE 4     // pixel binning : 1=1x1 (100x100), 2=2x2 (50x50), 4=4x4 (25x25)
#define QUANTIZATION_MM 5  // depth data resolution in mm (1 to 9)

#if BINNING_SIZE == 4
#define PICTURE_SIZE 625
#elif BINNING_SIZE == 2
#define PICTURE_SIZE 2500
#elif BINNING_SIZE == 1
#define PICTURE_SIZE 10000
#else
#error "Incorrect binning value !"
#endif

#include <Arduino.h>

#include <vector>

#include "Debugger.h"
#include "frame_struct.h"

struct ConfigA010
{
    int minDistance;
    int maxDistance;
    int IDMaxDiscontinuity;
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
     * @param discontinuity (int) the max delta in frame we can lost
     */
    void Config(int min, int max, int discontinuity);

    /**
     * Read data from serial and put in a buffer if it comes from the A010
     */
    boolean ReadSerial();
    void FillStructure();
    a010_frame_t GetData();
    boolean CheckContinuity();
    a010_point_cloud_t GetPointCloudFromFrame(a010_frame_t frame);
    void logHeader();

   private:
    std::vector<uint8_t> serialBuffer;
    uint16_t cursorTmp = 0;  // 16 bits => frame limited to 65535 bytes
    uint16_t packetSize = 0;

    ConfigA010 a010Config = {0, 0, 0};

    a010_frame_t a010Packet;
    a010_frame_head_t a010LastPacketHeader;
};
#endif