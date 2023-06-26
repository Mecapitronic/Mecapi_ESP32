/**
 * Camera TOF manipulation and data computation
 * fetch data from A010 scans and compute filters
 * Computations are done on ESP32 for performances purposes
 */
#ifndef A010_H
#define A010_H

// Serial 2 : U2TX = GPIO17 ; U2RX = GPIO16
#define SERIAL_A010 Serial2

// Header frame : 2Byte header + 2Byte length + 1Byte command + 1Byte output_mode + 1Byte Sensor Temp + 1Byte Driver Temp
// 4Bytes exposure time + 1Byte error code + 1Byte reserved1 + 1Byte res rows + 1Byte res cols
// 2Byte Frame ID + 1 Byte ISP version + 1 Byte reserved3
// length count from Byte4 to the Byte before Checksum
// Total Header : 20 bytes

// Data Frame : 100 x 100 bytes ?

// Footer frame : 1Byte checksum + 1Byte tail
// Total Footer : 2 bytes

// TOTAL FRAME : 20 + 625/2500/10000 + 2

// Minimal Baud rate speed : (20 + PICTURE_SIZE + 2) * FRAME_PER_SECOND * 10
//                BINN         BINN        BINN
//                  4           2           1
// FPS   5      32 350      126 100       501 100
//
// FPS  10      64 700      252 200     1 002 200
//
// FPS  15      97 050      378 300     1 503 300
//
// FPS  20     129 400      504 400     2 004 400

#define A010_FIRST_PACKET_BYTE 0x00
#define A010_SECOND_PACKET_BYTE 0xFF
#define A010_END_PACKET_BYTE 0xDD

#define SERIAL_A010_COPY Serial1
// we change the UART 1 RX pin from 9 to 2
// we change the UART 1 TX pin from 10 to 4
#define RX1 2
#define TX1 4

/***************** Configuration : REBOOT CAMERA AFTER !! *****************/
#define BINNING_SIZE 4      // pixel binning : 1=1x1 (100x100), 2=2x2 (50x50), 4=4x4 (25x25)
#define QUANTIZATION_MM 5   // depth data resolution in mm (1 to 9)
#define FRAME_PER_SECOND 6  // Frame per second, FPS from 1 to 20 (30?)
#define BAUD_RATE_STATE 5   // 0=9.600 1=57.600 2=115.200 3=230.400  4=460.800 5=921.600 6=1.000.000 7=2.000.000 8=3.000.000
/************************************************/

#define FOV_HOR 70
#define FOV_VER 60

#if BINNING_SIZE == 4
#define PICTURE_SIZE 625
#define PICTURE_RES 25
#elif BINNING_SIZE == 2
#define PICTURE_SIZE 2500
#define PICTURE_RES 50
#elif BINNING_SIZE == 1
#define PICTURE_SIZE 10000
#define PICTURE_RES 100
#else
#error "Incorrect binning value !"
#endif

#if BAUD_RATE_STATE == 0
#define BAUD_RATE_SPEED 9600
#elif BAUD_RATE_STATE == 1
#define BAUD_RATE_SPEED 57600
#elif BAUD_RATE_STATE == 2
#define BAUD_RATE_SPEED 115200
#elif BAUD_RATE_STATE == 3
#define BAUD_RATE_SPEED 230400
#elif BAUD_RATE_STATE == 4
#define BAUD_RATE_SPEED 460800
#elif BAUD_RATE_STATE == 5
#define BAUD_RATE_SPEED 921600
#elif BAUD_RATE_STATE == 6
#define BAUD_RATE_SPEED 1000000
#elif BAUD_RATE_STATE == 7
#define BAUD_RATE_SPEED 2000000
#elif BAUD_RATE_STATE == 8
#define BAUD_RATE_SPEED 3000000
#else
#error "Incorrect Baud Rate State value !"
#endif

#define BAUD_RATE_MIN ((20 + PICTURE_SIZE + 2) * FRAME_PER_SECOND * 10)
#if BAUD_RATE_SPEED < BAUD_RATE_MIN
#error "!!! Baud rate selected is not enough or BINN - FPS are too high !!!"
#endif

#include <Arduino.h>
#include <vector>

// #include "..\DBSCAN-for-ESP32\DBSCAN.h"
#include "Debugger.h"
#include "frame_struct.h"

using std::vector;

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
    boolean CheckContinuity();

    void ComputeCartesianCoefficient(uint16_t horRes, uint16_t verRes, float horFOVdeg, float verFOVdeg, float horOFFSETdeg, float verOFFSETdeg);
    void logCartesianCoefficient();
    void logHeader();

    ClusterPoint3D cloudFrame[PICTURE_SIZE];

   private:
    uint16_t cursorTmp = 0;  // 16 bits => frame limited to 65535 bytes
    uint16_t indexTmp = 0;
    uint16_t packetSize = 0;

    ConfigA010 a010Config = {0, 0, 0};

    a010_frame_t a010Packet;
    a010_frame_head_t a010LastPacketHeader;

    float coefX[PICTURE_RES];
    float coefY[PICTURE_RES];
    float coefZ[PICTURE_RES];
};
#endif
