/**
 * Lidar manipulation and data computation
 * fetch data from LIDAR scans and compute filters to extract other robots positions
 * Computations are done on ESP32 for performances purposes
 */
#ifndef LD06_H
#define LD06_H

#define SERIAL_LIDAR Serial2
// 47 = 1(Start) + 1(Datalen) + 2(Speed) + 2(StartAngle) + 36(12 * 3 DataByte) + 2(EndAngle) + 2(TimeStamp) + 1(CRC)
#define LIDAR_SERIAL_PACKET_SIZE 47
#define LIDAR_DATA_PACKET_SIZE 12

#include <Arduino.h>
#include "Debugger.h"
#include "Robot.h"

// LIDAR

struct ConfigLidar
{
    int min_distance;
    int max_distance;
    int min_quality;
    int obs_distance;
    int obs_angle;
};
struct PointLidar
{
    int angle;
    int distance;
    uint16_t confidence;
};

struct PacketLidar
{
    byte header;
    int dataLength;
    int radarSpeed;
    int startAngle;
    PointLidar dataPoint[LIDAR_DATA_PACKET_SIZE];
    int endAngle;
    int timestamp;
    byte crcCheck;
};

class Lidar
{

public:
    Lidar(void);
    void Config(int min, int max, int quality, int distance, int angle);
    boolean ReadSerial();
    void Analyze();
    PacketLidar GetData();
    void Print();
    void AggregatePoint(Robot robot, PointLidar p);
    Point ComputeCenter();

    Point findCircle(Point p1, Point p2, Point p3);
    Point findCircle(float x1, float y1, float x2, float y2, float x3, float y3);

private:
    int16_t data_count = 0;
    int16_t obs_count = 0;

    // Initialize obstacle
    static const uint8_t obs_length = 10;
    // int const obs_max_point = 20;
    static const uint8_t obs_min_point = 3;

    uint32_t tmpChars[LIDAR_SERIAL_PACKET_SIZE];
    uint8_t cursorTmp = 0;

    Obstacle lidar_obstacle[obs_length];

    PacketLidar lidar_packet;
    ConfigLidar lidar_config;
};
#endif
