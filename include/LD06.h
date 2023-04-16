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
    int distance_threshold; // represents the distance threshold to differentiate two obstacles
    int angle_threshold;    // represents the angle threshold to differentiate two obstacles
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
    /**
     * Lidar Constructor
     */
    Lidar(void);

    /**
     * Configure lidar_config local variable with the given values in parameters
     */
    void Config(int min, int max, int quality, int distance, int angle);

    /**
     * Read data from serial and put in a buffer if it comes form the Lidar LD06
     */
    boolean ReadSerial();

    /**
     * Put data from lidar in lidar_packet local variable.
     * Analyze and fix data according to angle step and out of bound distance
     */
    void Analyze();

    /**
     * Return lidar_packet data
     */
    PacketLidar GetData();

    /**
     * Debugging print: pretty print all data stored in lidar_packet
     */
    void Print();

    void AggregatePoint(Robot robot, PointLidar p);

    /**
     * Compute the center of local var lidar_obstacle
     * based on the fact that it is a cylinder of 70mm diameter
     */
    Point ComputeCenter();

    /**
     * Find the circle on which the given three points lie
     */
    Point findCircle(Point p1, Point p2, Point p3);

    /**
     * Find the circle on which the given three points coordonates lie
     */
    Point findCircle(float x1, float y1, float x2, float y2, float x3, float y3);

private:
    // counter of points while detecting an obstacle from data
    int16_t points_counter = 0;
    // counter of obstacles tracked right now
    int16_t obstacles_counter = 0;

    // Initialize obstacle
    // maximum number of obstacles we can track at the same time
    static const uint8_t obs_length = 10;
    // minimum number of pointsq needed to qualify as an obstacle
    static const uint8_t obs_min_point = 3;

    // why are you using uint32 instead of chars?
    uint32_t serial_buffer[LIDAR_SERIAL_PACKET_SIZE] = {0};
    uint8_t cursorTmp = 0;

    Obstacle lidar_obstacle[obs_length];

    PacketLidar lidar_packet;
    ConfigLidar lidar_config;
};
#endif
