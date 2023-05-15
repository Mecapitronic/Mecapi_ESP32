/**
 * Lidar manipulation and data computation
 * fetch data from LIDAR scans and compute filters to extract other robots positions
 * Computations are done on ESP32 for performances purposes
 */
#ifndef LD06_H
#define LD06_H

// Serial 2 : U2TX = GPIO17 (Not Used for Lidar LD06); U2RX = GPIO16
#define SERIAL_LIDAR Serial2
// 47 = 1(Start) + 1(Datalen) + 2(Speed) + 2(StartAngle) + 36(12 * 3 DataByte) + 2(EndAngle) + 2(TimeStamp) + 1(CRC)
#define LIDAR_SERIAL_PACKET_SIZE 47
#define LIDAR_DATA_PACKET_SIZE 12

// angular offset between robot and lidar
// if the lidar and the robot have different origins
// positive in trigonometric way
#define LIDAR_ROBOT_ANGLE_OFFSET 0

#include <Arduino.h>
#include "Debugger.h"
#include "Robot.h"
#include "Tracker.h"

// LIDAR

struct ConfigLidar
{
    int minDistance;
    int maxDistance;
    int minQuality;
    int distanceThreshold; // represents the distance threshold to differentiate two obstacles
    int angleThreshold;    // represents the angle threshold to differentiate two obstacles
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
     * Configure lidarConfig local variable with the given values in parameters
     */
    void Config(int min, int max, int quality, int distance, int angle);

    /**
     * Read data from serial and put in a buffer if it comes form the Lidar LD06
     */
    boolean ReadSerial();

    /**
     * Put data from lidar in lidarPacket local variable.
     * Analyze and fix data according to angle step and out of bound distance
     */
    void Analyze();

    /**
     * Check between 2 lidar packet received if there is no packet loss
     */
    boolean CheckContinuity();

    /**
     * Return lidarPacket data
     */
    PacketLidar GetData();

    /**
     * convert detected position from polar coordinates to cartesian coordinates
     * according to robot position on the field
     */
    Point PolarToCartesian(PointLidar polar_point, Robot robot);

    /**
     * returns whether or not the given point is outside the table
     * the margin represents the distance between the center of the obstacle
     * and the edges of the table
     */
    bool IsOutsideTable(Point point);

    /**
     * Custom segmentation algorithm to detect cylinders in 2D plan
     * Send data to object tracker that send it to the PIC
     */
    void AggregatePoint(PointLidar lidar_point, Tracker *tracker, Robot robot);

    void ObstacleDetected(Tracker *tracker, uint8_t size);

    /**
     * the limit of passing to new obstacle
     * compare the difference with the previous point to the defined threshold
     */
    bool NewObstacleThreshold(PointLidar polar_point);

    /**
     * Debugging print: pretty print all data stored in lidarPacket
     */
    void PrintPacket(PacketLidar packet);

    /**
     * Debugging print: pretty print data stored in one PointLidar
     */
    void PrintPoint(PointLidar point);

    /**
     * Compute the center of local var lidar_obstacle
     * computes the mean of all points position to approximate circle center (without offset)
     * based on the fact that it is a cylinder of 70mm diameter
     */
    Point ComputeCenter(Obstacle lidar_obstacle);

    /**
     * Find the circle on which the given three points lie
     */
    Point FindCircle(Point p1, Point p2, Point p3);

    /**
     * Find the circle on which the given three points coordinates lie
     */
    Point FindCircle(float x1, float y1, float x2, float y2, float x3, float y3);

private:
    // minimum number of points needed to qualify as an obstacle
    static const uint8_t obstacleMinPoints = 3;

    // Maximum angle between Lidar packet admissible  = angle * 100
    static const uint8_t angleMaxDiscontinuity = 160; // TODO move into config ?

    // counter of points while detecting an obstacle from data
    uint16_t pointsCounter = 0;
    Obstacle obstacleTmp;

    // why are you using uint32 instead of chars?
    uint32_t serialBuffer[LIDAR_SERIAL_PACKET_SIZE] = {0};
    uint8_t cursorTmp = 0;

    PacketLidar lidarPacket;
    PacketLidar lidarLastPacket;
    ConfigLidar lidarConfig;
};
#endif
