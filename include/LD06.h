/**
 * LidarLD06 manipulation and data computation
 * fetch data from LIDAR scans and compute filters to extract other robots positions
 * Computations are done on ESP32 for performances purposes
 */
#ifndef LD06_H
#define LD06_H

#pragma region DEFINE
// Serial 2 : U2TX = GPIO17 (Not Used for LidarLD06 LD06); U2RX = GPIO16
#define SERIAL_LIDAR Serial2
// 47 = 1(Start) + 1(Datalen) + 2(Speed) + 2(StartAngle) + 36(12 * 3 DataByte) + 2(EndAngle) + 2(TimeStamp) + 1(CRC)
#define LIDAR_SERIAL_PACKET_SIZE 47
#define LIDAR_DATA_PACKET_SIZE 12

// angular offset between robot and lidar
// if the lidar and the robot have different origins
// positive in trigonometric way
#define LIDAR_ROBOT_ANGLE_OFFSET 0
#pragma endregion

#include "ESP32_Helper.h"

using namespace Printer;

struct ConfigLidar
{
    int minDistance;        // represents the minimum distance of point from robot to be accepted
    int maxDistance;        // represents the maximum distance of point from robot to be accepted
    int minQuality;         // represents the minimum quality of a point to be accepted as good enough
    int distanceThreshold;  // represents the distance threshold to differentiate two obstacles
    int angleThreshold;     // represents the angle threshold to differentiate two obstacles
};

struct PacketLidar
{
    byte header;
    int dataLength;
    int radarSpeed;
    int startAngle;
    PolarPoint dataPoint[LIDAR_DATA_PACKET_SIZE];
    int endAngle;
    int timestamp;
    byte crcCheck;
};

/**
 * Represent a cluster of points, with the average of all point
 * The maximum points needed to represent a 70mm wide cylinder is 20
 */
struct Cluster
{
    vector<PolarPoint> data;
    PolarPoint mid;
    int index;
};

class LidarLD06
{
   public:
    void Initialisation();
    void Update();

    /**
     * @brief Configure lidarConfig local variable with the given values in parameters
     *
     * @param min (int) do not detect points closer than min distance (mm)
     * @param max (int) do not detect points further than max distance (mm)
     * @param quality (int) minimum confidence required to consider the detected point (%)
     * @param distance (int) distance threshold (mm)
     * @param angle (int) angle threshold (°)
     * @param count (int) count threshold (num)
     */
    void Config(int min, int max, int quality, int distance, int angle, int count);

    /**
     * Get LidarLD06 Configuration
     */
    ConfigLidar GetConfig();

    /**
     * @brief Change duty cycle for the PWM
     *
     * @param duty_cycle (int) the duty cycle of PWM in percentage (20% to 50%)
     * @details Scan rate around 5.0  HZ when PWM duty at 21 %
     * @details Scan rate around 6.1  HZ when PWM duty at 25 %
     * @details Scan rate around 10.1 HZ when PWM duty at 39 %
     * @details Scan rate around 13.2 HZ when PWM duty at 50 %
     *
     */
    void ChangePWM(uint32_t duty_cycle);

    /**
     * @brief Return the duty cycle of the PWM
     *
     * @return uint32_t the duty cycle
     */
    uint32_t GetPWM();

   private:
    /**
     * Read data from serial and put in a buffer if it comes form the LidarLD06 LD06
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
     * convert detected position from polar coordinates to cartesian coordinates
     * according to robot position on the field
     */
    void PolarToCartesian(PolarPoint& polarPoint);

    /**
     * returns whether or not the given point is outside the table
     * the margin represents the distance between the center of the obstacle
     * and the edges of the table
     */
    bool IsOutsideTable(PolarPoint polarPoint);

    /**
     * returns whether or not the given point is outside the config in distance min and max
     * and quality
     */
    bool IsOutsideConfig(PolarPoint polarPoint);

    /**
     * Custom segmentation algorithm to detect cylinders in 2D plan
     * Send data to object tracker that send it to the PIC
     */
    void AggregatePoint(PolarPoint polarPoint);

    void ObstacleDetected(Cluster& c);

    void CheckCluster(PolarPoint polarPoint);

    /**
     * Compute the center of the points aggregated
     * computes the mean of all points position to approximate circle center (without offset)
     * based on the fact that it is a cylinder of 70mm diameter
     */
    void ComputeCenter(Cluster& c);

    /**
     * Find the circle on which the given three points lie
     */
    Point FindCircle(Point p1, Point p2, Point p3);

    /**
     * Find the circle on which the given three points coordinates lie
     */
    Point FindCircle(float x1, float y1, float x2, float y2, float x3, float y3);

   private:
    // Number of points needed to qualify as an obstacle
    // TODO remove this : we need to know if obstacle is a beacon, auto calculation
    static const uint8_t obstacleMinPoints = 4;
    static const uint8_t obstacleMaxPoints = 25;

    // Maximum angle between LidarLD06 packet admissible  = angle * 100
    static const uint8_t angleMaxDiscontinuity = 160;  // TODO move into config ?

    // counter of points while detecting an obstacle from data
    uint16_t pointsCounter = 0;
    vector<Cluster> cluster;

    // why are you using uint32 instead of chars?
    uint32_t serialBuffer[LIDAR_SERIAL_PACKET_SIZE] = {0};
    uint8_t cursorTmp = 0;

    PacketLidar lidarPacket;
    PacketLidar lidarLastPacket;
    ConfigLidar lidarConfig = {0, 0, 0, 0, 0};

   public:
    // Data
    vector<PolarPoint> scan;
    RobotPosition robotPosition;
    vector<PolarPoint> tracker;
};
#endif
