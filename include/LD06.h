/**
 * LidarLD06 manipulation and data computation
 * fetch data from LIDAR scans and compute filters to extract other robots positions
 * Computations are done on ESP32 for performances purposes
 */
#ifndef LD06_H
#define LD06_H

#include "ESP32_Helper.h"

#pragma region DEFINE
#define PWM_PIN 23
#define PWM_CHANNEL 0        // Choisit le canal 0
#define PWM_FREQUENCE 30000  // Fréquence PWM de 30 KHz
#define PWM_RESOLUTION 8     // Résolution de 8 bits, 256 valeurs possibles

// Serial 2 : U2TX = GPIO17 (Not Used for LidarLD06 LD06); U2RX = GPIO16
#define SERIAL_LIDAR Serial2
#define SERIAL_LIDAR_RX RX2
#define SERIAL_LIDAR_TX TX2
// 47 = 1(Start) + 1(Datalen) + 2(Speed) + 2(StartAngle) + 36(12 * 3 DataByte) + 2(EndAngle) + 2(TimeStamp) + 1(CRC)
#define LIDAR_SERIAL_PACKET_SIZE 47
#define LIDAR_DATA_PACKET_SIZE 12
// Maximum angle between Lidar LD06 packet admissible = angle * 100. Here 16° (2 packets)
#define ANGLE_MAX_DISCONTINUITY 160

// angular offset between robot and lidar, in 0,1°
// if the lidar and the robot have different origins
// positive in trigonometric way
#define LIDAR_ROBOT_ANGLE_OFFSET_DEG 90
#pragma endregion

using namespace Printer;

struct ConfigLidar
{
    int minDistance;        // represents the minimum distance of point from robot to be accepted
    int maxDistance;        // represents the maximum distance of point from robot to be accepted
    int minQuality;         // represents the minimum quality of a point to be accepted as good enough
    int distanceThreshold;  // represents the distance threshold to differentiate two obstacles
    int angleThreshold;     // represents the angle threshold to differentiate two obstacles
    int tableMargin;        // represent the distance between the edge of the table inside it
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

    void Print()
    {
        // Printer::println("Header : ", header);
        Printer::println("Data Length : ", dataLength);
        Printer::println("Radar Speed : ", radarSpeed);
        Printer::println("Start Angle : ", startAngle, " deg");
        Printer::println("End Angle : ", endAngle, " deg");
        /*for (size_t i = 0; i < dataLength; i++)
        {
            Printer::print("Data Point ", i);
            Printer::print(" : ", dataPoint[i].angle / 100, " deg ");
            Printer::print("Distance : ", dataPoint[i].distance, " mm ");
            Printer::println("Confidence : ", dataPoint[i].confidence);
        }*/
        int i = 0;
        Printer::print("Data Point ", i);
        Printer::print(" : ", dataPoint[i].angle / 100, " deg ");
        Printer::print("Distance : ", dataPoint[i].distance, " mm ");
        Printer::println("Confidence : ", dataPoint[i].confidence);
        i = dataLength - 1;
        Printer::print("Data Point ", i);
        Printer::print(" : ", dataPoint[i].angle / 100, " deg ");
        Printer::print("Distance : ", dataPoint[i].distance, " mm ");
        Printer::println("Confidence : ", dataPoint[i].confidence);

        Printer::print("Timestamp : ", timestamp);
        // Printer::print("CRC Check : ", crcCheck);
        Printer::println();
    }
};

/**
 * Represent a cluster of points, with the average of all point
 * The maximum points needed to represent a 80mm wide cylinder is 20
 */
struct Cluster
{
    std::vector<PolarPoint> data;
    PolarPoint mid;
    int index;
};

class LidarLD06
{
   public:
    ConfigLidar lidarConfig = {0, 0, 0, 0, 0, 0};
    PoseF robotPosition;
    std::vector<PolarPoint> clusterCenterPoints;

    void Initialisation();
    void Update();
    void HandleCommand(Command cmd);

    /**
     * @brief Configure lidarConfig local variable with the given values in parameters
     *
     * @param min (int) do not detect points closer than min distance (mm)
     * @param max (int) do not detect points further than max distance (mm)
     * @param quality (int) minimum confidence required to consider the detected point (0-255)
     * @param distance (int) distance threshold (mm) to change cluster
     * @param angle (int) angle threshold (°) to change cluster
     * @param tableMargin (int) distance inside table at the border (mm)
     */
    void Config(int min, int max, int quality, int distance, int angle, int tableMargin);

    /**
     * @brief Change duty cycle for the PWM
     *
     * @param duty_cycle (float) the duty cycle of PWM in percentage (20% to 50%)
     * @details Scan rate around 5.0  HZ when PWM duty at 21 %
     * @details Scan rate around 6.1  HZ when PWM duty at 25 %
     * @details Scan rate around 10.1 HZ when PWM duty at 39 %
     * @details Scan rate around 13.2 HZ when PWM duty at 50 %
     *
     */
    void ChangePWM(float duty_cycle);

    /**
     * @brief Return the duty cycle of the PWM
     *
     * @return float the duty cycle
     */
    float GetPWM();

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
     * @brief Check if the given packet is valid: end angle must be greater than start angle
     * @details verify the continuity of the packet by checking the end angle is greater
     * than the start angle and the overflow must not be more than 0.8°
     *
     * TODO check CRC
     * check number of data points
     *
     * @return boolean whether the packet is valid or not
     */
    boolean CheckPacket();

    /**
     * @brief Check the CRC of the received packet
     *
     * @return boolean whether the CRC is valid or not
     */
    boolean CheckCRC();

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

    void CheckCluster(PolarPoint polarPoint);

    void ObstacleDetected(Cluster& c);

    /**
     * Compute the center of the points aggregated
     * computes the mean of all points position to approximate circle center (without offset)
     * based on the fact that it is a cylinder of 80mm diameter
     */
    void ComputeCenter(Cluster& c);

    // counter of points while detecting an obstacle from data
    std::vector<Cluster> cluster;

    // why are you using uint32 instead of chars?
    uint32_t serialBuffer[LIDAR_SERIAL_PACKET_SIZE] = {0};
    uint8_t cursorTmp = 0;

    const uint8_t CrcTable[256] = {
        0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4,
        0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
        0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82,
        0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
        0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28,
        0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
        0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e,
        0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
        0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e,
        0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
        0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76,
        0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
        0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2,
        0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
        0x7f, 0x32, 0xe5, 0xa8};

    // TODO : Public To be removed !
   public:
    PacketLidar lidarPacket;
    PacketLidar lidarLastPacket;
};
#endif
