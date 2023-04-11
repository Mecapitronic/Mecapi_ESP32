#ifndef LD06_H
#define LD06_H

#include "main.h"

// 47 = 1(Start) + 1(Datalen) + 2(Speed) + 2(StartAngle) + 36(12 * 3 DataByte) + 2(EndAngle) + 2(TimeStamp) + 1(CRC)
#define TOTAL_DATA_BYTE 47
#define PACKET_SIZE 12

namespace LD06
{

// Initialize obstacle
int const obs_length = 50;
// int const obs_max_point = 20;
int const obs_min_point = 3;

struct PointLidar
{
    int angle;
    int distance;
    int confidence;
};

struct PacketLidar
{
    byte header;
    int dataLength;
    int radarSpeed;
    int startAngle;
    PointLidar dataPoint[PACKET_SIZE];
    int endAngle;
    int timestamp;
    byte crcCheck;
};

struct Obstacle
{
    static constexpr size_t kMaxPoints = 20;
    PolarPoint data[kMaxPoints];
    int size;
};

void Init();

void Read_lidar_data();
PacketLidar Calc_lidar_data();
void Filter_lidar_data(PointLidar p[], int size);

Point findCircle(Point p1, Point p2, Point p3);
Point findCircle(float x1, float y1, float x2, float y2, float x3, float y3);

}  // namespace LD06

#endif
