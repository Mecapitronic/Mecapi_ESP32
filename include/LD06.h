#ifndef LD06_H
#define LD06_H

#include "main.h"

namespace LD06
{

    // 47 = 1(Start) + 1(Datalen) + 2(Speed) + 2(StartAngle) + 36(12 * 3 DataByte) + 2(EndAngle) + 2(TimeStamp) + 1(CRC)
    uint8_t const serial_packet_size = 47;
    uint8_t const data_packet_size = 12;

    // Initialize obstacle
    int const obs_length = 10;

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
        PointLidar dataPoint[data_packet_size];
        int endAngle;
        int timestamp;
        byte crcCheck;
    };

    void Init();
    void ReadSerial();
    void Analyze();
    PacketLidar GetData();
    void Print();
    void AggregatePoint(PointLidar p);
    void ComputeCenter();

    Point findCircle(Point p1, Point p2, Point p3);
    Point findCircle(float x1, float y1, float x2, float y2, float x3, float y3);

} // namespace LD06

#endif
