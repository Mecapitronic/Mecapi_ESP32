#ifndef LD06_H
#define LD06_H

#include "main.h"

class LD06
{
private:
    // note: 12 data *3 Byte
    const int DATA_BYTE = 36;
    // note: 1(Start)+2(Datalen)+2(Speed)+2(SAngle)+36(DataByte)+2(EAngle)+2(TimeStamp)+1(CRC)
    const int TOTAL_DATA_BYTE = 48;

    const int PACKSIZE = 12;

public:
    const int MIN_DISTANCE = 20;
    const int MIN_QUALITY = 100;

public:
    struct PolarPoint
    {
        float angle;
        uint16_t dist;
        uint16_t conf;
        float x;
        float y;
    };

    struct PointLidar
    {
        double angle;
        int distance;
        int confidence;
        int timestamp;
    };

public:
    struct PacketLidar
    {
        byte header;
        int dataLength;
        int radarSpeed;
        int startAngle;

        PointLidar dataPoint[12];
        int endAngle;
        int timestamp;
        byte crcCheck;
    };

    // Initialize lidar obstacle
    static int const obs_length = 50;
    static int const max_data_obs = 20;
    static int const min_data_obs = 3;

public:
    struct Obstacle
    {
        PolarPoint data[LD06::max_data_obs];
        int dataT;
    } lidar_obstacle[LD06::obs_length];
    typedef struct Point
    {
        float x;
        float y;
    } Point;

public:
    void Init();

public:
    void Read_lidar_data();
    void Calc_lidar_data();
    void Filter_lidar_data();
    Point findCircle(Point p1, Point p2, Point p3);
    Point findCircle(float x1, float y1, float x2, float y2, float x3, float y3);
    void Print_lidar_data(PointLidar data, float x, float y);
};

#endif
