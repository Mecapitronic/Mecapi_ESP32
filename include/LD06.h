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

public:
    void Init();

public:
    void Read_lidar_data();
    void Calc_lidar_data(LinkedList<uint32_t> &values);
    void Filter_lidar_data();
    void Print_lidar_data(PointLidar data, float x, float y);
};

#endif
