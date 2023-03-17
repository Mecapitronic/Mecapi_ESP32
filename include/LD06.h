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
    void Calc_lidar_data(LinkedList<uint32_t> &values);
    void Print_lidar_data(PointLidar data);
    void Read_lidar_data();
};

#endif
