#ifndef LD06_H
#define LD06_H

// #include "main.h"
#include <Arduino.h>

#include "LinkedList.h"

class LD06 {
   private:
    // note: 12 data *3 Byte
    const int DATA_BYTE = 36;
    // note: 1(Start)+2(Datalen)+2(Speed)+2(SAngle)+36(DataByte)+2(EAngle)+2(TimeStamp)+1(CRC)
    const int TOTAL_DATA_BYTE = 48;

    void calc_lidar_data(LinkedList<uint32_t> &values);

   public:
    LinkedList<float> angles;
    LinkedList<float> distances;
    LinkedList<int> confidences;

   private:
    char start_byte;
    char data_length;
    float Speed;
    float FSA;
    float LSA;
    int time_stamp;
    int CS;
    float angle_step;

   public:
    void Init();

   public:
    void Read_lidar_data();
};

#endif
