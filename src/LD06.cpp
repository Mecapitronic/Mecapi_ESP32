#include "LD06.h"

LinkedList<uint32_t> tmpChars;

void LD06::Init() { SERIAL_LIDAR.begin(230400); }

void LD06::Read_lidar_data()
{
    bool loopFlag = true;
    uint32_t tmpInt;

    while (loopFlag)
    {
        if (SERIAL_LIDAR.available() > 0)
        {
            tmpInt = SERIAL_LIDAR.read();

            if (tmpInt == 0x54 && tmpChars.size() == 0)
            {
                tmpChars.add(tmpInt);
                continue;
            }
            else if (tmpChars.size() == TOTAL_DATA_BYTE - 1)
            {
                loopFlag = false;
                Calc_lidar_data(tmpChars);
                tmpChars.clear();
                continue;
            }
            else if (tmpChars.size() > 0)
            {
                if (tmpChars[0] == 0x54)
                {
                    tmpChars.add(tmpInt);
                }
                if (tmpChars.size() > 1)
                {
                    if (tmpChars[1] != 0x2C)
                    {
                        tmpChars.clear();
                    }
                }
            }
        }
    }
}

LD06::PacketLidar data;
void LD06::Calc_lidar_data(LinkedList<uint32_t> &values)
{
    data.header = values[0];
    data.dataLength = 0x1F & values[1];
    data.radarSpeed = values[3] << 8 | values[2];
    data.startAngle = values[5] << 8 | values[4];

    data.endAngle = values[43] << 8 | values[42];
    data.timestamp = values[45] << 8 | values[44];
    data.crcCheck = values[46];

    float packetAngle = data.endAngle - data.startAngle;
    float angleStep = (packetAngle / float(PACKSIZE - 1)); // Calculate the angle step

    for (int i = 0; i < PACKSIZE; i++)
    {
        data.dataPoint[i].angle = float(data.startAngle) + i * angleStep;
        data.dataPoint[i].confidence = (values[8 + i * 3]);
        data.dataPoint[i].distance = (int(values[8 + i * 3 - 1] << 8 | values[8 + i * 3 - 2]));
        Print_lidar_data(data.dataPoint[i]);
    }
}

void LD06::Print_lidar_data(PointLidar data)
{
    SERIAL_PC.print(data.angle);
    SERIAL_PC.print(";");
    SERIAL_PC.print(data.distance);
    SERIAL_PC.print(";");
    SERIAL_PC.print(data.confidence);
    SERIAL_PC.print('\n');
}