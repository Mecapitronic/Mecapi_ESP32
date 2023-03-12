#include "LD06.h"

void LD06::Init() { SERIAL_LIDAR.begin(230400); }

void LD06::calc_lidar_data(LinkedList<uint32_t> &values)
{
    PacketLidar data;
    data.header = values[0];
    data.dataLength = 0x1F & values[1];
    data.radarSpeed = float(values[3] << 8 | values[2]) / 100;
    data.startAngle = float(values[5] << 8 | values[4]) / 100;
    data.endAngle = float(values[values.size() - 4] << 8 | values[values.size() - 5]) / 100;
    data.timestamp = int(values[values.size() - 2] << 8 | values[values.size() - 3]);
    data.crcCheck = int(values[values.size() - 1]);

    float packetAngle = data.endAngle - data.startAngle;
    float angleStep = (packetAngle / (data.dataLength - 1)); // Calculate the angle step

    if (data.endAngle - data.startAngle > 0)
    {
        angleStep = (data.endAngle - data.startAngle) / (data.dataLength - 1);
    }
    else
    {
        angleStep = (data.endAngle + (360 - data.startAngle)) / (data.dataLength - 1);
    }

    if (angleStep > 20)
    {
        return;
    }

    for (int i = 0; i < data.dataLength; i++)
    {
        float raw_deg = data.startAngle + i * angleStep;
        data.dataPoint[i].angle = (raw_deg <= 360 ? raw_deg : raw_deg - 360);
        data.dataPoint[i].confidence = (values[8 + i * 3]);
        data.dataPoint[i].distance = (int(values[8 + i * 3 - 1] << 8 | values[8 + i * 3 - 2]));

        SERIAL_PC.print((int)data.dataPoint[i].angle);
        SERIAL_PC.print(";");
        SERIAL_PC.print((int)data.dataPoint[i].distance);
        SERIAL_PC.print(";");
        SERIAL_PC.print((int)data.dataPoint[i].confidence);
        SERIAL_PC.print('\n');
    }
}

LinkedList<uint32_t> tmpChars;

void LD06::Read_lidar_data()
{
    if (!SERIAL_LIDAR.available() > 0)
    {
        return;
    }

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
                calc_lidar_data(tmpChars);
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
