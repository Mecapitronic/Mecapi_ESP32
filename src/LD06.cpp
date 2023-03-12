#include "LD06.h"

void LD06::Init() { SERIAL_LIDAR.begin(230400); }

void LD06::calc_lidar_data(LinkedList<uint32_t> &values)
{
    start_byte = values[0];
    data_length = 0x1F & values[1];
    Speed = float(values[3] << 8 | values[2]) / 100;
    FSA = float(values[5] << 8 | values[4]) / 100;
    LSA = float(values[values.size() - 4] << 8 | values[values.size() - 5]) / 100;
    time_stamp = int(values[values.size() - 2] << 8 | values[values.size() - 3]);
    CS = int(values[values.size() - 1]);

    if (LSA - FSA > 0)
    {
        angle_step = (LSA - FSA) / (data_length - 1);
    }
    else
    {
        angle_step = (LSA + (360 - FSA)) / (data_length - 1);
    }

    if (angle_step > 20)
    {
        return;
    }

    angles.clear();
    confidences.clear();
    distances.clear();

    for (int i = 0; i < data_length; i++)
    {
        float raw_deg = FSA + i * angle_step;
        angles.add(raw_deg <= 360 ? raw_deg : raw_deg - 360);
        confidences.add(values[8 + i * 3]);
        distances.add(int(values[8 + i * 3 - 1] << 8 | values[8 + i * 3 - 2]));

        SERIAL_PC.print((int)angles[i]);
        SERIAL_PC.print(";");
        SERIAL_PC.print((int)distances[i]);
        SERIAL_PC.print(";");
        SERIAL_PC.print((int)confidences[i]);
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
