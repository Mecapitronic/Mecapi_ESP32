#include "Robot.h"

namespace Robot
{

    Robot_t robot;
    uint32_t tmpChars[serial_packet_size];
    uint8_t cursorTmp = 0;

    void Init()
    {
        robot = {0, 0, 0.0};
        cursorTmp = 0;
        for (size_t i = 0; i < serial_packet_size; i++)
        {
            tmpChars[i] = 0;
        }

        // we change the UART 1 RX pin from 9 to 2
        // we change the UART 1 TX pin from 10 to 4
        int8_t RX1 = 2;
        int8_t TX1 = 4;
        SERIAL_ROBOT.begin(125000, SERIAL_8N1, RX1, TX1);
    }

    void ReadSerial()
    {
        if (SERIAL_ROBOT.available() > 0)
        {
            uint32_t tmpInt = SERIAL_ROBOT.read();
            if (tmpInt == 0x21 && cursorTmp == 0) // 0x21 = '!'
            {
                tmpChars[cursorTmp++] = tmpInt;
            }
            else if (cursorTmp > 0)
            {
                tmpChars[cursorTmp++] = tmpInt;

                if (cursorTmp >= data_packet_size)
                {
                    cursorTmp = 0;
                    if (tmpChars[data_packet_size - 1] == 10)
                    {
                        Analyze();
                    }
                }
            }
        }
    }

    void Analyze()
    {
        int8_t header = tmpChars[0];
        robot.x = tmpChars[2] << 8 | tmpChars[1];
        robot.y = tmpChars[4] << 8 | tmpChars[3];
        robot.angle = tmpChars[6] << 8 | tmpChars[5];
        int8_t footer = tmpChars[7];
    }

    Robot_t GetData() { return robot; }

    void Print()
    {
        SERIAL_PC.print("X= ");
        SERIAL_PC.print(robot.x);
        SERIAL_PC.print("  Y= ");
        SERIAL_PC.print(robot.y);
        SERIAL_PC.print("  A= ");
        SERIAL_PC.println(robot.angle / 100);
    }
} // namespace Robot