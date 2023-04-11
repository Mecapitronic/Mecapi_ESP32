#include "Robot.h"

namespace Robot
{

Robot_t robot;
uint32_t tmpChars[32];
uint8_t cursorTmp = 0;
uint8_t const data_size = 16 + 1;
// "!2000,2000,36000" + 'n'

void Init()
{
    robot = {0, 0, 0.0};
    cursorTmp = 0;
    for (size_t i = 0; i < 32; i++)
    {
        tmpChars[i] = 0;
    }

    // we change the UART 1 RX pin from 9 to 2
    // we change the UART 1 TX pin from 10 to 4
    int8_t RX1 = 2;
    int8_t TX1 = 4;
    SERIAL_ROBOT.begin(125000, SERIAL_8N1, RX1, TX1);
}

boolean ReadSerial()
{
    if (SERIAL_ROBOT.available() > 0)
    {
        uint32_t tmpInt = SERIAL_ROBOT.read();
        if (tmpInt == 0x21 && cursorTmp == 0)  // 0x21 = '!'
        {
            tmpChars[cursorTmp++] = tmpInt;
        }
        else if (cursorTmp > 0)
        {
            tmpChars[cursorTmp++] = tmpInt;

            if (cursorTmp >= data_size)
            {
                cursorTmp = 0;
                if (tmpChars[data_size - 1] == 10)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

void AnalyseData() {}
}  // namespace Robot