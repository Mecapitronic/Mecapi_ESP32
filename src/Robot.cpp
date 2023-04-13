#include "Robot.h"

Robot::Robot()
{
    robot_tbd = {0, 0, 0.0};
    cursorTmp = 0;
    for (size_t i = 0; i < ROBOT_SERIAL_PACKET_SIZE; i++)
    {
        tmpChars[i] = 0;
    }

    // we change the UART 1 RX pin from 9 to 2
    // we change the UART 1 TX pin from 10 to 4
    int8_t RX1 = 2;
    int8_t TX1 = 4;
    SERIAL_ROBOT.begin(125000, SERIAL_8N1, RX1, TX1);
}

Robot_t Robot::GetData() { return robot_tbd; }

boolean Robot::ReadSerial()
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

            if (cursorTmp >= ROBOT_DATA_PACKET_SIZE)
            {
                cursorTmp = 0;
                if (tmpChars[ROBOT_DATA_PACKET_SIZE - 1] == 10)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

void Robot::Analyze()
{
    int8_t header = tmpChars[0];
    robot_tbd.x = tmpChars[2] << 8 | tmpChars[1];
    robot_tbd.y = tmpChars[4] << 8 | tmpChars[3];
    robot_tbd.angle = tmpChars[6] << 8 | tmpChars[5];
    int8_t footer = tmpChars[7];
}

void Robot::Print()
{
    Debugger::log("X= ", robot_tbd.x, "  ", VERBOSE);
    Debugger::log("Y= ", robot_tbd.y, "  ", VERBOSE);
    Debugger::log("A= ", robot_tbd.angle / 100, "  ", VERBOSE);
}

void Robot::WriteSerial(int n, int x, int y)
{
    SERIAL_ROBOT.print(n);
    SERIAL_ROBOT.print(";");
    SERIAL_ROBOT.print(x);
    SERIAL_ROBOT.print(";");
    SERIAL_ROBOT.print(y);
    SERIAL_ROBOT.print('\n');
}
