#include "Robot.h"

Robot::Robot()
{
    robot_position = {0, 0, 0.0};
    cursorTmp = 0;
    for (size_t i = 0; i < ROBOT_SERIAL_PACKET_SIZE; i++)
    {
        serial_buffer[i] = 0;
    }

    // we change the UART 1 RX pin from 9 to 2
    // we change the UART 1 TX pin from 10 to 4
    int8_t RX1 = 2;
    int8_t TX1 = 4;
    SERIAL_ROBOT.begin(125000, SERIAL_8N1, RX1, TX1);
}

RobotPosition_t Robot::GetData() { return robot_position; }

boolean Robot::ReadSerial()
{
    if (SERIAL_ROBOT.available() > 0)
    {
        uint32_t tmpInt = SERIAL_ROBOT.read();
        if (tmpInt == 0x21 && cursorTmp == 0) // 0x21 = '!'
        {
            serial_buffer[cursorTmp++] = tmpInt;
        }
        else if (cursorTmp > 0)
        {
            serial_buffer[cursorTmp++] = tmpInt;

            if (cursorTmp >= ROBOT_DATA_PACKET_SIZE)
            {
                cursorTmp = 0;
                if (serial_buffer[ROBOT_DATA_PACKET_SIZE - 1] == 10)
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
    int8_t header = serial_buffer[0];
    robot_position.x = serial_buffer[2] << 8 | serial_buffer[1];
    robot_position.y = serial_buffer[4] << 8 | serial_buffer[3];
    robot_position.angle = serial_buffer[6] << 8 | serial_buffer[5];
    int8_t footer = serial_buffer[7];
}

void Robot::Print()
{
    Debugger::log("X= ", robot_position.x, "  ", VERBOSE);
    Debugger::log("Y= ", robot_position.y, "  ", VERBOSE);
    Debugger::log("A= ", robot_position.angle / 100, "  ", VERBOSE);
}

void Robot::WriteSerial(int n, Point p)
{
    SERIAL_ROBOT.print(n);
    SERIAL_ROBOT.print(";");
    SERIAL_ROBOT.print(p.x);
    SERIAL_ROBOT.print(";");
    SERIAL_ROBOT.print(p.y);
    SERIAL_ROBOT.print('\n');
}
