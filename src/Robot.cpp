#include "Robot.h"

void Robot::Initialisation()
{
    println("Init Robot");

    robotPosition = {1000, 1000, 0.0};
    print("Robot Position : ", robotPosition, "", LEVEL_INFO);
    cursorTmp = 0;
    for (size_t i = 0; i < ROBOT_SERIAL_PACKET_SIZE; i++)
    {
        serialBuffer[i] = 0;
    }
    dsPicSerial(ENABLE_TRUE);
}

void Robot::dsPicSerial(Enable enable)
{
    dsPicSerialStatus = enable;
    SERIAL_ROBOT.end();
    switch (dsPicSerialStatus)
    {
        case ENABLE_NONE:
            /* code */
            println("dsPic Serial Stop", LEVEL_INFO);
            break;
        case ENABLE_TRUE:
            println("dsPic Serial Start", LEVEL_INFO);
            SERIAL_ROBOT.begin(250000, SERIAL_8N1, RX1, TX1);
            break;
        case ENABLE_FALSE:
            println("dsPic Serial Debug", LEVEL_INFO);
            SERIAL_ROBOT.begin(230400, SERIAL_8N1, RX1, TX1);
            break;

        default:
            break;
    }
}
Enable Robot::dsPicSerial() { return dsPicSerialStatus; }

RobotPosition Robot::GetPosition() { return robotPosition; }

void Robot::SetPosition(int x, int y, int angle)
{
    robotPosition.x = x;
    robotPosition.y = y;
    robotPosition.angle = angle;
}

boolean Robot::ReadSerial()
{
    if (dsPicSerialStatus != ENABLE_NONE)
{
    if (SERIAL_ROBOT.available() > 0)
    {
        uint32_t tmpInt = SERIAL_ROBOT.read();
        if (tmpInt == 0x21 && cursorTmp == 0) // 0x21 = '!'
        {
            serialBuffer[cursorTmp++] = tmpInt;
        }
        else if (cursorTmp > 0)
        {
            serialBuffer[cursorTmp++] = tmpInt;

            if (cursorTmp >= ROBOT_DATA_PACKET_SIZE)
            {
                cursorTmp = 0;
                if (serialBuffer[ROBOT_DATA_PACKET_SIZE - 1] == 10)
                {
                    return true;
                    }
                }
            }
        }
    }
    return false;
}

void Robot::Analyze()
{
    int8_t header = serialBuffer[0];
    robotPosition.x = serialBuffer[2] << 8 | serialBuffer[1];
    robotPosition.y = serialBuffer[4] << 8 | serialBuffer[3];
    robotPosition.angle = serialBuffer[6] << 8 | serialBuffer[5];
    int8_t footer = serialBuffer[7];
}

void Robot::WriteSerialdsPic(int n, Point p)
{
    if (dsPicSerialStatus == ENABLE_TRUE)
    {
        // Starting char : '!'
        SERIAL_ROBOT.write(0x21);

        // Number
        SERIAL_ROBOT.write(n);

        // X
        int x = (int)p.x;
        SERIAL_ROBOT.write(x % 256);
        SERIAL_ROBOT.write(x >> 8);

        // Y
        int y = (int)p.y;
        SERIAL_ROBOT.write(y % 256);
        SERIAL_ROBOT.write(y >> 8);

        // Ending char : 'LF'
        SERIAL_ROBOT.write(10);
    }
}