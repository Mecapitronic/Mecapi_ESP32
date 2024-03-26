#include "Robot.h"

void Robot::Initialisation()
{
    println("Init Robot");

    //! TODO put robot in middle of map at start ?
    position = {0.0, 0, 0, 0.0, 0.0};
    print("Robot Position : ", position, "", LEVEL_INFO);
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

PolarPoint Robot::GetPosition() { return position; }

void Robot::SetPosition(int x, int y, int angle)
{
    position.x = x;
    position.y = y;
    position.angle = angle;
}

void Robot::Update()
{
    if (ReadSerial())
    {
        Analyze();
        plotRobot(GetPosition());
    }
}

boolean Robot::ReadSerial()
{
    if (dsPicSerialStatus != ENABLE_NONE)
    {
        if (SERIAL_ROBOT.available() > 0)
        {
            uint32_t tmpInt = SERIAL_ROBOT.read();
            if (tmpInt == 0x21 && cursorTmp == 0)  // 0x21 = '!'
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
    position.x = serialBuffer[2] << 8 | serialBuffer[1];
    position.y = serialBuffer[4] << 8 | serialBuffer[3];
    position.angle = serialBuffer[6] << 8 | serialBuffer[5];
    int8_t footer = serialBuffer[7];
}

void Robot::WriteSerial(int n, PolarPoint p)
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
