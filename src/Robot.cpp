#include "Robot.h"

// '!' + "1000,1500,9000" + '\n' :  1 + 2 * 3 + 1;
// 21 e8 03 dc 05 28 23 0A
const int8_t readBufferMax = 8;
std::vector<char> readBuffer;
PoseF position = {0.0, 0.0, 0.0};

void Robot::Initialisation()
{
    println("Init Robot");

    position = {1500.0, 1000.0, 0.0};
    println("Robot Position : ", position, "", Level::LEVEL_INFO);

    // + 1 in case ...
    readBuffer.reserve(readBufferMax + 1);
    readBuffer.clear();
    SERIAL_ROBOT.end();
    SERIAL_ROBOT.setPins(SERIAL_ROBOT_RX, SERIAL_ROBOT_TX);
    SERIAL_ROBOT.setRxBufferSize(1024);
    SERIAL_ROBOT.setTxBufferSize(1024);
    SERIAL_ROBOT.begin(230400);
}

void Robot::Update()
{
    while (SERIAL_ROBOT.available() > 0)
    {
        char c = SERIAL_ROBOT.read();

        // Check Overflow
        if (readBuffer.size() >= readBufferMax)
            readBuffer.clear();

        readBuffer.push_back(c);

        // Check start of packet
        if (readBuffer[0] != 0x21)  // 0x21 = '!'
        {
            readBuffer.clear();
            continue;
        }

        // Check packet length
        if (readBuffer.size() == readBufferMax)
        {
            // Check end of packet
            if (c != 0x0A)  // 0x0A = '\n'
            {
                readBuffer.clear();
                continue;
            }
            else
            {
                // int8_t header = readBuffer[0];
                position.x = readBuffer[2] << 8 | readBuffer[1];
                position.y = readBuffer[4] << 8 | readBuffer[3];
                float deg = readBuffer[6] << 8 | readBuffer[5];
                position.h = radians(deg / 100.0);
                // int8_t footer = readBuffer[7];
                // println("Robot Position : ", position);
                readBuffer.clear();
            }
        }
    }
}

void Robot::HandleCommand(Command cmd)
{
    if (cmd.cmd == ("RXYA") && cmd.size == 3)
    {
        // RobotXYA:1000;1500;00000
        SetPosition(cmd.data[0], cmd.data[1], cmd.data[2]);
        println("Robot Position : ", position);
    }
    else if (cmd.cmd == ("RPos"))
    {
        println("Robot Position : ", position);
        teleplot("Position", position);
        teleplot("Orient", position.h);
    }
}

void Robot::SetPosition(float x, float y, float angle)
{
    position.x = x;
    position.y = y;
    position.h = angle;
}

PoseF Robot::GetPosition() { return position; }

void Robot::WriteSerial(int n, PolarPoint p)
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

    // Ending char : '\n'
    SERIAL_ROBOT.write(0x0A);
}
