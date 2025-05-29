#include "Robot.h"

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
                // vérifions que la position du robot donnée par le ESP32S3
                // soit bien dans les limites de la table de jeu
                PoseF tempPosition;
                // int8_t header = readBuffer[0];
                tempPosition.x = readBuffer[2] << 8 | readBuffer[1];
                tempPosition.y = readBuffer[4] << 8 | readBuffer[3];
                position.h = readBuffer[6] << 8 | readBuffer[5];
                if (tempPosition.x > 0 && tempPosition.x < 3000 && tempPosition.y > 0 && tempPosition.y < 2000)
                {
                    position = tempPosition;
                }
                // int8_t footer = readBuffer[7];
                // println("Robot Position : ", position);
                readBuffer.clear();
            }
        }
        if (readBuffer.size() == readBufferCmd && c == 0x0A)
        {
            Command cmdTmp;
            cmdTmp.cmd = readBuffer[1] + readBuffer[2] + readBuffer[3];
            cmdTmp.size = 1;
            cmdTmp.data[0] = readBuffer[4];
            ESP32_Helper::HandleCommand(cmdTmp);
        }
    }
}

void Robot::HandleCommand(Command cmd)
{
    if (cmd.cmd == ("RXYA"))
    {
        if (cmd.size == 3)
        {
            // RobotXYA:1000;1500;9000
            position.x = cmd.data[0];
            position.y = cmd.data[1];
            position.h = cmd.data[2];
        }
        println("Robot Position : ", position);
    }
    else if (cmd.cmd == ("RPos"))
    {
        println("Robot Position : ", position);
        teleplot("Position", position);
        teleplot("Orient", position.h / 100);
    }
}

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
