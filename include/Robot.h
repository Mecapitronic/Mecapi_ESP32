/**
 * In charge of communication with the robot PIC
 * Receive robot position from robot PIC
 * Send objects/obstacles positions via serial communication
 */

#ifndef ROBOT_H
#define ROBOT_H

// Serial 1 : U1TX = GPIO9 ; U1RX = GPIO10
#define SERIAL_ROBOT Serial1

// we change the UART 1 RX pin from 9 to 2
// we change the UART 1 TX pin from 10 to 4
#define SERIAL_ROBOT_RX 2
#define SERIAL_ROBOT_TX 4

#include "ESP32_Helper.h"

using namespace Printer;

class Robot
{
   private:
    // '!' + "1000,1500,9000" + '\n' :  1 + 2 * 3 + 1;
    // 21 e8 03 dc 05 28 23 0A
    const int8_t readBufferMax = 8;
    // '!' + "cmd" + "data" + '\n' : 1 + 3 + 1 + 1
    // const int8_t readBufferCmd = 6;
    std::vector<char> readBuffer;

   public:
    PoseF position = {0.0, 0.0, 0.0};

    void Initialisation();
    void Update();
    void HandleCommand(Command cmd);

    /**
     * Send data to robot : send obstacle position given in args
     * n : number, p : Cartesian Point
     */
    void WriteSerial(int n, PolarPoint p);
};

#endif
