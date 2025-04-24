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
   public:
    void Initialisation();
    void Update();
    void HandleCommand(Command cmd);

    /**
     * Set the robotPosition with the Cartesian coordinates x, y and angle
     */
    void SetPosition(float x, float y, float angle);

    PoseF GetPosition();
    /**
     * Send data to robot : send obstacle position given in args
     * n : number, p : Cartesian Point
     */
    void WriteSerial(int n, PolarPoint p);
};

#endif
