/**
 * In charge of communication with the robot PIC
 * Receive robot position from robot PIC
 * Send objects/obstacles positions via serial communication
 */

#ifndef ROBOT_H
#define ROBOT_H

#define SERIAL_ROBOT Serial1
#define ROBOT_SERIAL_PACKET_SIZE 32
// '!' + "1000,1500,9000" + '\n' :  1 + 2 * 3 + 1;
// 21 e8 03 dc 05 28 23 0A
#define ROBOT_DATA_PACKET_SIZE 8

#include "Debugger.h"
#include "Structure.h"

class Robot
{
public:
    /**
     * Robot Constructor: init all variables and serial communication with PIC
     */
    Robot();

    /**
     * Read data coming from Robot PIC giving the actual position of the robot
     * put data in local buffer
     */
    boolean ReadSerial();

    /**
     * Register robot position received from PIC on serial
     * in a local variable robotPosition
     */
    void Analyze();

    /**
     * Return robotPosition: last known robot position
     */
    RobotPosition_t GetPosition();

    /**
     * Debug print: pretty print robot position and orientation
     */
    void PrintPosition();

    /**
     * Send data to robot PIC: send obstacle position given in args
     */
    void WriteSerial(int n, Point p);

private:
    RobotPosition_t robotPosition;
    uint32_t serialBuffer[ROBOT_SERIAL_PACKET_SIZE] = {0};
    uint8_t cursorTmp = 0;
};

#endif