/**
 * In charge of communication with the robot PIC
 * Send objects/obstacles positions via serial connexion
 * Can get data from robot PIC
 */

#ifndef ROBOT_H
#define ROBOT_H

#define SERIAL_ROBOT Serial1
#define ROBOT_SERIAL_PACKET_SIZE 32
// '!' + "2000,2000,36000" + 'n' :  1 + 2 * 3 + 1;
#define ROBOT_DATA_PACKET_SIZE 8

#include "Debugger.h"
#include "Structure.h"

class Robot
{
public:
    Robot();
    boolean ReadSerial();
    void Analyze();
    Robot_t GetData();
    void Print();
    void WriteSerial(int n, int x, int y);

private:
    Robot_t robot_tbd;
    uint32_t tmpChars[ROBOT_SERIAL_PACKET_SIZE];
    uint8_t cursorTmp = 0;
};

#endif