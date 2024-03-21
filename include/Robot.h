/**
 * In charge of communication with the robot PIC
 * Receive robot position from robot PIC
 * Send objects/obstacles positions via serial communication
 */

#ifndef ROBOT_H
#define ROBOT_H

#define SERIAL_ROBOT Serial1

// we change the UART 1 RX pin from 9 to 2
// we change the UART 1 TX pin from 10 to 4
#define RX1 2
#define TX1 4

#define ROBOT_SERIAL_PACKET_SIZE 32
// '!' + "1000,1500,9000" + '\n' :  1 + 2 * 3 + 1;
// 21 e8 03 dc 05 28 23 0A
#define ROBOT_DATA_PACKET_SIZE 8

#include "ESP32_Helper.h"

using namespace Printer;

class Robot  // : public IModule
{
   public:
    /**
     * Robot Init : init all variables and serial communication with PIC
     */
    void Initialisation();
    void Update();

    /**
     * Enable/disable communication to dsPIC : used for debug as it re-route serial to PC
     */
    void dsPicSerial(Enable enable);
    Enable dsPicSerial();
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
    RobotPosition GetPosition();

    /**
     * Set the robotPosition with the Cartesian coordinates x, y and angle
     */
    void SetPosition(int x, int y, int angle);

    /**
     * Send data to robot : send obstacle position given in args
     * n : number, p : Cartesian Point
     */
    void WriteSerial(int n, Point p);

   private:
    RobotPosition robotPosition = {0, 0, 0.0};  // x, y, angle
    uint32_t serialBuffer[ROBOT_SERIAL_PACKET_SIZE] = {0};
    uint8_t cursorTmp = 0;
    Enable dsPicSerialStatus = ENABLE_TRUE;
};

#endif
