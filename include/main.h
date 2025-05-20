#ifndef MAIN_H
#define MAIN_H

#include "ESP32_Helper.h"
using namespace Printer;

void TaskSerial(void *pvParameters);
void TaskTeleplot(void *pvParameters);
void TaskCommand(void *pvParameters);

#ifdef AX12
#include <Dynamixel2Arduino.h>
// https://github.com/ROBOTIS-GIT/Dynamixel2Arduino/tree/master
// Please modify it to suit your hardware.
#define DXL_SERIAL Serial2  // Serial 2 : U2TX = GPIO17; U2RX = GPIO16
#define DXL_DIR_PIN 5
const uint8_t DXL_ID = 5;
const float DXL_PROTOCOL_VERSION = 1.0;

#define MAX_BAUD 5
const int32_t baud[MAX_BAUD] = {57600, 115200, 1000000, 2000000, 3000000};

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
#endif

#ifdef LD06
#warning "! Compiling for LD06 !"
#include "LD06.h"
#include "Robot.h"
#include "Tracker.h"
#include "testModule.h"
extern Robot robot;
#endif

#ifdef A010
#warning "! Compiling for MetaSenseA010 !"
#include "A010.h"
// #include "DBSCAN.h"
#endif

#ifdef VL53
#warning "! Compiling for VL53L5CX !"
#include "VL53L5CX.h"
#endif

void functionChrono(int nbrLoop = 1);
#endif
