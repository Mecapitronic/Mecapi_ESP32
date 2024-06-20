#ifndef MAIN_H
#define MAIN_H

#include "ESP32_Helper.h"
using namespace Printer;

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

/**
 * Get data from serial
 * Get robot position from serial
 * Send data in a queue for the other thread to compute
 */
void Task1code(void *pvParameters);

/**
 * Get data from queue and compute them
 */
void Task2code(void *pvParameters);

TaskHandle_t Task1;
TaskHandle_t Task2;

#endif  // MAIN_H
