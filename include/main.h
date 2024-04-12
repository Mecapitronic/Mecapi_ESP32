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

LidarLD06 ld06;
Robot robot;
Tracker tracker;
testModule test;

PolarPoint MapBoundaries[] = {{0, 0}, {0, 2000}, {3000, 2000}, {3000, 0}};

#endif

#ifdef A010
#warning "! Compiling for MetaSenseA010 !"

#include "A010.h"
#include "DBSCAN.h"

MetaSenseA010 a010;
Dbscan dbscan;

#endif

#ifdef VL53
#warning "! Compiling for VL53L5CX !"

// #include <Wire.h>

#include <SparkFun_VL53L5CX_Library.h>  //http://librarymanager/All#SparkFun_VL53L5CX

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData;  // Result data class structure, 1356 byes of RAM

int imageResolution = 0;  // Used to pretty print output
int imageWidth = 0;       // Used to pretty print output

long measurements = 0;          // Used to calculate actual output rate
long measurementStartTime = 0;  // Used to calculate actual output rate

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

QueueHandle_t myQueue;
int queueSize = 500;

#endif  // MAIN_H
