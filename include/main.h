#ifndef MAIN_H
#define MAIN_H

#include "ESP32_Helper.h"

using namespace Printer;

#ifdef LD06
#warning "! Compiling for LD06 !"

#include "LD06.h"
#include "Robot.h"
#include "testModule.h"

LidarLD06 ld06;
Robot robot;
Tracker tracker;
Point obstacle;
testModule test;

#endif

#ifdef A010
#warning "! Compiling for MetaSenseA010 !"

#include "A010.h"
#include "DBSCAN.h"

MetaSenseA010 a010;
Dbscan dbscan;

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

#endif
