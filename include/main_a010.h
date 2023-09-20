#ifndef MAIN_H
#define MAIN_H

#include "ESP32_Helper.h"
#include "A010.h"
#include "DBSCAN.h"

using namespace Printer;
/**
 * Get lidar data from serial
 * Get robot position from serial
 * Send lidar data in a queue for another thread to compute
 */
void Task1code(void *pvParameters);

/**
 * Get lidar data from queue and compute them in order to send obstacles positions to PIC
 */
void Task2code(void *pvParameters);

TaskHandle_t Task1;
TaskHandle_t Task2;

QueueHandle_t myQueue;
uint16_t queueSize = 500;

A010 a010;
Dbscan dbscan;

#endif
