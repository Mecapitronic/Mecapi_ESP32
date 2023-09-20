#ifndef MAIN_H
#define MAIN_H

#include "ESP32_Helper.h"
#include "LD06.h"
#include "Robot.h"

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
int queueSize = 500;

Lidar lidar06;
Robot robot;
Tracker tracker;

Point obstacle;

#endif
