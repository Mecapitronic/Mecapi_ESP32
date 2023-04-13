#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include "Debugger.h"
#include "LD06.h"
#include "Robot.h"
#include "Structure.h"

void Task1code(void *pvParameters);
void Task2code(void *pvParameters);

TaskHandle_t Task1;
TaskHandle_t Task2;

QueueHandle_t queue;
int queueSize = 500;

const int MIN_DISTANCE = 0;
const int MAX_DISTANCE = 500;
const int MIN_QUALITY = 100;

Lidar lidar06;
Robot robot;

#endif