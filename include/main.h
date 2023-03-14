#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>

#define SERIAL_PC Serial
#define SERIAL_LIDAR Serial2

#include <LinkedList.h>
#include "LD06.h"

// #include "LD06Driver.h"
void Task1code(void *pvParameters);
void Task2code(void *pvParameters);

#endif