#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>

#define SERIAL_PC Serial
#define SERIAL_LIDAR Serial2
#define SERIAL_ROBOT Serial1

#include "Structure.h"

#include "Debugger.h"
#include "LD06.h"

void Task1code(void* pvParameters);
void Task2code(void* pvParameters);

#endif