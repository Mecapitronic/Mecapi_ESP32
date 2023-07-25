/**
 * @file ESP32_Helper.h
 * @author Mecapitronic (mecapitronic@gmail.com)
 * @brief
 * @date 2023-07-25
 */
#ifndef ESP32_HELPER_H
#define ESP32_HELPER_H

#include <Arduino.h>
#include "Structure.h"
#include "Printer.h"
#include "Debugger.h"

#define SERIAL_DEBUG Serial

namespace ESP32_Helper
{
const int Serial_Read_Buffer = 64;

// ESP32 Serial Bauds rates
// static const unsigned long default_rates[] = {300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 74880,
// 115200, 230400, 256000, 460800, 921600, 1843200, 3686400};

/**
 * Initialize serial for PC communication
 */
void ESP32_Helper(int baud_speed = 921600);

/**
 * Check for commands send on debugging serial plugged to a computer
 * List of commands is still empty now
 */
char* checkSerial();

}  // namespace ESP32_Helper
#endif
