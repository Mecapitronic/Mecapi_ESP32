#ifndef ESP32_HELPER_H
#define ESP32_HELPER_H

#include <Arduino.h>
#include "Structure.h"
#include "Printer.h"
#include "Debugger.h"

#define SERIAL_DEBUG Serial
#define SERIAL_DEBUG_SPEED 921600
#define READ_SERIAL_BUFFER_SIZE 64

// ESP32 Serial Bauds rates
// static const unsigned long default_rates[] = {300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 74880,
// 115200, 230400, 256000, 460800, 921600, 1843200, 3686400};

/**
 * Initialize serial for PC communication
 */
void ESP32_Helper();

/**
 * Check for commands send on debugging serial plugged to a computer
 * List of commands is still empty now
 */
static char* checkSerial();

#endif
