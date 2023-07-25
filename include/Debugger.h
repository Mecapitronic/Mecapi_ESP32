/**
 * Helper functions to debug via serial
 */

#ifndef DEBUGGER_H
#define DEBUGGER_H

#include "ESP32_Helper.h"

class Debugger
{
   private:
    static const boolean enabled = true;

   public:
    /**
     * Print custom header on debugging serial
     */
    static void header();
    /**
     * Initialize debugging serial for PC communication
     */
    static void init();

    /**
     * Check for commands send on debugging serial plugged to a computer
     * List of commands is still empty now
     */
    static char* checkSerial();

    static bool waitForAvailableSteps();
    static void addSteps(uint16_t steps);
    static void subSteps(uint16_t steps = 1);
};
#endif
